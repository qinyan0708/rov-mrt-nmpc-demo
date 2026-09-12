import math
import time

import casadi as ca
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class NmpcControllerNode(Node):

    def __init__(self):
        super().__init__('nmpc_controller_node')

        self.ts = 0.30
        self.horizon = 14
        self.length = 3.0
        self.reference_speed = 0.80

        self.v_min = 0.20
        self.v_max = 1.40
        self.delta_max = math.radians(42.0)
        self.a_max = 0.60
        self.omega_delta_max = math.radians(24.0)

        self.path_x = np.linspace(0.0, 48.0, 961)
        self.path_y = 1.2 * np.sin(
            2.0 * np.pi * self.path_x / 24.0
        )
        self.path_s = np.concatenate((
            [0.0],
            np.cumsum(np.hypot(
                np.diff(self.path_x),
                np.diff(self.path_y),
            )),
        ))
        dx_ds = np.gradient(self.path_x, self.path_s)
        dy_ds = np.gradient(self.path_y, self.path_s)
        self.path_psi = np.unwrap(np.arctan2(dy_ds, dx_ds))
        self.path_length = float(self.path_s[-1])

        self.s_progress = 0.0
        self.u_previous = np.zeros(2)
        self.w_guess = np.zeros((3, self.horizon))
        self.w_guess[2, :] = self.reference_speed

        self.control_publisher = self.create_publisher(
            Float64MultiArray,
            '/rov_mrt/control',
            10,
        )
        self.prediction_publisher = self.create_publisher(
            Path,
            '/predicted_path',
            10,
        )
        self.solve_time_publisher = self.create_publisher(
            Float64,
            '/nmpc/solve_time',
            10,
        )

        self.get_logger().info('Building CasADi NMPC solver...')
        self.build_solver()

        self.state_subscription = self.create_subscription(
            Float64MultiArray,
            '/rov_mrt/state',
            self.state_callback,
            10,
        )

        self.get_logger().info(
            'CasADi NMPC road-tracking controller ready.'
        )

    def vehicle_dynamics(self, state, control):
        return ca.vertcat(
            state[3] * ca.cos(state[2]),
            state[3] * ca.sin(state[2]),
            state[3] / self.length * ca.tan(state[4]),
            control[0],
            control[1],
        )

    def rk4_symbolic(self, state, control):
        k1 = self.vehicle_dynamics(state, control)
        k2 = self.vehicle_dynamics(
            state + 0.5 * self.ts * k1,
            control,
        )
        k3 = self.vehicle_dynamics(
            state + 0.5 * self.ts * k2,
            control,
        )
        k4 = self.vehicle_dynamics(
            state + self.ts * k3,
            control,
        )

        next_state = state + self.ts * (
            k1 + 2.0 * k2 + 2.0 * k3 + k4
        ) / 6.0

        return ca.vertcat(
            next_state[0],
            next_state[1],
            ca.atan2(ca.sin(next_state[2]), ca.cos(next_state[2])),
            next_state[3],
            ca.atan2(ca.sin(next_state[4]), ca.cos(next_state[4])),
        )

    def build_solver(self):
        x_reference = ca.interpolant(
            'x_reference',
            'linear',
            [self.path_s],
            self.path_x,
        )
        y_reference = ca.interpolant(
            'y_reference',
            'linear',
            [self.path_s],
            self.path_y,
        )
        psi_reference = ca.interpolant(
            'psi_reference',
            'linear',
            [self.path_s],
            self.path_psi,
        )

        decision = ca.SX.sym(
            'decision',
            3 * self.horizon,
        )
        extended_control = ca.reshape(
            decision,
            3,
            self.horizon,
        )

        parameter = ca.SX.sym('parameter', 8)
        state = parameter[0:5]
        progress = parameter[5]
        previous_control = parameter[6:8]

        objective = 0
        constraints = []
        predicted_states = [state]
        predicted_progress = [progress]

        for step in range(self.horizon):
            reference_x = x_reference(progress)
            reference_y = y_reference(progress)
            reference_psi = psi_reference(progress)

            difference_x = state[0] - reference_x
            difference_y = state[1] - reference_y

            contour_error = (
                -ca.sin(reference_psi) * difference_x
                + ca.cos(reference_psi) * difference_y
            )
            lag_error = (
                ca.cos(reference_psi) * difference_x
                + ca.sin(reference_psi) * difference_y
            )
            heading_error = ca.atan2(
                ca.sin(state[2] - reference_psi),
                ca.cos(state[2] - reference_psi),
            )

            control = extended_control[0:2, step]
            path_speed = extended_control[2, step]

            if step == 0:
                control_change = control - previous_control
            else:
                control_change = (
                    control
                    - extended_control[0:2, step - 1]
                )

            objective += (
                32.0 * contour_error ** 2
                + 8.0 * lag_error ** 2
                + 12.0 * heading_error ** 2
                + 12.0 * (state[3] - self.reference_speed) ** 2
                + 0.8 * control[0] ** 2
                + 0.5 * control[1] ** 2
                + 1.2 * control_change[0] ** 2
                + 0.8 * control_change[1] ** 2
                - 8.0 * self.ts * path_speed
            )

            constraints.extend([state[3], state[4]])

            state = self.rk4_symbolic(state, control)
            progress = ca.fmin(
                self.path_length,
                progress + self.ts * path_speed,
            )
            predicted_states.append(state)
            predicted_progress.append(progress)

        reference_x = x_reference(progress)
        reference_y = y_reference(progress)
        reference_psi = psi_reference(progress)

        difference_x = state[0] - reference_x
        difference_y = state[1] - reference_y

        contour_error = (
            -ca.sin(reference_psi) * difference_x
            + ca.cos(reference_psi) * difference_y
        )
        lag_error = (
            ca.cos(reference_psi) * difference_x
            + ca.sin(reference_psi) * difference_y
        )
        heading_error = ca.atan2(
            ca.sin(state[2] - reference_psi),
            ca.cos(state[2] - reference_psi),
        )

        objective += (
            120.0 * contour_error ** 2
            + 8.0 * lag_error ** 2
            + 55.0 * heading_error ** 2
            + 6.0 * (state[3] - self.reference_speed) ** 2
        )

        constraints.extend([state[3], state[4]])

        problem = {
            'x': decision,
            'p': parameter,
            'f': objective,
            'g': ca.vertcat(*constraints),
        }

        options = {
            'ipopt.print_level': 0,
            'ipopt.max_iter': 60,
            'ipopt.tol': 1.0e-4,
            'print_time': False,
        }

        self.solver = ca.nlpsol(
            'road_tracking_solver',
            'ipopt',
            problem,
            options,
        )

        self.trajectory_function = ca.Function(
            'trajectory_function',
            [decision, parameter],
            [ca.hcat(predicted_states)],
        )

        self.lower_decision = np.tile(
            [
                -self.a_max,
                -self.omega_delta_max,
                self.v_min,
            ],
            self.horizon,
        )
        self.upper_decision = np.tile(
            [
                self.a_max,
                self.omega_delta_max,
                self.v_max,
            ],
            self.horizon,
        )
        self.lower_constraint = np.tile(
            [self.v_min, -self.delta_max],
            self.horizon + 1,
        )
        self.upper_constraint = np.tile(
            [self.v_max, self.delta_max],
            self.horizon + 1,
        )

    def publish_prediction(self, prediction):
        path_message = Path()
        path_message.header.frame_id = 'map'
        path_message.header.stamp = self.get_clock().now().to_msg()

        for step in range(prediction.shape[1]):
            pose = PoseStamped()
            pose.header = path_message.header
            pose.pose.position.x = float(prediction[0, step])
            pose.pose.position.y = float(prediction[1, step])
            pose.pose.position.z = 0.10

            heading = float(prediction[2, step])
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            path_message.poses.append(pose)

        self.prediction_publisher.publish(path_message)

    def state_callback(self, message):
        if len(message.data) != 5:
            return

        current_state = np.asarray(message.data, dtype=float)
        parameter = np.concatenate((
            current_state,
            [self.s_progress],
            self.u_previous,
        ))

        start_time = time.perf_counter()

        try:
            solution = self.solver(
                x0=self.w_guess.flatten(order='F'),
                p=parameter,
                lbx=self.lower_decision,
                ubx=self.upper_decision,
                lbg=self.lower_constraint,
                ubg=self.upper_constraint,
            )
            success = bool(self.solver.stats()['success'])
        except RuntimeError as error:
            self.get_logger().error(str(error))
            success = False

        solve_time = time.perf_counter() - start_time
        timing_message = Float64()
        timing_message.data = solve_time
        self.solve_time_publisher.publish(timing_message)

        if not success:
            stop_message = Float64MultiArray()
            stop_message.data = [0.0, 0.0]
            self.control_publisher.publish(stop_message)
            self.get_logger().warning('NMPC solve failed.')
            return

        optimal_decision = np.asarray(
            solution['x'],
            dtype=float,
        ).reshape((3, self.horizon), order='F')

        applied_control = optimal_decision[0:2, 0]

        control_message = Float64MultiArray()
        control_message.data = applied_control.tolist()
        self.control_publisher.publish(control_message)

        prediction = np.asarray(
            self.trajectory_function(
                optimal_decision.flatten(order='F'),
                parameter,
            ),
            dtype=float,
        )
        self.publish_prediction(prediction)

        self.s_progress = min(
            self.path_length,
            self.s_progress
            + self.ts * optimal_decision[2, 0],
        )
        self.u_previous = applied_control.copy()

        self.w_guess[:, :-1] = optimal_decision[:, 1:]
        self.w_guess[:, -1] = optimal_decision[:, -1]


def main(args=None):
    rclpy.init(args=args)
    node = NmpcControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
