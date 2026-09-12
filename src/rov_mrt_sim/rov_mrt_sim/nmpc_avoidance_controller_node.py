import math
import time

import casadi as ca
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from rov_mrt_sim.model import braking_control


class NmpcAvoidanceControllerNode(Node):
    """CasADi port of the MATLAB opposite-side two-obstacle NMPC demo."""

    def __init__(self):
        super().__init__('nmpc_avoidance_controller_node')
        defaults = {
            'sample_time': 0.30,
            'horizon': 14,
            'vehicle_length': 3.00,
            'rov_offset': 1.20,
            'reference_speed': 0.80,
            'speed_min': 0.20,
            'speed_max': 1.40,
            'steering_max_deg': 42.0,
            'acceleration_max': 0.60,
            'steering_rate_max_deg_s': 24.0,
            'mrt_circle_offsets': [-0.90, 0.55, 2.00],
            'mrt_circle_radius': 0.95,
            'rov_circle_offsets': [-0.45, 0.45],
            'rov_circle_radius': 0.80,
            'safety_padding': 0.50,
            'hard_margin': 0.10,
            'slack_max': 0.08,
            'path_amplitude': 1.20,
            'path_wavelength': 24.0,
            'path_x_end': 48.0,
            'path_points': 961,
            'obstacle_path_x': [14.0, 34.0],
            'corridor_amplitude': 3.50,
            'corridor_lead': 8.00,
            'corridor_front_buffer': 0.50,
            'corridor_obstacle_tail': 1.50,
            'corridor_release': 8.00,
            'solver_max_iterations': 60,
            'solver_max_cpu_time': 0.25,
            'solver_tolerance': 1.0e-4,
            'acceptable_violation': 1.0e-3,
            'emergency_deceleration': 0.60,
            'emergency_steering_gain': 2.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def value(name):
            return self.get_parameter(name).value

        self.ts = float(value('sample_time'))
        self.horizon = int(value('horizon'))
        self.length = float(value('vehicle_length'))
        self.rov_offset = float(value('rov_offset'))
        self.reference_speed = float(value('reference_speed'))
        self.v_min = float(value('speed_min'))
        self.v_max = float(value('speed_max'))
        self.delta_max = math.radians(float(value('steering_max_deg')))
        self.a_max = float(value('acceleration_max'))
        self.omega_delta_max = math.radians(
            float(value('steering_rate_max_deg_s'))
        )
        self.rho_b = np.asarray(value('mrt_circle_offsets'), dtype=float)
        self.radius_b = float(value('mrt_circle_radius'))
        self.rho_r = np.asarray(value('rov_circle_offsets'), dtype=float)
        self.radius_r = float(value('rov_circle_radius'))
        self.d_safe = float(value('safety_padding'))
        self.hard_margin = float(value('hard_margin'))
        self.epsilon_max = float(value('slack_max'))
        self.solver_max_iterations = int(value('solver_max_iterations'))
        self.solver_max_cpu_time = float(value('solver_max_cpu_time'))
        self.solver_tolerance = float(value('solver_tolerance'))
        self.acceptable_violation = float(value('acceptable_violation'))
        self.emergency_deceleration = float(value('emergency_deceleration'))
        self.emergency_steering_gain = float(value('emergency_steering_gain'))

        path_points = int(value('path_points'))
        path_x_end = float(value('path_x_end'))
        amplitude = float(value('path_amplitude'))
        wavelength = float(value('path_wavelength'))
        self.path_x = np.linspace(0.0, path_x_end, path_points)
        self.path_y = amplitude * np.sin(2.0 * np.pi * self.path_x / wavelength)
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

        obstacle_stations = []
        obstacle_path_x = list(value('obstacle_path_x'))
        if len(obstacle_path_x) != 2:
            raise ValueError('obstacle_path_x must contain exactly two values.')
        for obstacle_x in obstacle_path_x:
            index = int(np.argmin(np.abs(self.path_x - obstacle_x)))
            obstacle_stations.append(float(self.path_s[index]))

        front_offset = self.length + self.rov_offset + max(self.rho_r)
        front_buffer = float(value('corridor_front_buffer'))
        tail = float(value('corridor_obstacle_tail'))
        self.profile_s0 = (
            obstacle_stations[0] - front_offset - front_buffer
            - float(value('corridor_lead'))
        )
        self.profile_s1 = obstacle_stations[0] - front_offset - front_buffer
        self.profile_s2 = obstacle_stations[0] + tail
        self.profile_s3 = obstacle_stations[1] - front_offset - front_buffer
        self.profile_s4 = obstacle_stations[1] + tail
        self.profile_s5 = self.profile_s4 + float(value('corridor_release'))
        self.offset_amplitude = float(value('corridor_amplitude'))

        self.s_progress = 0.0
        self.u_previous = np.zeros(2)
        self.w_guess = np.zeros((3, self.horizon))
        self.w_guess[2, :] = self.reference_speed
        self.epsilon_guess = np.zeros(self.horizon + 1)

        self.avoidance_active = False
        self.avoidance_completed = False
        self.switch_seeded = False
        self.obstacle_state = None

        self.control_publisher = self.create_publisher(
            Float64MultiArray, '/rov_mrt/control', 10
        )
        self.prediction_publisher = self.create_publisher(
            Path, '/predicted_path', 10
        )
        self.solve_time_publisher = self.create_publisher(
            Float64, '/nmpc/solve_time', 10
        )
        self.diagnostics_publisher = self.create_publisher(
            Float64MultiArray, '/nmpc/diagnostics', 10
        )

        self.obstacle_subscription = self.create_subscription(
            Float64MultiArray,
            '/rov_mrt/obstacle_states',
            self.obstacle_callback,
            10,
        )

        self.get_logger().info('Building avoidance NMPC solver...')
        self.build_solver()

        self.state_subscription = self.create_subscription(
            Float64MultiArray,
            '/rov_mrt/state',
            self.state_callback,
            10,
        )

        self.get_logger().info(
            'Opposite-side two-obstacle NMPC ready. '
            f'Profile: [{self.profile_s0:.2f}, {self.profile_s1:.2f}, '
            f'{self.profile_s2:.2f}, {self.profile_s3:.2f}, '
            f'{self.profile_s4:.2f}, {self.profile_s5:.2f}] m'
        )

    def obstacle_callback(self, message):
        if len(message.data) != 10:
            self.get_logger().warning(
                'Obstacle state must contain ten values.'
            )
            return
        self.obstacle_state = np.asarray(
            message.data, dtype=float
        ).reshape((2, 5)).T

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
            state + 0.5 * self.ts * k1, control
        )
        k3 = self.vehicle_dynamics(
            state + 0.5 * self.ts * k2, control
        )
        k4 = self.vehicle_dynamics(state + self.ts * k3, control)
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

    @staticmethod
    def cosine_blend(start, finish, ratio):
        blend = 0.5 * (1.0 - ca.cos(ca.pi * ratio))
        return start + (finish - start) * blend

    def avoidance_offset_symbolic(self, progress):
        offset = 0.0
        ratio = (progress - self.profile_s0) / (
            self.profile_s1 - self.profile_s0
        )
        offset = ca.if_else(
            ca.logic_and(
                progress >= self.profile_s0,
                progress < self.profile_s1,
            ),
            self.cosine_blend(0.0, self.offset_amplitude, ratio),
            offset,
        )
        offset = ca.if_else(
            ca.logic_and(
                progress >= self.profile_s1,
                progress <= self.profile_s2,
            ),
            self.offset_amplitude,
            offset,
        )
        ratio = (progress - self.profile_s2) / (
            self.profile_s3 - self.profile_s2
        )
        offset = ca.if_else(
            ca.logic_and(
                progress > self.profile_s2,
                progress < self.profile_s3,
            ),
            self.cosine_blend(
                self.offset_amplitude,
                -self.offset_amplitude,
                ratio,
            ),
            offset,
        )
        offset = ca.if_else(
            ca.logic_and(
                progress >= self.profile_s3,
                progress <= self.profile_s4,
            ),
            -self.offset_amplitude,
            offset,
        )
        ratio = (progress - self.profile_s4) / (
            self.profile_s5 - self.profile_s4
        )
        offset = ca.if_else(
            ca.logic_and(
                progress > self.profile_s4,
                progress <= self.profile_s5,
            ),
            self.cosine_blend(-self.offset_amplitude, 0.0, ratio),
            offset,
        )
        return offset

    def append_safety_constraints(
        self, constraints, state, obstacle_matrix, slack, step
    ):
        constraints.extend([state[3], state[4]])
        direction_m = ca.vertcat(ca.cos(state[2]), ca.sin(state[2]))
        hinge = state[0:2] + self.length * direction_m
        psi_r = state[2] + state[4]
        direction_r = ca.vertcat(ca.cos(psi_r), ca.sin(psi_r))
        rov_center = hinge + self.rov_offset * direction_r

        for obstacle_index in range(2):
            obstacle_position = (
                obstacle_matrix[0:2, obstacle_index]
                + step * self.ts
                * obstacle_matrix[2:4, obstacle_index]
            )
            obstacle_radius = obstacle_matrix[4, obstacle_index]

            for rho in self.rho_b:
                difference = (
                    state[0:2] + float(rho) * direction_m
                    - obstacle_position
                )
                padded_radius = (
                    self.radius_b + obstacle_radius + self.d_safe
                )
                hard_radius = (
                    self.radius_b + obstacle_radius + self.hard_margin
                )
                constraints.extend([
                    ca.dot(difference, difference)
                    - padded_radius ** 2 + slack,
                    ca.dot(difference, difference) - hard_radius ** 2,
                ])

            for rho in self.rho_r:
                difference = (
                    rov_center + float(rho) * direction_r
                    - obstacle_position
                )
                padded_radius = (
                    self.radius_r + obstacle_radius + self.d_safe
                )
                hard_radius = (
                    self.radius_r + obstacle_radius + self.hard_margin
                )
                constraints.extend([
                    ca.dot(difference, difference)
                    - padded_radius ** 2 + slack,
                    ca.dot(difference, difference) - hard_radius ** 2,
                ])

    def build_solver(self):
        reference_x_function = ca.interpolant(
            'avoidance_reference_x', 'linear',
            [self.path_s], self.path_x
        )
        reference_y_function = ca.interpolant(
            'avoidance_reference_y', 'linear',
            [self.path_s], self.path_y
        )
        reference_psi_function = ca.interpolant(
            'avoidance_reference_psi', 'linear',
            [self.path_s], self.path_psi
        )

        number_w = 3 * self.horizon
        decision = ca.SX.sym(
            'avoidance_decision', number_w + self.horizon + 1
        )
        extended_control = ca.reshape(
            decision[0:number_w], 3, self.horizon
        )
        epsilon = decision[number_w:]

        parameter = ca.SX.sym('avoidance_parameter', 19)
        state = parameter[0:5]
        progress = parameter[5]
        previous_control = parameter[6:8]
        obstacle_matrix = ca.reshape(parameter[8:18], 5, 2)
        profile_active = parameter[18]

        objective = 0.0
        constraints = []
        predicted_states = [state]

        for step in range(self.horizon):
            reference_x = reference_x_function(progress)
            reference_y = reference_y_function(progress)
            reference_psi = reference_psi_function(progress)
            difference = state[0:2] - ca.vertcat(
                reference_x, reference_y
            )
            contour_error = (
                -ca.sin(reference_psi) * difference[0]
                + ca.cos(reference_psi) * difference[1]
            )
            contour_target_error = (
                contour_error
                - profile_active
                * self.avoidance_offset_symbolic(progress)
            )
            lag_error = (
                ca.cos(reference_psi) * difference[0]
                + ca.sin(reference_psi) * difference[1]
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
                    control - extended_control[0:2, step - 1]
                )

            objective += (
                32.0 * contour_target_error ** 2
                + 8.0 * lag_error ** 2
                + 12.0 * heading_error ** 2
                + 12.0 * (state[3] - self.reference_speed) ** 2
                + 0.8 * control[0] ** 2
                + 0.5 * control[1] ** 2
                + 1.2 * control_change[0] ** 2
                + 0.8 * control_change[1] ** 2
                - 8.0 * self.ts * path_speed
                + 1.0e5 * epsilon[step] ** 2
            )

            self.append_safety_constraints(
                constraints, state, obstacle_matrix,
                epsilon[step], step
            )
            state = self.rk4_symbolic(state, control)
            progress = ca.fmin(
                self.path_length,
                progress + self.ts * path_speed,
            )
            predicted_states.append(state)

        reference_x = reference_x_function(progress)
        reference_y = reference_y_function(progress)
        reference_psi = reference_psi_function(progress)
        difference = state[0:2] - ca.vertcat(
            reference_x, reference_y
        )
        contour_error = (
            -ca.sin(reference_psi) * difference[0]
            + ca.cos(reference_psi) * difference[1]
        )
        contour_target_error = (
            contour_error
            - profile_active * self.avoidance_offset_symbolic(progress)
        )
        lag_error = (
            ca.cos(reference_psi) * difference[0]
            + ca.sin(reference_psi) * difference[1]
        )
        heading_error = ca.atan2(
            ca.sin(state[2] - reference_psi),
            ca.cos(state[2] - reference_psi),
        )
        objective += (
            120.0 * contour_target_error ** 2
            + 8.0 * lag_error ** 2
            + 55.0 * heading_error ** 2
            + 6.0 * (state[3] - self.reference_speed) ** 2
            + 1.0e5 * epsilon[self.horizon] ** 2
        )
        self.append_safety_constraints(
            constraints, state, obstacle_matrix,
            epsilon[self.horizon], self.horizon
        )

        problem = {
            'x': decision,
            'p': parameter,
            'f': objective,
            'g': ca.vertcat(*constraints),
        }
        options = {
            'ipopt.print_level': 0,
            'ipopt.max_iter': self.solver_max_iterations,
            'ipopt.max_cpu_time': self.solver_max_cpu_time,
            'ipopt.tol': self.solver_tolerance,
            'print_time': False,
            'error_on_fail': False,
        }
        self.solver = ca.nlpsol(
            'two_obstacle_solver', 'ipopt', problem, options
        )
        self.trajectory_function = ca.Function(
            'avoidance_trajectory_function',
            [decision, parameter],
            [ca.hcat(predicted_states)],
        )

        lower_w = np.tile(
            [-self.a_max, -self.omega_delta_max, self.v_min],
            self.horizon,
        )
        upper_w = np.tile(
            [self.a_max, self.omega_delta_max, self.v_max],
            self.horizon,
        )
        self.lower_decision = np.concatenate((
            lower_w, np.zeros(self.horizon + 1)
        ))
        self.upper_decision = np.concatenate((
            upper_w,
            self.epsilon_max * np.ones(self.horizon + 1),
        ))

        lower_constraint = []
        upper_constraint = []
        safety_count_per_step = 20
        for _ in range(self.horizon + 1):
            lower_constraint.extend(
                [self.v_min, -self.delta_max]
                + [0.0] * safety_count_per_step
            )
            upper_constraint.extend(
                [self.v_max, self.delta_max]
                + [np.inf] * safety_count_per_step
            )
        self.lower_constraint = np.asarray(lower_constraint)
        self.upper_constraint = np.asarray(upper_constraint)

    def update_avoidance_state(self, state):
        squared_distance = (
            (self.path_x - state[0]) ** 2
            + (self.path_y - state[1]) ** 2
        )
        actual_progress = float(
            self.path_s[int(np.argmin(squared_distance))]
        )

        if (
            not self.avoidance_active
            and not self.avoidance_completed
            and actual_progress >= self.profile_s0 - 1.0
        ):
            self.avoidance_active = True
            turn_count = min(4, self.horizon)
            self.w_guess[1, :turn_count] += (
                0.18 * self.omega_delta_max
            )
            self.get_logger().info('Avoidance profile activated: +1.')

        if (
            self.avoidance_active
            and not self.switch_seeded
            and actual_progress >= self.profile_s2 - 1.0
        ):
            turn_count = min(4, self.horizon)
            self.w_guess[1, :turn_count] -= (
                0.18 * self.omega_delta_max
            )
            self.switch_seeded = True
            self.get_logger().info('Side-transition seed applied: -1.')

        if (
            self.avoidance_active
            and actual_progress > self.profile_s5 + 1.0
        ):
            self.avoidance_active = False
            self.avoidance_completed = True
            self.get_logger().info('Avoidance profile released.')

    def physical_margins(self, state):
        psi_m = state[2]
        direction_m = np.array([math.cos(psi_m), math.sin(psi_m)])
        hinge = state[0:2] + self.length * direction_m
        psi_r = psi_m + state[4]
        direction_r = np.array([math.cos(psi_r), math.sin(psi_r)])
        rov_center = hinge + self.rov_offset * direction_r
        minimum_b = np.inf
        minimum_r = np.inf

        for obstacle_index in range(2):
            obstacle_position = self.obstacle_state[0:2, obstacle_index]
            obstacle_radius = self.obstacle_state[4, obstacle_index]
            for rho in self.rho_b:
                center = state[0:2] + rho * direction_m
                margin = np.dot(
                    center - obstacle_position,
                    center - obstacle_position,
                ) - (self.radius_b + obstacle_radius) ** 2
                minimum_b = min(minimum_b, margin)
            for rho in self.rho_r:
                center = rov_center + rho * direction_r
                margin = np.dot(
                    center - obstacle_position,
                    center - obstacle_position,
                ) - (self.radius_r + obstacle_radius) ** 2
                minimum_r = min(minimum_r, margin)
        return minimum_b, minimum_r

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

    def publish_emergency_control(self, state):
        """Command braking and steering centering after unavailable NMPC."""
        control = braking_control(
            state[3],
            state[4],
            min(self.emergency_deceleration, self.a_max),
            self.emergency_steering_gain,
            self.omega_delta_max,
        )
        message = Float64MultiArray()
        message.data = control.tolist()
        self.control_publisher.publish(message)

    def state_callback(self, message):
        if len(message.data) != 5:
            return
        state = np.asarray(message.data, dtype=float)
        if self.obstacle_state is None:
            self.publish_emergency_control(state)
            return

        self.update_avoidance_state(state)
        parameter = np.concatenate((
            state,
            [self.s_progress],
            self.u_previous,
            self.obstacle_state.flatten(order='F'),
            [1.0 if self.avoidance_active else 0.0],
        ))
        initial_decision = np.concatenate((
            self.w_guess.flatten(order='F'),
            self.epsilon_guess,
        ))

        start_time = time.perf_counter()
        solution = self.solver(
            x0=initial_decision,
            p=parameter,
            lbx=self.lower_decision,
            ubx=self.upper_decision,
            lbg=self.lower_constraint,
            ubg=self.upper_constraint,
        )
        solve_time = time.perf_counter() - start_time

        # SIGINT can arrive while IPOPT is running. Avoid publishing through
        # an invalid ROS context when the solve returns during shutdown.
        if not rclpy.ok():
            return

        timing_message = Float64()
        timing_message.data = solve_time
        self.solve_time_publisher.publish(timing_message)

        constraint_value = np.asarray(
            solution['g'], dtype=float
        ).ravel()
        lower_violation = np.maximum(
            self.lower_constraint - constraint_value, 0.0
        )
        upper_violation = np.maximum(
            constraint_value - self.upper_constraint, 0.0
        )
        maximum_violation = float(max(
            np.max(lower_violation),
            np.max(upper_violation),
        ))

        solver_success = bool(self.solver.stats()['success'])
        acceptable = (
            solver_success
            or maximum_violation <= self.acceptable_violation
        )
        if not acceptable:
            self.publish_emergency_control(state)
            self.get_logger().error(
                'NMPC rejected: '
                f"{self.solver.stats()['return_status']}, "
                f'violation={maximum_violation:.3e}'
            )
            return

        number_w = 3 * self.horizon
        optimal = np.asarray(solution['x'], dtype=float).ravel()
        optimal_w = optimal[0:number_w].reshape(
            (3, self.horizon), order='F'
        )
        optimal_epsilon = optimal[number_w:]
        applied_control = optimal_w[0:2, 0]

        control_message = Float64MultiArray()
        control_message.data = applied_control.tolist()
        self.control_publisher.publish(control_message)

        prediction = np.asarray(
            self.trajectory_function(optimal, parameter), dtype=float
        )
        self.publish_prediction(prediction)

        minimum_b, minimum_r = self.physical_margins(state)
        diagnostic_message = Float64MultiArray()
        diagnostic_message.data = [
            minimum_b,
            minimum_r,
            float(np.max(optimal_epsilon)),
            solve_time,
            maximum_violation,
            1.0 if self.avoidance_active else 0.0,
        ]
        self.diagnostics_publisher.publish(diagnostic_message)

        self.s_progress = min(
            self.path_length,
            self.s_progress + self.ts * optimal_w[2, 0],
        )
        self.u_previous = applied_control.copy()
        self.w_guess[:, :-1] = optimal_w[:, 1:]
        self.w_guess[:, -1] = optimal_w[:, -1]
        self.epsilon_guess[:-1] = optimal_epsilon[1:]
        self.epsilon_guess[-1] = optimal_epsilon[-1]


def main(args=None):
    rclpy.init(args=args)
    node = NmpcAvoidanceControllerNode()
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
