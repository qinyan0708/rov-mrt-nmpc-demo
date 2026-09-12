"""RK4 simulation node for the five-state articulated vehicle model."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from rov_mrt_sim.model import braking_control, rk4_step


class VehicleSimNode(Node):
    """Simulate the planar ROV-MRT kinematic model."""

    def __init__(self):
        super().__init__('vehicle_sim_node')
        defaults = {
            'sample_time': 0.30,
            'vehicle_length': 3.00,
            'path_amplitude': 1.20,
            'path_wavelength': 24.0,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_speed': 0.50,
            'initial_steering_deg': 0.0,
            'speed_min': 0.0,
            'speed_max': 1.40,
            'acceleration_max': 0.60,
            'steering_max_deg': 42.0,
            'steering_rate_max_deg_s': 24.0,
            'command_timeout': 0.75,
            'timeout_deceleration': 0.60,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        def value(name):
            return self.get_parameter(name).value

        self.ts = float(value('sample_time'))
        self.length = float(value('vehicle_length'))
        self.v_min = float(value('speed_min'))
        self.v_max = float(value('speed_max'))
        self.a_max = float(value('acceleration_max'))
        self.delta_max = math.radians(float(value('steering_max_deg')))
        self.omega_delta_max = math.radians(
            float(value('steering_rate_max_deg_s'))
        )
        self.control_timeout = float(value('command_timeout'))
        self.timeout_deceleration = float(value('timeout_deceleration'))

        amplitude = float(value('path_amplitude'))
        wavelength = float(value('path_wavelength'))
        initial_heading = math.atan(amplitude * 2.0 * math.pi / wavelength)
        self.state = np.asarray([
            float(value('initial_x')),
            float(value('initial_y')),
            initial_heading,
            float(value('initial_speed')),
            math.radians(float(value('initial_steering_deg'))),
        ])
        self.control = np.zeros(2)
        self.last_control_time = self.get_clock().now()

        self.state_publisher = self.create_publisher(
            Float64MultiArray, '/rov_mrt/state', 10
        )
        self.control_subscription = self.create_subscription(
            Float64MultiArray, '/rov_mrt/control', self.control_callback, 10
        )
        self.timer = self.create_timer(self.ts, self.simulation_step)
        self.get_logger().info(
            f'ROV-MRT RK4 vehicle model started at {1.0 / self.ts:.2f} Hz.'
        )

    def control_callback(self, message):
        """Accept and saturate acceleration and steering-rate commands."""
        if len(message.data) != 2:
            self.get_logger().warning(
                'Control command must contain exactly two values.'
            )
            return
        self.control = np.asarray([
            np.clip(float(message.data[0]), -self.a_max, self.a_max),
            np.clip(
                float(message.data[1]),
                -self.omega_delta_max,
                self.omega_delta_max,
            ),
        ])
        self.last_control_time = self.get_clock().now()

    def timeout_control(self):
        """Brake and center steering after command loss."""
        return braking_control(
            self.state[3],
            self.state[4],
            min(self.timeout_deceleration, self.a_max),
            2.0,
            self.omega_delta_max,
        )

    def simulation_step(self):
        """Advance and publish one simulation sample."""
        control_age = (
            self.get_clock().now() - self.last_control_time
        ).nanoseconds * 1.0e-9
        control = (
            self.timeout_control()
            if control_age > self.control_timeout
            else self.control
        )
        self.state = rk4_step(self.state, control, self.ts, self.length)
        self.state[3] = np.clip(self.state[3], self.v_min, self.v_max)
        self.state[4] = np.clip(
            self.state[4], -self.delta_max, self.delta_max
        )
        message = Float64MultiArray()
        message.data = self.state.tolist()
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimNode()
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
