import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PathTrackingControllerNode(Node):

    def __init__(self):
        super().__init__(
            'path_tracking_controller_node'
        )

        self.length = 3.0
        self.reference_speed = 0.80

        self.a_max = 0.60
        self.delta_max = math.radians(42.0)
        self.omega_delta_max = math.radians(24.0)

        self.k_speed = 0.8
        self.k_contour = 0.35
        self.k_heading = 1.2
        self.k_delta = 2.0

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/rov_mrt/control',
            10,
        )

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/rov_mrt/state',
            self.state_callback,
            10,
        )

        self.get_logger().info(
            'S-curve tracking controller started.'
        )

    @staticmethod
    def wrap_to_pi(angle):
        return (angle + math.pi) % (
            2.0 * math.pi
        ) - math.pi

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def state_callback(self, message):
        if len(message.data) != 5:
            return

        x_m, y_m, psi_m, v_m, delta_r = (
            message.data
        )

        wave_number = 2.0 * math.pi / 24.0
        phase = wave_number * x_m

        y_reference = 1.2 * math.sin(phase)

        dy_dx = (
            1.2
            * wave_number
            * math.cos(phase)
        )

        d2y_dx2 = (
            -1.2
            * wave_number
            * wave_number
            * math.sin(phase)
        )

        psi_reference = math.atan2(dy_dx, 1.0)

        curvature = d2y_dx2 / (
            1.0 + dy_dx * dy_dx
        ) ** 1.5

        contour_error = (
            math.cos(psi_reference)
            * (y_m - y_reference)
        )

        heading_error = self.wrap_to_pi(
            psi_reference - psi_m
        )

        desired_yaw_rate = (
            v_m * curvature
            - self.k_contour * contour_error
            + self.k_heading * heading_error
        )

        effective_speed = max(v_m, 0.20)

        desired_delta = math.atan(
            self.length
            * desired_yaw_rate
            / effective_speed
        )

        desired_delta = self.clamp(
            desired_delta,
            -self.delta_max,
            self.delta_max,
        )

        acceleration = self.k_speed * (
            self.reference_speed - v_m
        )

        acceleration = self.clamp(
            acceleration,
            -self.a_max,
            self.a_max,
        )

        steering_rate = self.k_delta * (
            desired_delta - delta_r
        )

        steering_rate = self.clamp(
            steering_rate,
            -self.omega_delta_max,
            self.omega_delta_max,
        )

        control_message = Float64MultiArray()
        control_message.data = [
            acceleration,
            steering_rate,
        ]

        self.publisher.publish(control_message)


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackingControllerNode()

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
