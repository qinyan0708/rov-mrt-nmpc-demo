import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class PathPublisherNode(Node):
    """Publish the configured S-curve as a transient-local ROS Path."""

    def __init__(self):
        super().__init__('path_publisher_node')

        self.declare_parameter('path_amplitude', 1.2)
        self.declare_parameter('path_wavelength', 24.0)
        self.declare_parameter('path_x_end', 48.0)
        self.declare_parameter('path_points', 961)
        self.amplitude = float(self.get_parameter('path_amplitude').value)
        self.wavelength = float(self.get_parameter('path_wavelength').value)
        self.path_x_end = float(self.get_parameter('path_x_end').value)
        self.number_of_points = int(self.get_parameter('path_points').value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            Path,
            '/reference_path',
            qos,
        )

        # 稍等节点初始化完成后发布一次
        self.timer = self.create_timer(
            0.5,
            self.publish_path,
        )

        self.path_published = False

        self.get_logger().info(
            'Long S-curve path publisher started.'
        )

    def publish_path(self):
        """Publish one path message; transient-local durability retains it."""
        path_message = Path()
        path_message.header.frame_id = 'map'
        path_message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for index in range(self.number_of_points):
            x = (
                self.path_x_end
                * index
                / (self.number_of_points - 1)
            )

            phase = 2.0 * math.pi * x / self.wavelength
            y = self.amplitude * math.sin(phase)

            # dy/dx，用于计算道路切线航向角
            dy_dx = (
                self.amplitude
                * 2.0
                * math.pi
                / self.wavelength
                * math.cos(phase)
            )

            heading = math.atan2(dy_dx, 1.0)

            pose = PoseStamped()
            pose.header = path_message.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.01

            pose.pose.orientation.z = math.sin(
                heading / 2.0
            )
            pose.pose.orientation.w = math.cos(
                heading / 2.0
            )

            path_message.poses.append(pose)

        self.publisher.publish(path_message)

        if not self.path_published:
            self.path_published = True
            self.get_logger().info(
                f'Published {self.number_of_points}-point S-curve reference path.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()

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
