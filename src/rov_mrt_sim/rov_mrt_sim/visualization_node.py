import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from rov_mrt_sim.model import collision_centers


class VisualizationNode(Node):
    """Visualize articulated links and the collision-circle approximation."""

    def __init__(self):
        super().__init__('visualization_node')

        self.declare_parameter('vehicle_length', 3.0)
        self.declare_parameter('rov_offset', 1.2)
        self.declare_parameter('mrt_circle_offsets', [-0.90, 0.55, 2.00])
        self.declare_parameter('mrt_circle_radius', 0.95)
        self.declare_parameter('rov_circle_offsets', [-0.45, 0.45])
        self.declare_parameter('rov_circle_radius', 0.80)
        self.length = float(self.get_parameter('vehicle_length').value)
        self.rov_offset = float(self.get_parameter('rov_offset').value)
        self.rho_b = list(self.get_parameter('mrt_circle_offsets').value)
        self.radius_b = float(self.get_parameter('mrt_circle_radius').value)
        self.rho_r = list(self.get_parameter('rov_circle_offsets').value)
        self.radius_r = float(self.get_parameter('rov_circle_radius').value)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/rov_mrt/state',
            self.state_callback,
            10,
        )

        self.publisher = self.create_publisher(
            MarkerArray,
            '/rov_mrt/markers',
            10,
        )

        self.get_logger().info(
            'ROV-MRT visualization node started.'
        )

    def make_point(self, x, y, z=0.0):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def make_link(self, marker_id, points, color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'vehicle_links'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.22
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.points = points

        return marker

    def make_circle(
        self,
        marker_id,
        x,
        y,
        radius,
        color,
    ):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'collision_circles'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.03
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = 0.06

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.25

        return marker

    def state_callback(self, message):
        if len(message.data) != 5:
            return

        x_m, y_m, psi_m, _, delta_r = message.data

        # Link endpoints.
        x_h = x_m + self.length * math.cos(psi_m)
        y_h = y_m + self.length * math.sin(psi_m)

        psi_r = psi_m + delta_r

        x_r = x_h + self.rov_offset * math.cos(psi_r)
        y_r = y_h + self.rov_offset * math.sin(psi_r)

        markers = MarkerArray()
        mrt_centers, rov_centers = collision_centers(
            message.data,
            self.length,
            self.rov_offset,
            self.rho_b,
            self.rho_r,
        )

        # MRT至铰接点
        markers.markers.append(
            self.make_link(
                0,
                [
                    self.make_point(x_m, y_m, 0.12),
                    self.make_point(x_h, y_h, 0.12),
                ],
                (0.1, 0.1, 0.1),
            )
        )

        # 铰接点至ROV
        markers.markers.append(
            self.make_link(
                1,
                [
                    self.make_point(x_h, y_h, 0.14),
                    self.make_point(x_r, y_r, 0.14),
                ],
                (0.1, 0.3, 1.0),
            )
        )

        for index, center in enumerate(mrt_centers):
            markers.markers.append(
                self.make_circle(
                    10 + index,
                    center[0],
                    center[1],
                    self.radius_b,
                    (0.1, 0.1, 0.1),
                )
            )

        for index, center in enumerate(rov_centers):
            markers.markers.append(
                self.make_circle(
                    20 + index,
                    center[0],
                    center[1],
                    self.radius_r,
                    (0.1, 0.3, 1.0),
                )
            )

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()

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
