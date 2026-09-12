"""Publish two constant-velocity obstacles and their short predictions."""

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleNode(Node):
    """Simulate the default opposite-side two-obstacle scenario."""

    COLORS = ((1.0, 0.1, 0.1), (0.8, 0.1, 0.8))

    def __init__(self):
        super().__init__('obstacle_node')
        self.declare_parameter('sample_time', 0.30)
        self.declare_parameter('prediction_steps', 14)
        self.declare_parameter(
            'obstacles',
            [14.0, -5.0, 0.0, 0.30, 0.55,
             34.0, 6.0, 0.0, -0.12, 0.55],
        )
        self.ts = float(self.get_parameter('sample_time').value)
        self.prediction_steps = int(
            self.get_parameter('prediction_steps').value
        )
        raw = list(self.get_parameter('obstacles').value)
        if len(raw) != 10:
            raise ValueError('The demo requires two [x0,y0,vx,vy,radius] records.')
        self.obstacles = [raw[:5], raw[5:]]

        self.marker_publisher = self.create_publisher(
            MarkerArray, '/dynamic_obstacles', 10
        )
        self.state_publisher = self.create_publisher(
            Float64MultiArray, '/rov_mrt/obstacle_states', 10
        )
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.ts, self.update_obstacles)
        self.get_logger().info('Two dynamic obstacles started.')

    def make_marker(self, marker_id, x, y, radius, color, predicted=False):
        """Build either the obstacle cylinder or predicted-position dots."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacle_predictions' if predicted else 'dynamic_obstacles'
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.85 if predicted else 0.90
        if predicted:
            marker.type = Marker.SPHERE_LIST
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
        else:
            marker.type = Marker.CYLINDER
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 2.0 * radius
            marker.scale.z = 0.30
        return marker

    def update_obstacles(self):
        """Publish current and predicted constant-velocity obstacle states."""
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds * 1.0e-9
        marker_array = MarkerArray()
        state_data = []
        for index, obstacle in enumerate(self.obstacles):
            x0, y0, vx, vy, radius = obstacle
            x = x0 + vx * elapsed
            y = y0 + vy * elapsed
            state_data.extend([x, y, vx, vy, radius])
            color = self.COLORS[index]
            marker_array.markers.append(
                self.make_marker(index, x, y, radius, color)
            )
            prediction = self.make_marker(
                10 + index, x, y, radius, color, predicted=True
            )
            for step in range(self.prediction_steps + 1):
                point = Point()
                point.x = x + vx * step * self.ts
                point.y = y + vy * step * self.ts
                point.z = 0.35
                prediction.points.append(point)
            marker_array.markers.append(prediction)

        state_message = Float64MultiArray()
        state_message.data = state_data
        self.marker_publisher.publish(marker_array)
        self.state_publisher.publish(state_message)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
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
