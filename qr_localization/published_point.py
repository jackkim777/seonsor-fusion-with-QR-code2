#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class ClickedPointListener(Node):

    def __init__(self):
        super().__init__('clicked_point_listener')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        self.subscription  # prevent unused variable warning

    def clicked_point_callback(self, msg):
        point = msg.point
        self.get_logger().info(f'Clicked Point: x: {point.x}, y: {point.y}, z: {point.z}')

def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
