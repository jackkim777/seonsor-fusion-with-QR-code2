import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as ROSPoint
import re


class Point:
    def __init__(self, index, x, y, z):
        self.index = index
        self.x = x
        self.y = y
        self.z = z


class DisplayPoints(Node):
    def __init__(self):
        super().__init__('display_points')
        self.publisher_ = self.create_publisher(MarkerArray, 'marker/node', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.vec_point = self.create_points()

    def create_points(self):
        file_path = '/home/jm/jm_ws/src/qr_localization/global/hallway7_3m.txt'
        index_pattern = re.compile(r'Index:\s*(\d+)')
        position_pattern = re.compile(r'Position:\s*\[([^\]]+)\]')
        vec_point = []

        with open(file_path, 'r') as f:
            lines = f.readlines()

        for line in lines:
            index_match = index_pattern.search(line)
            position_match = position_pattern.search(line)

            if index_match and position_match:
                index = int(index_match.group(1))
                position_str = position_match.group(1)
                position = [float(value.strip()) for value in position_str.split(',')]
                p = Point(index, position[0], position[1], 0.0)
                vec_point.append(p)

        return vec_point

    def timer_callback(self):
        marker_array = MarkerArray()

        for o_node in self.vec_point:
            points_marker = Marker()
            points_marker.header.frame_id = "map"
            points_marker.header.stamp = self.get_clock().now().to_msg()
            points_marker.type = Marker.POINTS
            points_marker.id = o_node.index 
            points_marker.action = Marker.ADD
            points_marker.pose.orientation.w = 1.0
            points_marker.scale.x = 0.2  
            points_marker.scale.y = 0.2  
            points_marker.color.r = 0.0
            points_marker.color.g = 1.0
            points_marker.color.b = 0.0
            points_marker.color.a = 1.0  

            point = ROSPoint()
            point.x = o_node.x
            point.y = o_node.y
            point.z = o_node.z
            points_marker.points.append(point)

            marker_array.markers.append(points_marker)

        self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    display_points = DisplayPoints()
    rclpy.spin(display_points)
    display_points.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

