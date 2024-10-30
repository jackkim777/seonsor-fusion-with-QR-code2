import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import pandas as pd

class LineVisualizer(Node):
    def __init__(self):
        super().__init__('line_visualizer')
        self.publisher = self.create_publisher(Marker, 'line_marker', 10)
        
        # CSV 파일에서 직선 좌표를 읽어옴
        self.line_coordinates = self.read_coordinates_from_csv('groundtruth.csv')
        
        # 주기적으로 라인 마커를 RViz에 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_line_marker)

    def read_coordinates_from_csv(self, file_name):
        """
        CSV 파일에서 직선 좌표를 읽어옵니다.
        :param file_name: CSV 파일 이름
        :return: [(x, y, z), ...] 형식의 좌표 리스트
        """
        df = pd.read_csv(file_name)
        coordinates = [(row['x'], row['y'], row['z']) for index, row in df.iterrows()]
        return coordinates

    def publish_line_marker(self):
        """
        직선 마커를 RViz에 퍼블리시합니다.
        """
        marker = Marker()
        marker.header.frame_id = "map"  # RViz에서 사용하고자 하는 좌표계로 변경
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # 선의 두께 설정

        # 색상 설정 (빨간색, 불투명)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # 좌표를 Point 메시지로 변환하여 마커에 추가
        for (x, y, z) in self.line_coordinates:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            marker.points.append(point)
        
        # 마커 퍼블리시
        self.publisher.publish(marker)
        self.get_logger().info('Published line marker')

def main(args=None):
    rclpy.init(args=args)
    line_visualizer = LineVisualizer()
    rclpy.spin(line_visualizer)
    line_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()