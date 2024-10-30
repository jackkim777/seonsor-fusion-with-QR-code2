import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class OdomTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('odom_trajectory_publisher')

        # MarkerArray 퍼블리셔
        self.publisher = self.create_publisher(MarkerArray, '/odom_trajectory_marker', 10)

        # odom 토픽 구독
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # Odometry 데이터를 퍼블리시하는 토픽
            self.odom_callback,
            10
        )

        # Trajectory를 저장할 리스트
        self.trajectory_points = []

        # 주기적으로 궤적을 퍼블리시
        self.timer = self.create_timer(0.5, self.publish_trajectory)

    def odom_callback(self, msg: Odometry):
        # Odometry에서 로봇의 현재 위치를 추출하고 궤적에 추가
        current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.trajectory_points.append(current_point)

    def publish_trajectory(self):
        marker_array = MarkerArray()

        # MarkerArray에 들어갈 Marker 설정
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = 'odom'  # odom 좌표계에서 표시
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.ns = 'odom_trajectory'
        line_strip_marker.id = 0
        line_strip_marker.type = Marker.LINE_STRIP  # 선을 그리기 위한 타입
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.05  # 선의 두께
        line_strip_marker.color.a = 0.5  # 투명도
        line_strip_marker.color.r = 0.0  # 빨간색 (R)
        line_strip_marker.color.g = 0.0  # 초록색 (G)
        line_strip_marker.color.b = 1.0  # 파란색 (B)

        # 저장된 Trajectory 좌표를 Marker에 추가
        for point in self.trajectory_points:
            p = Point()
            p.x, p.y, p.z = point
            line_strip_marker.points.append(p)

        marker_array.markers.append(line_strip_marker)

        # 퍼블리시
        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()