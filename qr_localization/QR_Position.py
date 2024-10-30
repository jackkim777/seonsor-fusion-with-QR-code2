import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import re
import tf_transformations

class RobotPositionNode(Node):

    def __init__(self):
        super().__init__('robot_position_node')
        
        # 파일에서 Position 값을 읽어오기
        file_path = '/home/jm/jm_ws/src/qr_localization/global/hallway7_5m.txt'
        with open(file_path, 'r') as file:
            self.line_num = len(file.readlines())
        self.x_list = []
        self.y_list = []
        self.orient_x_list = []
        self.orient_y_list = []
        self.orient_z_list = []
        self.orient_w_list = []
        for i in range(self.line_num):
            x, y = self.read_position_from_file(file_path, i)
            self.x_list.append(x)
            self.y_list.append(y)
        
        for i in range(self.line_num):
            x, y, z = self.read_orientation_from_file(file_path, i)
            self.orient_x_list.append(x)
            self.orient_y_list.append(y)
            self.orient_z_list.append(z)
            # self.orient_w_list.append(w)

        # TF 브로드캐스터 초기화
        self.tf_broadcaster = TransformBroadcaster(self)
        print(f"x: {self.x_list}\ny: {self.y_list}\n\n\n")
        # TF 발행 타이머 설정 (1초마다 발행)
        self.timer = self.create_timer(1.0, self.publish_robot_tf)
        
    def read_position_from_file(self, file_path, i):
        with open(file_path, 'r') as file:
            line = file.readlines()
        # print(line[0])
        # print("파일 내용:")
        # print(content)
        
        # 정규 표현식을 사용하여 Position 값 추출
        match = re.search(r'Position:\s*\[(-?\d+\.\d+),\s*(-?\d+\.\d+)\]', line[i])
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            # print(x , y)
            return x, y
        else:
            raise ValueError("Position 값을 찾을 수 없습니다.")
        
    def read_orientation_from_file(self, file_path, i):
        with open(file_path, 'r') as file:
            line = file.readlines()
        match = re.search(r'Orientation:\s*\[(-?\d+\.\d+),\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)\]', line[i])
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            # w = float(match.group(4))
            # print(f"x,y,z,w : {x,y,z,w}")
            return x, y, z
        else:
            raise ValueError("Orientation 값을 찾을 수 없습니다.")

    def publish_robot_tf(self):
        # 현재 시간 가져오기
        now = self.get_clock().now().to_msg()
        
        # TransformStamped 메시지 생성
        t = TransformStamped()
        # print(self.line_num)
        for i in range(self.line_num):
            print(i+1)
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = f'QR_CODE{i+1}'

            # 로봇의 위치 설정
            t.transform.translation.x = self.x_list[i]
            t.transform.translation.y = self.y_list[i]
            t.transform.translation.z = 0.0  # z값은 0으로 설정

            # 회전 각도 (여기서는 기본 값 사용, 필요시 수정)
            quat = tf_transformations.quaternion_from_euler(self.orient_x_list[i], self.orient_y_list[i], self.orient_z_list[i])
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            # t.transform.rotation.x = self.orient_x_list[i]
            # t.transform.rotation.y = self.orient_y_list[i]
            # t.transform.rotation.z = self.orient_z_list[i]
            # t.transform.rotation.w = self.orient_w_list[i]
            print(t)
            # TF 발행
            self.tf_broadcaster.sendTransform(t)
            # print(i)

def main(args=None):
    rclpy.init(args=args)
    robot_position_node = RobotPositionNode()
    rclpy.spin(robot_position_node)
    robot_position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
