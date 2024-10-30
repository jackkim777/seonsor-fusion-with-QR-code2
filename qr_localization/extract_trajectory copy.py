import rclpy
from rclpy.node import Node
import pandas as pd
import threading
from tf2_ros import Buffer, TransformListener, LookupException

class TrajectorySaver(Node):
    def __init__(self):
        super().__init__('trajectory_saver')
        
        # tf2 Buffer와 TransformListener 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 궤적 데이터를 저장할 리스트 초기화
        self.trajectory_data = []
        self.exit_flag = False

        # 변환할 소스 및 타겟 프레임 설정
        self.source_frame = 'Global_Robot'  # 로봇의 기준 프레임
        self.target_frame = 'map'        # 기준이 될 프레임 (필요에 따라 변경)

        # tf 데이터를 주기적으로 가져오는 타이머 설정 (0.5초마다)
        self.timer = self.create_timer(0.5, self.get_transform)

    def get_transform(self):
        try:
            # 소스 프레임에서 타겟 프레임으로의 변환 가져오기
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())

            # 변환에서 위치 정보 추출
            position = transform.transform.translation
            self.trajectory_data.append([position.x, position.y, position.z])

        except LookupException:
            self.get_logger().info('Transform not available yet')

    def save_to_csv(self, file_name='trajectory.csv'):
        # 궤적 데이터를 CSV로 저장
        df = pd.DataFrame(self.trajectory_data, columns=['x', 'y', 'z'])
        df.to_csv(file_name, index=False)
        self.get_logger().info(f'Saved trajectory to {file_name}')

    def handle_user_input(self):
        # 사용자 입력을 처리하는 메소드
        while not self.exit_flag:
            user_input = input("Enter 'q' to save and exit: ")
            if user_input.lower() == 'q':
                self.exit_flag = True
                self.save_to_csv('trajectory.csv')
                rclpy.shutdown()  # ROS2 노드 종료

def main(args=None):
    rclpy.init(args=args)
    trajectory_saver = TrajectorySaver()
    
    # 사용자 입력을 처리할 쓰레드 생성
    user_input_thread = threading.Thread(target=trajectory_saver.handle_user_input)
    user_input_thread.start()

    # ROS2 노드를 실행 (사용자 입력을 대기하면서 동작)
    rclpy.spin(trajectory_saver)
    
    # 사용자 입력 쓰레드가 종료될 때까지 대기
    user_input_thread.join()
    trajectory_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()