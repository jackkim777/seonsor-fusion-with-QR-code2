import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TransformStamped, Twist, Vector3, PoseStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from std_msgs.msg import Float32
import math
import tf_transformations
import re
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
# from direction import Direction


class RobotPositionNode(Node):

    def __init__(self):
        super().__init__('robot_position_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/estimated_Z',
            self.qr_code_callback,
            10)
        self.start_time = self.get_clock().now().seconds_nanoseconds()
        self.timerCallback = self.create_timer(0.1, self.process)
        self.subscription = self.create_subscription(Float32, '/current_angle', self.angleCallback, 10)
        self.subscription  # prevent unused variable warning
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.cmdVelSub = self.create_subscription(Twist, '/cmd_vel', self.cmdVelCallback, 10)
        self.qr_publiseher = self.create_subscription(LaserScan, '/qr_index_number', self.index_callback, qos)
        # self.path_publisher_ = self.create_publisher(Path, 'robot_path', 10)
        # self.qr_remapping = self.create_subscription(LaserScan, '/scan', self.QRRemapping, qos)
        self.qr_pub = self.create_publisher(LaserScan, '/scan_re', qos)
        self.scanCallback = self.create_subscription(LaserScan, '/scan', self.ScanCallback, qos)
        # TF 브로드캐스터 초기화
        self.tf_broadcaster = TransformBroadcaster(self)
        self.distance = 0.0
        self.angle = 0.0
        self.Robot_Pose_buffer = [0.0, 0.0]
        self.isQR = False
        self.addPose = [0.0, 0.0]
        self.scan = None
        # self.Direction = Direction()
        self.last_time = self.get_clock().now()
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.msg_accel_x = 0.0
        self.msg_accel_y = 0.0
        self.received_accel_x = 0.0
        self.received_accel_y = 0.0
        # 속도 초기화
        self.linear_x = 0.0
        self.velocity = Vector3()
        self.velocity.x = 0.0
        self.velocity.y = 0.0
        self.velocity.z = 0.0
        
        # 거리 초기화
        self.distance_change = Vector3()
        self.distance_change.x = 0.0
        self.distance_change.y = 0.0
        self.distance_change.z = 0.0

        self.change_angle = None
        self.yaw = 0.0
        self.initial = True

        self.quat_buffer = [0.0, 0.0, 0.0, 0.0]
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.QR = None
        self.dt = 0.0
        self.initial_error_angle = 0.0
        self.initial_accel_x_error = 0.0
        self.initial_accel_y_error = 0.0

        self.index = 4

        self.path = Path()
        self.path.header.frame_id = "map"
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.yaw_buffer = 0.0
        self.yaw_rad_msg = 0.0
        self.elapsed_time = 0.0
        self.Stamp = False
    def cmdVelCallback(self, msg):
        self.linear_x = msg.linear.x

    def ScanCallback(self, msg):
        # print("scan subscribing")
        self.scan = msg
        
    # def path_callback(self):
    #     try:
    #         now = rclpy.time.Time()
    #         trans = self.tf_buffer.lookup_transform('map', 'Global_Robot', now)

    #         pose = PoseStamped()
    #         pose.header.stamp = self.get_clock().now().to_msg()
    #         pose.header.frame_id = 'map'
    #         pose.pose.position.x = trans.transform.translation.x
    #         pose.pose.position.y = trans.transform.translation.y
    #         pose.pose.position.z = trans.transform.translation.z
    #         pose.pose.orientation = trans.transform.rotation

    #         self.path.poses.append(pose)
    #         self.path_publisher_.publish(self.path)

    #     except Exception as e:
    #         self.get_logger().error(f'Failed to get transform: {e}')

    def index_callback(self, msg):
        self.index = int(msg.ranges[0])

    def qr_code_callback(self, msg):
        self.distance = msg.ranges[0]

    def remove_noise(self, value, threshold=0.02):
        return value if abs(value) > threshold else 0.0
    
    def imu_callback(self, msg):
        self.velocity.x = self.linear_x

        self.distance_change.x = self.velocity.x * self.dt
        self.distance_change.y = self.velocity.y * self.dt
        self.change_angle = msg.orientation
        print(f"Orientation : {self.change_angle}")
        # self.change_angle = lpf_x(msg.orientation)
        roll, pitch, self.yaw_rad_msg = tf_transformations.euler_from_quaternion([
            self.change_angle.x * 1.2,
            self.change_angle.y*1.2, 
            self.change_angle.z*1.2,
            self.change_angle.w*1.2
            # 0.0,0.0
        ])   

        self.addPose[0] = self.distance_change.x
        self.addPose[1] = self.distance_change.y
        
    def read_position_from_file(self, file_path, index):
        with open(file_path, 'r') as file:
            lines = file.readlines()

        for line in lines:
            if line.startswith(f"Index: {index},"):
                match = re.search(r'Position:\s*\[(-?\d+\.\d+),\s*(-?\d+\.\d+)\]', line)
                
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    return x, y
                else:
                    raise ValueError("Position 값을 찾을 수 없습니다.")

    def read_orientation_from_file(self, file_path, index):
        with open(file_path, 'r') as file:
            lines = file.readlines()
        for line in lines:
            if line.startswith(f"Index: {index},"):
                match = re.search(r'Orientation:\s*\[(-?\d+\.\d+),\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)\]', line)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    z = float(match.group(3))
                    return x, y, z
                else:
                    raise ValueError("Position 값을 찾을 수 없습니다.")
                
    def angleCallback(self, msg):
        self.angle = msg.data
        self.isQR = True

    def process(self):
        current_time2 = self.get_clock().now().seconds_nanoseconds()
        self.elapsed_time = current_time2[0] - self.start_time[0] + (current_time2[1] - self.start_time[1]) * 1e-9
        self.current_time = self.get_clock().now()
        self.dt = (self.current_time - self.last_time).nanoseconds / 1e9  # 시간 간격 계산 (초 단위)
        self.last_time = self.current_time
        qr_roll, qr_pitch, qr_yaw = self.read_orientation_from_file("/home/jm/jm_ws/src/qr_localization/global/hallway7_5m.txt", self.index)
        qr_x, qr_y = self.read_position_from_file("/home/jm/jm_ws/src/qr_localization/global/hallway7_5m.txt", self.index)
        # print(f"QR_to_Robot pitch : {qr_pitch}")
        # print("Processing!")
        if self.isQR: # QR코드 보일 때
            print("QR Code Detected")
            self.Robot_Pose_buffer = [0.0, 0.0]
            self.velocity.x = 0.0
            self.velocity.y = 0.0
            self.velocity.z = 0.0
            # print("QR Code is detected!")
            # QR코드 기반 로봇 Pose 계산
            robot_x, robot_y = self.calculate_robot_position(qr_x, qr_y, self.distance, self.angle)
            print(math.degrees(qr_yaw))
            # Map이 틀어져있기 떄문에 Rotation Matrix 적용
            self.robot_x, self.robot_y = self.Rotation_matrix(robot_x, robot_y, qr_x, qr_y, qr_yaw)
            # QR코드 안 보일 때 위치 그대로 유지 적용하기 위한 버퍼
            self.Robot_Pose_buffer = [self.robot_x, self.robot_y]

            self.publish_robot_tf(self.robot_x, self.robot_y, 0.0, qr_yaw)
            self.isQR = False
        else: # QR코드 안보일 때 
            print("QR Code is not detected!")
            self.qr_yaw = qr_yaw

            yaw = self.yaw + 3.141592 # 3.141592는 매번 달라짐 (초기 맵 각도에 따라) 로봇 이동 방향 결정
            
            # 맵 틀어져있기 떄문에 Rotation Matrix 적용
            self.addPose[0], self.addPose[1] = self.Rotation_matrix(self.addPose[0], self.addPose[1], 0.0, 0.0, yaw)
            # QR코드 보였을 때 저장했던 위치에 이어서 위치 업데이트        
            self.Robot_Pose_buffer[0] = self.Robot_Pose_buffer[0] - self.addPose[0]
            self.Robot_Pose_buffer[1] = self.Robot_Pose_buffer[1] - self.addPose[1]

            # 위치 업데이트
            self.robot_x = self.Robot_Pose_buffer[0] 
            self.robot_y = self.Robot_Pose_buffer[1] 
            # robot_x, robot_y = self.Rotation_matrix(robot_x, robot_y, 0.0, 0.0, self.qr_yaw - self.yaw)

            # Cmdvel에서 받아올 위치 버퍼 초기화
            self.addPose = [0.0, 0.0]

            self.publish_robot_tf(self.robot_x, self.robot_y, qr_pitch, self.yaw)
            # print(f"self.yaw : {self.yaw}")
        
        # trajectory 생성
        # self.path_callback()  

        # print(f"linear accelaton : {self.received_accel_x, self.received_accel_y}")
        # print(f"Initial Accel Error : {self.initial_accel_x_error, self.initial_accel_y_error}")
        # print(f"current Accel : {self.accel_x, self.accel_y}")
        # print(f"current Position : {self.robot_x, self.robot_y}")
        # print("\n\n")
   

    def calculate_robot_position(self, qr_x, qr_y, distance, angle_deg):
        angle_rad = math.radians(angle_deg)
        # robot_x = qr_x - distance * math.cos(angle_rad)
        if angle_rad == 90.0:
            robot_x = qr_x
            robot_y = qr_y - distance
        elif angle_rad > 90.0:
            robot_x = qr_x - (distance * math.cos(180.0-angle_rad))
            robot_y = qr_y - (distance * math.sin(180.0-angle_rad))
        else:
            robot_x = qr_x + (distance * math.cos(angle_rad))
            robot_y = qr_y - (distance * math.sin(angle_rad))
        # print(f"Global Robot Pose : {robot_x} , {robot_y}")
        self.Robot_Pose_buffer = robot_x , robot_y
        return robot_x, robot_y

    def Rotation_matrix(self, robot_x, robot_y, trans_x, trans_y, angle_rad):
        R = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad)],
                [np.sin(angle_rad), np.cos(angle_rad)] 
            ])
        translated_x = robot_x - trans_x   
        translated_y = robot_y - trans_y

        ratate_position = np.array([translated_x, translated_y])

        rotated_position = np.dot(R, ratate_position)

        rotated_robot_x = rotated_position[0] + trans_x
        rotated_robot_y = rotated_position[1] + trans_y
        return rotated_robot_x, rotated_robot_y

    def publish_robot_tf(self, x, y, qr_pitch, yaw_rad):
        # print(f"initial : {self.initial}")
        # 현재 시간 가져오기
        now = self.get_clock().now().to_msg()
        
        # TransformStamped 메시지 생성
        t = TransformStamped()

        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'Global_Robot'
        
        # 로봇의 위치 설정
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
    
        # 각도 설정
        if self.isQR: # QR코드 보이면
            if not self.Stamp:
                print(f"TimeStamp : {t.header.stamp}")
                with open("/home/jm/TimeStamp", 'a') as f:
                    f.write(f'Index : {self.index}   Timestamp: %s\n' % self.elapsed_time)

            self.Stamp = True
            self.yaw_buffer = self.yaw_rad_msg
            #self.qr_yaw : Global QR TF의 각도
            self.qr_yaw = yaw_rad
            # print(f"yaw_rad : {yaw_rad}")
            quat = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.quat_buffer = [quat[0], quat[1], quat[2], quat[3]]
            self.yaw = yaw_rad
            # print(f"pitch : {qr_pitch}")

        else: # QR코드 안보이면
            self.Stamp = False
            refine_angle = 0.0
            if self.change_angle is not None:
                
                # Global QR TF각도에서 IMU orientation 누적 적용
                yaw_rad = self.qr_yaw - self.yaw_buffer + self.yaw_rad_msg

                # print(f"self.yaw_buffer , self.yaw_rad_msg : {self.yaw_buffer, self.yaw_rad_msg}")
                quat = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
                t.transform.rotation.x = self.quat_buffer[0] + quat[0]
                t.transform.rotation.y = self.quat_buffer[1] + quat[1]
                t.transform.rotation.z = self.quat_buffer[2] + quat[2]
                t.transform.rotation.w = self.quat_buffer[3] + quat[3]
                print(f"self.qr_yaw : {self.qr_yaw}")
                print(f"self.yaw_buffer : {self.yaw_buffer}")
                print(f"self.yaw_rad_msg : {self.yaw_rad_msg}")
                print(f"yaw_rad : {yaw_rad}")
                # print(f"quat buffer : {self.quat_buffer}")
                self.yaw = yaw_rad
                # print(f"imu 각도 : {math.degrees(self.yaw)}")

                if self.initial:
                    self.initial_error_angle = self.yaw # 처음 구동시켰을 때 IMU 오차 값
                    # self.initial_x_vel = self.
                    # print(f"self.initial_error_angle : {self.initial_error_angle}")
                    t.transform.rotation.x = 0.0
                    t.transform.rotation.y = 0.0
                    t.transform.rotation.z = 0.0

                self.initial = False
        # TF 발행
        self.tf_broadcaster.sendTransform(t)
        
def main(args=None):
    rclpy.init(args=args)
    robot_position_node = RobotPositionNode()
    rclpy.spin(robot_position_node)
    robot_position_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
