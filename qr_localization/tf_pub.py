import cv2 as cv
import numpy as np
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations

class QRSubscriber(Node):
    def __init__(self):
        super().__init__('qr_subscriber')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tvec = None  # tvec 데이터를 저장하기 위한 변수
        self.angle_subscribe = self.create_subscription(Float32, '/current_angle', self.get_angle,  10)
        self.angle = 0.0

    def get_angle(self, msg):
        self.angle = msg.data
        print("callback angle: ", self.angle)

    def quaternion_from_euler(self, ai, aj, ak):  
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q


    def publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        with open("/home/jm/jm_ws/src/qr_localization/Pose/qr_position_fin.txt", 'r') as file:
            for line in file:
                data = list(map(str, line.split()))
                # print("data:", line)
                index_value = data[1].split(".")[0]  # 인덱스 값을 가져오기 위해 수정
                # print(index_value)
                if 'Position:' in line:
                    pos_index = data.index('Position:')
                    t.child_frame_id = f'Global QR'  # 고유한 child_frame_id 설정

                    position_str = line.split("Position:")[1].strip()  
                    position_str = position_str.split(", quat :")[0].strip()  
                    # print(position_str)
                    position_str = position_str.strip('[]')  

                    # print("position_str: ", position_str)

                    position_list = position_str.split(", ")  
                    # position_list = position_str.split(" quat: ")  

                    position = [float(coord.strip('[],')) for coord in position_list]  # "]" 문자 제거 후 실수형으로 변환
                    
                    # print("Position 값:", position)
                    # print("X 좌표:", position[0])
                    # print("Y 좌표:", position[1])

                    # TransformStamped 메시지에 translation 값 설정
                    t.transform.translation.x = position[0]
                    t.transform.translation.y = position[1]
                    # t.transform.translation.z = position[2]  # 필요에 따라 Z 값을 설정
                    quat_value = line.split(" quat :")[1].strip()  
                    # print("quat_value: ", quat_value)
                    quat_list = quat_value.strip('[]').split(" ")  # quat 값을 리스트로 변환
                    # print("quat_list: ", quat_list)
                    quat = [float(val) for val in quat_list]  # 문자열을 실수형으로 변환하여 리스트로 저장
                    # yaw_radians = np.radians(angle)
                    # print("angle: ", angle)
                    # quat = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)

                    # print("quat 값:", quat)

                    # TransformStamped 메시지에 rotation 값 설정
                    t.transform.rotation.x = quat[0]
                    t.transform.rotation.y = quat[1]
                    t.transform.rotation.z = quat[2]
                    t.transform.rotation.w = quat[3]
                    # print("transform_rotation : ", t.transform.rotation)
                    self.tf_broadcaster.sendTransform(t)

    def publish_tf_local(self, stamp, position, degree):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map' 
        t.child_frame_id = 'local_qr'
        # TransformStamped 메시지에 translation 값 설정
        print("local_position :", position)
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        # t.transform.translation.z = position[2]  # 필요에 따라 Z 값을 설정
        yaw_radians = np.radians(degree)
        print("yaw_radians:", yaw_radians)
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)
        print("quat : ", quat)
        # TransformStamped 메시지에 rotation 값 설정
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        print("Local transform_rotation : ", t.transform.rotation)
        self.tf_broadcaster.sendTransform(t)




def main(args=None):
    rclpy.init(args=args)
    qr_subscriber = QRSubscriber()
    qr_subscriber.publish_tf(qr_subscriber.get_clock().now().to_msg())
    rclpy.spin(qr_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
