import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from std_msgs.msg import Float32MultiArray, Float32
import cv2
import numpy as np
from cv_bridge import CvBridge
from pyzbar import pyzbar
from pyzbar.pyzbar import decode
from std_msgs.msg import Header
import sys
# from tf_pub import QRSubscriber
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
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
        # with open("/home/jm/jm_ws/src/qr_localization/Pose/qr_position_fin.txt", 'r') as file:
        #     for line in file:
        #         data = list(map(str, line.split()))
        #         # print("data:", line)
        #         index_value = data[1].split(".")[0]  # 인덱스 값을 가져오기 위해 수정
        #         # print(index_value)
        #         if 'Position:' in line:
        #             pos_index = data.index('Position:')
        #             t.child_frame_id = f'Global QR'  # 고유한 child_frame_id 설정

        #             position_str = line.split("Position:")[1].strip()  
        #             position_str = position_str.split(", quat :")[0].strip()  
        #             # print(position_str)
        #             position_str = position_str.strip('[]')  

        #             # print("position_str: ", position_str)

        #             position_list = position_str.split(", ")  
        #             # position_list = position_str.split(" quat: ")  

        #             position = [float(coord.strip('[],')) for coord in position_list]  # "]" 문자 제거 후 실수형으로 변환
                    
        #             # print("Position 값:", position)
        #             # print("X 좌표:", position[0])
        #             # print("Y 좌표:", position[1])

        #             # TransformStamped 메시지에 translation 값 설정
        #             t.transform.translation.x = position[0]
        #             t.transform.translation.y = position[1]
        #             # t.transform.translation.z = position[2]  # 필요에 따라 Z 값을 설정
        #             quat_value = line.split(" quat :")[1].strip()  
        #             # print("quat_value: ", quat_value)
        #             quat_list = quat_value.strip('[]').split(" ")  # quat 값을 리스트로 변환
        #             # print("quat_list: ", quat_list)
        #             quat = [float(val) for val in quat_list]  # 문자열을 실수형으로 변환하여 리스트로 저장
        #             # yaw_radians = np.radians(angle)
        #             # print("angle: ", angle)
        #             # quat = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)

        #             # print("quat 값:", quat)

        #             # TransformStamped 메시지에 rotation 값 설정
        #             t.transform.rotation.x = quat[0]
        #             t.transform.rotation.y = quat[1]
        #             t.transform.rotation.z = quat[2]
        #             t.transform.rotation.w = quat[3]
        #             # print("transform_rotation : ", t.transform.rotation)
        #             self.tf_broadcaster.sendTransform(t)

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


class GetRange:
    def __init__(self, frame):
        self.frame = frame
        self.undistorted = None
        self.points = None
        self.left_angle = None
        self.right_angle = None
        self.qr_angle = None
        self.corrected_angle = None
        self.result = None
        self.bug = None
        self.check = None
        self.qr_detector = cv2.QRCodeDetector()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvec = None
        self.tvec = None
        self.pre_axis = None
    def undistort_image(self):
        self.camera_matrix = np.array([[634.01677684, 0.0, 273.10834464],
                                  [0.0, 634.68641236, 252.47644385],
                                  [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([[-3.84862231e-01,  1.21797133e-01, -3.45009993e-03, -3.17162086e-04, 2.96403796e-02]])

        h, w = self.frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        x, y, w, h = roi
        self.undistorted = cv2.undistort(self.frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        self.undistorted = self.undistorted[y:y + h, x:x + w]
        # self.show_axes(camera_matrix, dist_coeffs)

    def qr_detection(self):
        self.undistort_image()
        self.qr_detector = cv2.QRCodeDetector()
        retval, points, _ = self.qr_detector.detectAndDecode(self.undistorted)
        if retval:
            points = points.astype(np.int32)
            points = points.reshape((-1, 1, 2))
            self.check = cv2.polylines(self.undistorted, [points], True, (0, 0, 0), 3)
            self.points = points
        else:
            self.points = None
            self.check = self.undistorted


    def get_angle(self):
        self.qr_detection()

        if self.points is not None and len(self.points) >= 4:
            left_x = self.points[3][0][0]
            right_x = self.points[2][0][0]
            center_x = np.mean(self.points[:, 0, 0])

            fov = 59.558346608095846
            cx = self.undistorted.shape[1] / 2
            cy = self.undistorted.shape[0] / 2
            real_offset = 0
            image_offset = 0
            self.qr_angle = (center_x - cx) / self.undistorted.shape[1] * fov
            self.corrected_angle = self.qr_angle + real_offset + image_offset
            self.left_angle = (-1)*((left_x - cx) / self.undistorted.shape[1] * fov + real_offset + image_offset)
            self.right_angle = (-1)*((right_x - cx) / self.undistorted.shape[1] * fov + real_offset + image_offset)

            # Adjust angles based on camera position (-90 degrees offset)
            self.corrected_angle += 90
            self.left_angle += 90
            self.right_angle += 90

            # Normalize angles to be within the range [0, 360)
            self.corrected_angle = self.corrected_angle % 360
            self.left_angle = self.left_angle % 360
            self.right_angle = self.right_angle % 360
            print("present angle: ", self.corrected_angle)
        else:
            return None
        
    def get_qr_coords(self, cmtx, dist, points):

        qr_edges = np.array([[0,0,0],
                            [0,1,0],
                            [1,1,0],
                            [1,0,0]], dtype = 'float32').reshape((4,1,3))

        ret, rvec, tvec = cv2.solvePnP(qr_edges, points, cmtx, dist)

        unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
        if ret:
            points, _ = cv2.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
            return points, rvec, tvec

        else: return [], [], []

    def show_axes(self):
        print("TF Generator Running")
        img = self.frame
        ret_qr, points = self.qr_detector.detect(img) 

        if ret_qr:
            axis_points, self.rvec, self.tvec = self.get_qr_coords(self.camera_matrix, self.dist_coeffs, points)
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 0, 0)]

            if self.pre_axis is not None:
                differences = [self.pre_axis[i] - axis_points[i] for i in range(len(self.pre_axis))]
                print("각 원소들 간의 차이:", differences)
            self.pre_axis = axis_points
            if len(axis_points) > 0:
                axis_points = axis_points.reshape(-1, 2)
                origin = (int(axis_points[0][0]), int(axis_points[0][1]))
                for p, c in zip(axis_points[1:], colors[:3]):
                    p = (int(p[0]), int(p[1]))
                    # print("axis points : ", axis_points)
                    if origin[0] > 5 * img.shape[1] or origin[1] > 5 * img.shape[1]:
                        break
                    if p[0] > 5 * img.shape[1] or p[1] > 5 * img.shape[1]:
                        break
                    # cv2.line(img, origin, p, c, 5)
        # cv2.imshow('frame', img)

        # k = cv2.waitKey(1)

class QRCamera(Node):
    def __init__(self):
        super().__init__('qr_camera') 
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(CompressedImage, '/camera/image', self.image_callback, qos)
        self.check_pub = self.create_publisher(Image, '/camera/qr_cam', 20)
        self.ranged_pub = self.create_publisher(LaserScan, '/filtered', qos)
        # self.image_msg = self.create_publisher(Image, '/camera/image_noncompressed', 10)
        self.laserscan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.qr_publiseher = self.create_publisher(LaserScan, '/qr_index_number', qos)
        self.publisher_rvec = self.create_publisher(Float32MultiArray, 'qr_rvec', 10)
        self.publisher_tvec = self.create_publisher(Float32MultiArray, 'qr_tvec', 10)
        self.angle_publish = self.create_publisher(Float32, '/current_angle', 10)
        self.subscription_car_pose = self.create_subscription(
            PoseStamped,
            '/cartographer_pose',
            self.car_pose_callback,
            qos
        )
        self.tf_pub = QRSubscriber()
        self.float = Float32()
        self.frame = None
        self.qr = None

    def car_pose_callback(self, msg):
        self.robot_pose = msg.pose
        print("Robot Pose: ", self.robot_pose)

        self.car_pose_timestamp = msg.header.stamp
        print(f"\n\n\n{self.robot_pose.position}\n\n\n")
        print("corrected_angle: ", self.qr.corrected_angle)
        if self.qr.corrected_angle is not None:
            self.tf_pub.publish_tf_local(self.get_clock().now().to_msg(), self.robot_pose.position, self.qr.corrected_angle)
            print("Local TF Publishing. . . ")

    def decode_qr_code(self, frame):
        decoded_objects = pyzbar.decode(frame)
        for obj in decoded_objects:
            (x,y,w,h) = obj.rect
            qr_data = obj.data.decode("utf-8")
            qr_type = obj.type

            text = f"{qr_type}: {qr_data}"
            # cv2.putText(frame, text, (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))

            try:
                data = float(qr_data)
                qr_msg = LaserScan()
                qr_msg.header = Header()
                qr_msg.header.stamp = self.get_clock().now().to_msg()
                qr_msg.header.frame_id = 'qr_index'
                qr_msg.angle_min = np.radians(0)
                qr_msg.angle_max = np.radians(1)
                qr_msg.time_increment = 0.0
                qr_msg.range_min = 0.0
                qr_msg.range_max = 100.0
                qr_msg.ranges = [data, data]
                qr_msg.intensities = []
                self.qr_publiseher.publish(qr_msg)
            except ValueError:
                self.get_logger().error(f"Could not convert QR code data to float: '{qr_data}'")

        # return frame
    
    def image_callback(self, msg):
        try:
            self.frame = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.qr = GetRange(self.frame)
            self.qr.qr_detection()

            check_msg = self.bridge.cv2_to_imgmsg(self.qr.check, encoding='mono8')
            self.check_pub.publish(check_msg)
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))

    def lidar_callback(self, lidar_msg):
        if self.frame is not None:
            self.qr.qr_detection()
            print(self.qr.points)
            if self.qr.points is not None:
                print("debug")
                self.qr.get_angle()
                self.float.data = self.qr.corrected_angle
                print("self.float: ", self.float.data)
                self.angle_publish.publish(self.float)
                print("current angle: ",self.qr.corrected_angle)
                new_maxang = np.radians(self.qr.right_angle) 
                new_minang = np.radians(self.qr.left_angle) 
                if new_minang < 0:
                    new_minang += 2 * np.pi
                if new_maxang < 0:
                    new_maxang += 2 * np.pi

                if new_maxang < new_minang:
                    new_minang, new_maxang = new_maxang, new_minang

                filtered_ranges = []
                for i, distance in enumerate(lidar_msg.ranges):
                    angle_rad = lidar_msg.angle_min + i * lidar_msg.angle_increment
                    angle_rad = angle_rad % (2 * np.pi)
                    if new_minang <= angle_rad <= new_maxang:
                        filtered_ranges.append(float(distance))

                if filtered_ranges:
                    new_scan = LaserScan()
                    new_scan.header = lidar_msg.header
                    new_scan.angle_min = new_minang
                    new_scan.angle_max = new_maxang
                    new_scan.angle_increment = lidar_msg.angle_increment
                    new_scan.time_increment = lidar_msg.time_increment
                    new_scan.scan_time = lidar_msg.scan_time
                    new_scan.range_min = lidar_msg.range_min
                    new_scan.range_max = lidar_msg.range_max
                    new_scan.ranges = filtered_ranges
                    new_scan.intensities = []

                    self.ranged_pub.publish(new_scan)
            self.decode_qr_code(self.frame) # decode qr_code
            
            self.qr.show_axes()
            if self.qr.rvec is not None: 
                self.publish_tf(self.qr.rvec, self.qr.tvec)
            self.tf_pub.publish_tf(self.get_clock().now().to_msg())

    def publish_tf(self, rvec, tvec):        
        if len(rvec) > 0 and len(tvec) > 0:
            msg_rvec = Float32MultiArray()
            msg_rvec.data = rvec.flatten().tolist()  # Convert numpy array to list
            self.publisher_rvec.publish(msg_rvec)
            # print("Publishing: ", msg_rvec)
            msg_tvec = Float32MultiArray()
            msg_tvec.data = tvec.flatten().tolist()  # Convert numpy array to list
            self.publisher_tvec.publish(msg_tvec)
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    qr_camera= QRCamera()
    rclpy.spin(qr_camera)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

