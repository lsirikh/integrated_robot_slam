import os
import csv
from datetime import datetime
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String 
from sensor_msgs.msg import Imu, MagneticField
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
import time
import threading

import pyttsx3
from playsound import playsound

# engine = pyttsx3.init()
# engine.setProperty('rate', 200)


def quaternion_to_yaw(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)

class DataReportNode(Node):
    def __init__(self):
        super().__init__('data_report_node')

        self.get_logger().info('Start Data Report Node initialized')
         # 구독자 생성: 오도메트리 메시지
        self.odom_sub = self.create_subscription(Odometry, '/sync/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/sync/imu', self.imu_callback, 10)
        # self.mag_sub = self.create_subscription(MagneticField, '/imu/mag_raw', self.mag_callback, 10)
        self.ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_sub = self.create_subscription(String, '/cmd_key', self.cmd_key_callback, 10)
        #self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        #tf_recorder
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_data = None
        self.imu_data = None
        self.ekf_data = None

        # Variables to store previous values for acceleration calculation
        self.prev_odom = {'linear_x': 0.0, 'angular_z': 0.0}
        self.prev_imu = None
        self.prev_ekf = {'linear_x': 0.0, 'angular_z': 0.0}

        self.prev_odom_time = time.time()
        self.prev_ekf_time = time.time()

        self.rotation_command_received = False  # 회전 명령 플래그

        # Position and velocity estimates for IMU
        self.imu_position_x = 0.0
        self.imu_position_y = 0.0
        self.imu_velocity_x = 0.0
        self.imu_velocity_y = 0.0
        self.imu_reference_yaw = None
        self.prev_angular_velocity_z = 0.0
        self.prev_imu_time = time.time()
       
        # CSV 파일 경로 설정 및 로그 출력
        timestamp_str = datetime.now().strftime('%Y-%m-%d_%Hh%Mm')
        self.csv_file_path = os.path.expanduser(f'~/csv_logs/data_report_node_{timestamp_str}.csv')
        self.get_logger().info(f'CSV file path: {self.csv_file_path}')

        # self.count = 1
        # self.point_index = 0  # 데이터 포인트 인덱스 초기화

        # CSV 파일 경로의 디렉터리 생성
        os.makedirs(os.path.dirname(self.csv_file_path), exist_ok=True)

        if not os.path.exists(self.csv_file_path):
            self.get_logger().info('Creating new CSV file')
            with open(self.csv_file_path, mode='w') as file:
                writer = csv.writer(file)
                
                writer.writerow([
                                    'timestamp', 
                                    'no', 'gt_x', 'gt_y',
                                    'odom_position_x', 'odom_position_y', 'odom_orientation_yaw',
                                    'odom_linear_velocity_x', 'odom_linear_accel_x', 
                                    'odom_angular_velocity_z', 'odom_angular_accel_z',

                                    'imu_position_x', 'imu_position_y', 'imu_orientation_yaw',
                                    'imu_linear_velocity_x', 'imu_linear_acceleration_x',
                                    'imu_angular_velocity_z', 'imu_angular_acceleration_z',

                                    'ekf_position_x', 'ekf_position_y', 'ekf_orientation_yaw',
                                    'ekf_linear_velocity_x', 'ekf_linear_acceleration_x',
                                    'ekf_angular_velocity_z', 'ekf_angular_acceleration_z',
                                    'map_x', 'map_y'
                                ])
        # timer callback based csv logs storing process
        # self.timer = self.create_timer(0.5, self.timer_callback)

        self.tf_timer = self.create_timer(2.0, self.update_transform)  # Edited!!

        self.point_index = 0  # 데이터 포인트 인덱스 초기화
        self.points = [
            [1, 0, 0], [2, 0, 10], [3, 0, 20], [4, 0, 30], [5, 0, 40], [6, 0, 50], [7, 0, 60], [8, 0, 70], [9, 0, 80], [10, 0, 90],
            [11, 0, 100], [12, 0, 110], [13, 0, 120], [14, 0, 130], [15, 0, 140], [16, 0, 150], [17, 0, 160], [18, 0, 170], [19, 0, 180], [20, 0, 190],
            [21, 0, 200], [22, 0, 210], [23, 0, 220], [24, 0, 230], [25, 10, 230], [26, 20, 230], [27, 20, 220], [28, 20, 210], [29, 20, 200], [30, 20, 190],
            [31, 20, 180], [32, 20, 170], [33, 20, 160], [34, 20, 150], [35, 20, 140], [36, 20, 130], [37, 20, 120], [38, 20, 110], [39, 20, 100], [40, 20, 90],
            [41, 20, 80], [42, 20, 70], [43, 20, 60], [44, 20, 50], [45, 20, 40], [46, 20, 30], [47, 20, 20], [48, 30, 20], [49, 40, 20], [50, 40, 30],
            [51, 40, 40], [52, 40, 50], [53, 40, 60], [54, 40, 70], [55, 40, 80], [56, 40, 90], [57, 40, 100], [58, 40, 110], [59, 40, 120], [60, 40, 130],
            [61, 40, 140], [62, 40, 150], [63, 40, 160], [64, 40, 170], [65, 40, 180], [66, 40, 190], [67, 40, 200], [68, 40, 210], [69, 40, 220], [70, 40, 230],
            [71, 50, 230], [72, 60, 230], [73, 60, 220], [74, 60, 210], [75, 60, 200], [76, 60, 190], [77, 60, 180], [78, 60, 170], [79, 60, 160], [80, 60, 150],
            [81, 60, 140], [82, 60, 130], [83, 60, 120], [84, 60, 110], [85, 60, 100], [86, 60, 90], [87, 60, 80], [88, 60, 70], [89, 60, 60], [90, 60, 50],
            [91, 60, 40], [92, 60, 30], [93, 60, 20], [94, 70, 20], [95, 80, 20], [96, 80, 30], [97, 80, 40], [98, 80, 50], [99, 80, 60], [100, 80, 70],
            [101, 80, 80], [102, 80, 90], [103, 80, 100], [104, 80, 110], [105, 80, 120], [106, 80, 130], [107, 80, 140], [108, 80, 150], [109, 80, 160], [110, 80, 170],
            [111, 80, 180], [112, 80, 190], [113, 80, 200], [114, 80, 210], [115, 80, 220], [116, 80, 230], [117, 90, 230], [118, 100, 230], [119, 100, 220], [120, 100, 210],
            [121, 100, 200], [122, 100, 190], [123, 100, 180], [124, 100, 170], [125, 100, 160], [126, 100, 150], [127, 100, 140], [128, 100, 130], [129, 100, 120], [130, 100, 110],
            [131, 100, 100], [132, 100, 90], [133, 100, 80], [134, 100, 70], [135, 100, 60], [136, 100, 50], [137, 100, 40], [138, 100, 30], [139, 100, 20], [140, 110, 20],
            [141, 120, 20], [142, 120, 30], [143, 120, 40], [144, 120, 50], [145, 120, 60], [146, 120, 70], [147, 120, 80], [148, 120, 90], [149, 120, 100], [150, 120, 110],
            [151, 120, 120], [152, 120, 130], [153, 120, 140], [154, 120, 150], [155, 120, 160], [156, 120, 170], [157, 120, 180], [158, 120, 190], [159, 120, 200], [160, 120, 210],
            [161, 120, 220], [162, 120, 230], [163, 130, 230], [164, 140, 230]
        ]

    def cmd_vel_callback(self, msg):
        # 회전 명령을 감지하여 플래그 설정
        if msg.angular.z != 0 and msg.linear.x == 0:  # 회전 명령 감지 (선속도는 0, 각속도는 0이 아님)
            if not self.rotation_command_received:
                self.get_logger().info('Rotation command received, updating transform once')
                self.update_transform()  # 단 한 번 transform 업데이트
                self.rotation_command_received = True  # 이후로는 동일한 회전 명령에 대한 기록 방지
        else:
            self.rotation_command_received = False  # 직진 혹은 정지 상태로 돌아오면 플래그 해제

    def odom_callback(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z
        
        current_time = time.time()
        
        if self.prev_odom is not None and self.prev_odom_time is not None:
            # Calculate linear and angular acceleration
            delta_time = current_time - self.prev_odom_time
            # self.get_logger().info(f'delta_time:{delta_time}')
            if delta_time >= 0.2:
                
                linear_accel_x = (linear_x - self.prev_odom['linear_x']) / delta_time
                # self.get_logger().info(f"{linear_accel_x} = {linear_x - self.prev_odom['linear_x']} / {delta_time}")
                angular_accel_z = (angular_z - self.prev_odom['angular_z']) / delta_time
                # self.get_logger().info(f"{angular_accel_z} = {angular_z - self.prev_odom['angular_z']} / {delta_time}")

                self.odom_data = {
                'position_x': msg.pose.pose.position.x,
                'position_y': msg.pose.pose.position.y,
                'orientation_yaw': yaw,
                'linear_velocity_x': linear_x,
                'linear_accel_x': linear_accel_x,
                'angular_velocity_z': angular_z,
                'angular_accel_z': angular_accel_z
                }

                # Update previous data
                self.prev_odom = {'linear_x': linear_x, 'angular_z': angular_z}
                self.prev_odom_time = current_time
    
    def mag_callback(self, msg):
        return

    def imu_callback(self, msg):
        # 초기 Yaw 기준 설정
        if self.imu_reference_yaw is None:
            self.imu_reference_yaw = quaternion_to_yaw(msg.orientation)

        # 현재 Yaw 값 (기준 Yaw 값 보정)
        yaw = quaternion_to_yaw(msg.orientation) - self.imu_reference_yaw
        angular_velocity_z = msg.angular_velocity.z
        linear_accel_x = msg.linear_acceleration.x
        linear_accel_y = msg.linear_acceleration.y
        current_time = time.time()

        # 초기화된 이전 시간 값이 있는지 확인
        if self.prev_imu_time is not None:
            delta_time = current_time - self.prev_imu_time
            
            # 지정된 주기마다 업데이트 (0.2초)
            if delta_time >= 0.2:
                # self.get_logger().info(f"imu_callback - linear_accel_x : {linear_accel_x}")
                self.imu_velocity_x += linear_accel_x * delta_time

                # 각속도 변화에 따른 각가속도 계산
                angular_acceleration_z = 0.0
                if self.prev_angular_velocity_z is not None:
                    angular_acceleration_z = (angular_velocity_z - self.prev_angular_velocity_z) / delta_time
                else:
                    angular_acceleration_z = 0.0  # 초기 상태 설정

                # 이전 상태 업데이트
                self.prev_angular_velocity_z = angular_velocity_z
                self.prev_imu_time = current_time

                # IMU 데이터를 저장
                self.imu_data = {
                    'position_x': self.imu_position_x,
                    'position_y': self.imu_position_y,
                    'orientation_yaw': yaw,
                    'linear_velocity_x': self.imu_velocity_x,
                #   'linear_velocity_y': self.imu_velocity_y,
                    'linear_acceleration_x': linear_accel_x,
                #    'linear_acceleration_y': linear_accel_y,
                    'angular_velocity_z': angular_velocity_z,
                    'angular_acceleration_z': angular_acceleration_z
                }

                # 초기화 단계 - 이전 시간 값을 현재 시간으로 설정
                self.prev_imu_time = current_time

    def ekf_callback(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z
        current_time = time.time()
        
        # Calculate linear acceleration and angular acceleration similarly to odom
        if self.prev_ekf_time is not None :
            # Calculate linear and angular acceleration
            delta_time = current_time - self.prev_ekf_time
            if delta_time >= 0.2:
                linear_accel_x = (linear_x - self.prev_ekf['linear_x']) / delta_time
                angular_accel_z = (angular_z - self.prev_ekf['angular_z']) / delta_time

                self.ekf_data = {
                    'position_x': msg.pose.pose.position.x,
                    'position_y': msg.pose.pose.position.y,
                    'yaw': yaw,
                    'linear_velocity_x': linear_x,
                    'linear_acceleration_x': linear_accel_x,
                    'angular_velocity_z': angular_z,
                    'angular_acceleration_z': angular_accel_z
                }

                # Update previous EKF data
                self.prev_ekf = {'linear_x': linear_x, 'angular_z': angular_z}
                self.prev_ekf_time = current_time

    # When timer callback is called, this method runs.
    def timer_callback(self):
        self.save_data_to_csv()

    def cmd_key_callback(self, msg):
        if msg.data == "save":
            self.save_data_to_csv()
        elif msg.data == "remove":
            self.undo_last_save()
        else:
            return

    # def update_transform(self):  # Edited!!
    #     try:
    #         if not self.rotation_command_received:
    #             now = Time()
    #             trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
    #             self.translation_x = trans.transform.translation.x
    #             self.translation_y = trans.transform.translation.y
            
    #     except Exception as e:
    #         self.get_logger().warn(f'Could not get transform: {e}')
    #         self.translation_x = None
    #         self.translation_y = None
    def update_transform(self):  # Edited!!
        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            self.translation_x = trans.transform.translation.x
            self.translation_y = trans.transform.translation.y
            
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            self.translation_x = None
            self.translation_y = None

    def save_data_to_csv(self):
        if self.imu_data is None:
            self.get_logger().warn('Imu data is not available yet')
            return

        if self.odom_data is None:
            self.get_logger().warn('Odometry data is not available yet')
            return

        if self.ekf_data is None:
            self.get_logger().warn('Extended Kalman Filter data is not available yet')
            return

        if self.translation_x is None or self.translation_y is None:
            self.get_logger().warn('Transform data not available yet')
            return

        self.get_logger().info(f'point_index : {self.point_index}')
        no, x, y = self.points[self.point_index]
        self.point_index = (self.point_index + 1) % len(self.points)  # 다음 데이터 포인트로 이동

        try:
            
            # 각 데이터의 값이 None이 아닌지 확인 후 float으로 변환
            odom_values = [float(val) if isinstance(val, (float, int)) else 0.0 for val in self.odom_data.values()]
            imu_values = [float(val) if isinstance(val, (float, int)) else 0.0 for val in self.imu_data.values()]
            ekf_values = [float(val) if isinstance(val, (float, int)) else 0.0 for val in self.ekf_data.values()]


            # timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(current_time))
            # milliseconds = int((current_time - int(current_time)) * 1000)
            # timestamp_with_milliseconds = f"{timestamp}.{milliseconds:03d}"

            # 저장할 데이터를 메모리에 저장
            self.last_saved_data = [
                datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-4],
                no, x, y,
                *["{:.4f}".format(val) for val in odom_values],
                *["{:.4f}".format(val) for val in imu_values],
                *["{:.4f}".format(val) for val in ekf_values],
                f'{self.translation_x:.4f}', f'{self.translation_y:.4f}'
            ]

            with open(self.csv_file_path, mode='a') as file:
                writer = csv.writer(file)
                writer.writerow(self.last_saved_data)
            self.get_logger().info(f'a csv data record was stored...')
            
            
            # TTS 재생을 별도 스레드에서 실행
            tts_thread = threading.Thread(target=self.play_point_index_tts)
            tts_thread.start()
            # # point_index 값을 한글로 읽어주는 TTS 생성 및 재생
            # tts_text = f'{self.point_index}'
            # tts = gTTS(tts_text, lang='ko')
            # tts.save("/home/lsirikh/Effect_sounds/tmp/point_index.mp3")
            # playsound("/home/lsirikh/Effect_sounds/ding.mp3")
            # os.remove("/home/lsirikh/Effect_sounds/tmp/point_index.mp3")  # 재생 후 임시 파일 삭제
            
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

    def undo_last_save(self):
        """CSV 파일에서 마지막으로 저장된 줄을 삭제하고 되돌리는 함수"""
        if self.last_saved_data is None:
            self.get_logger().warn('No previous data to undo.')
            return

        try:
            # 모든 줄을 메모리에 읽고 마지막 줄을 제외하고 다시 씀
            with open(self.csv_file_path, 'r') as file:
                lines = file.readlines()
            with open(self.csv_file_path, 'w') as file:
                file.writelines(lines[:-1])  # 마지막 줄을 제외하고 다시 저장

            self.get_logger().info('Last saved record was removed from CSV file.')
            self.last_saved_data = None  # 마지막 데이터 초기화
            self.point_index -= 1
        except Exception as e:
            self.point_index = 0
            self.get_logger().warn(f'Could not undo last save: {e}')

    def play_point_index_tts(self):
        playsound("/home/lsirikh/Effect_sounds/ding.mp3")
    #     # """point_index 값을 한글로 읽어주는 TTS 생성 및 재생 함수 (별도 스레드에서 실행)"""
    #     # tts_text = f'{self.point_index}'
    #     # tts = gTTS(tts_text, lang='ko')
    #     # tts_file_path = f"/home/lsirikh/Effect_sounds/tmp/point_index_{self.point_index}.mp3"

    #     # # 디렉터리가 없으면 생성
    #     # os.makedirs(os.path.dirname(tts_file_path), exist_ok=True)

    #     # # TTS 파일 저장 및 재생
    #     # tts.save(tts_file_path)
    #     # playsound(tts_file_path)
    #     # os.remove(tts_file_path)  # 재생 후 임시 파일 삭제
    #     tts_text = f'숫자 {self.point_index}'
    #     engine.say(tts_text)
    #     engine.runAndWait()


def main(args=None):
    rclpy.init(args=args)
    try:
        data_report_node = DataReportNode()
        rclpy.spin(data_report_node)
    except KeyboardInterrupt:
        data_report_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        data_report_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
