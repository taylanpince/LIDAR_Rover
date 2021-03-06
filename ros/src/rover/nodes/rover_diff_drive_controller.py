import rospy
import sys
import time
import serial
import math
import numpy as np

from scipy import interpolate

from std_msgs.msg import Int32, Float32, Bool
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, LaserScan

from rover.arduino import ArduinoController
from rover.commands import *


MAX_POWER = 255
MIN_POWER = 125

TOTAL_SCAN_SAMPLES_PER_REV = 720
MAX_STEPS_PER_SCAN_REV = 3640
SCAN_X_SPACE_RADS = np.linspace(-math.pi, math.pi, TOTAL_SCAN_SAMPLES_PER_REV)

PORT_NAME = "/dev/ttyUSB0"


class RoverController:
    def __init__(self):
        self.left_motor_pos = 0
        self.right_motor_pos = 0
        self.left_speed = 0
        self.right_speed = 0
        self.max_motor_speed = 3000
        self.last_orientation = None
        self.last_gyro = None
        self.last_acceleration = None
        self.last_motor_command_time = 0
        self.ori_cov = 0.0025 # Orientation covariance
        self.vel_cov = 0.02 # Angular velocity covariance
        self.acc_cov = 0.04 # Linear acceleration covariance
        self.reverse_motor_direction = False

        self.last_scan_start_time = None
        self.scan_points = []
        self.lidar_active = False
        
        self.serial_conn = serial.Serial(PORT_NAME, 57600, timeout=10)
        self.arduino = ArduinoController(self.serial_conn)
    
    def leftMotorCallback(self, speed):
        self.left_speed = min(speed.data * (MAX_POWER - MIN_POWER) / 1200, 130)
        
    def rightMotorCallback(self, speed):
        self.right_speed = min(speed.data * (MAX_POWER - MIN_POWER) / 1200, 130)
        
    def lidarStatusCallback(self, status):
        if self.lidar_active == status.data:
            return
        
        self.lidar_active = status.data
        
        if self.lidar_active:
            self.arduino.send_button_command(SCAN_START_COMMAND)
        else:
            self.arduino.send_button_command(SCAN_STOP_COMMAND)

    def send_motor_command(self, motor, speed):
        if self.reverse_motor_direction:
            motor = MOTOR_LEFT if motor == MOTOR_RIGHT else MOTOR_RIGHT
            motor_direction = DIRECTION_BACKWARDS if speed >= 0 else DIRECTION_FORWARDS
        else:
            motor_direction = DIRECTION_FORWARDS if speed >= 0 else DIRECTION_BACKWARDS
        
        if abs(speed) > 10:
            motor_power = MIN_POWER + abs(int(speed))
        else:
            motor_power = 0

        self.arduino.send_command(motor, motor_direction, motor_power)

    def send_motor_commands(self):
        now = time.time_ns() // 1_000_000
        
        if now - self.last_motor_command_time < 250:
            return
        
        self.last_motor_command_time = now
        
        self.send_motor_command(MOTOR_LEFT, self.left_speed)
        self.send_motor_command(MOTOR_RIGHT, self.right_speed)
        
    def publish_imu_message(self, publisher):
        if self.last_orientation is None or self.last_gyro is None or self.last_acceleration is None:
            return
        
        imu_msg = Imu()
        
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        gxf, gyf, gzf = self.last_gyro
        axf, ayf, azf = self.last_acceleration
        oxf, oyf, ozf, owf = self.last_orientation
        
        orientation = Quaternion()
        angular_velocity = Vector3()
        linear_acceleration = Vector3()
        
        orientation.x = oxf
        orientation.y = oyf
        orientation.z = ozf
        orientation.w = owf
        
        angular_velocity.x = gxf
        angular_velocity.y = gyf
        angular_velocity.z = gzf
        
        linear_acceleration.x = axf
        linear_acceleration.y = ayf
        linear_acceleration.z = azf
        
        imu_msg.orientation = orientation
        imu_msg.angular_velocity = angular_velocity
        imu_msg.linear_acceleration = linear_acceleration
        
        imu_msg.orientation_covariance[0] = self.ori_cov
        imu_msg.orientation_covariance[4] = self.ori_cov
        imu_msg.orientation_covariance[8] = self.ori_cov
        imu_msg.angular_velocity_covariance[0] = self.vel_cov
        imu_msg.angular_velocity_covariance[4] = self.vel_cov
        imu_msg.angular_velocity_covariance[8] = self.vel_cov
        imu_msg.linear_acceleration_covariance[0] = self.acc_cov
        imu_msg.linear_acceleration_covariance[4] = self.acc_cov
        imu_msg.linear_acceleration_covariance[8] = self.acc_cov
        
        publisher.publish(imu_msg)
        
        self.last_orientation = None
        self.last_gyro = None
        self.last_acceleration = None
    
    def publish_scan(self, scan_publisher, scan_time, scans):
        """
        Applies interpolation to the scan to make it fit the expected range exactly
        Then publishes the scan measurements as a LaserScan message
        """
        x_points = []
        y_points = []

        for pos, dist in scans:
            if pos > MAX_STEPS_PER_SCAN_REV:
                continue
            
            x_points.append(math.radians(360.0 * float(pos) / MAX_STEPS_PER_SCAN_REV) - math.pi)
            y_points.append(dist)
        
        f = interpolate.interp1d(x_points, y_points, kind="nearest", fill_value="extrapolate")
        scan_ranges = f(SCAN_X_SPACE_RADS)
        
        scan = LaserScan()
        
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "scan_link"
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / len(scan_ranges)
        scan.scan_time = scan_time
        scan.time_increment = scan_time / len(scan_ranges)
        scan.range_min = 0.1
        scan.range_max = 1.5
        
        scan.ranges = [dist / 100.0 for dist in scan_ranges]
        
        scan_publisher.publish(scan)

    def main(self):
        rospy.init_node('rover_diff_controller')
        
        lwheel_publisher = rospy.Publisher('~left_encoder_ticks', Int32, queue_size=10)
        rwheel_publisher = rospy.Publisher('~right_encoder_ticks', Int32, queue_size=10)
        imu_publisher = rospy.Publisher('~imu_link', Imu, queue_size=10)
        scan_publisher = rospy.Publisher('~scan_link', LaserScan, queue_size=10)

        self.reverse_motor_direction = rospy.get_param('~reverse_motor_direction', True)
        self.activate_lidar = rospy.get_param('~activate_lidar', False)
        
        rospy.Subscriber('~motor_left', Int32, self.leftMotorCallback)
        rospy.Subscriber('~motor_right', Int32, self.rightMotorCallback)
        rospy.Subscriber('~scan_active', Bool, self.lidarStatusCallback)
        
        while not rospy.is_shutdown():
            incoming_commands = self.arduino.read_incoming_data()
            
            for command in incoming_commands:
                data_type, payload = command

                if data_type == INCOMING_DATA_TYPE_SCAN:
                    scanner_pos, scanner_dist = payload
                    
                    if len(self.scan_points) > 0:
                        last_scan_pos, _ = self.scan_points[-1]
                        
                        if scanner_pos < last_scan_pos:
                            scan_end_time = time.time()
                            scan_delta_time = scan_end_time - self.last_scan_start_time
                        
                            self.publish_scan(scan_publisher, scan_delta_time, self.scan_points)
                            self.scan_points = []
                    
                    if len(self.scan_points) == 0:
                        self.last_scan_start_time = time.time()
                    
                    self.scan_points.append(payload)
                elif data_type == INCOMING_DATA_TYPE_MOTOR:
                    self.left_motor_pos, self.right_motor_pos = payload
                    
                    lwheel_publisher.publish(self.left_motor_pos)
                    rwheel_publisher.publish(self.right_motor_pos)
                elif data_type == INCOMING_DATA_TYPE_QUATERNION:
                    self.last_orientation = payload
                elif data_type == INCOMING_DATA_TYPE_GYRO:
                    self.last_gyro = payload
                elif data_type == INCOMING_DATA_TYPE_ACCELEROMETER:
                    self.last_acceleration = payload
            
            self.publish_imu_message(imu_publisher)
            self.send_motor_commands()


if __name__ == '__main__':
    try:
        node = RoverController()
        node.main()
    except rospy.ROSInterruptException:
        pass

