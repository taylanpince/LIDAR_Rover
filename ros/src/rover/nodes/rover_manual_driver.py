import rospy
import sys
import time
import serial
import math

from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, LaserScan

from rover.arduino import ArduinoController
from rover.commands import *


JOYSTICK_MAX = 65535

AVG_STEPS_PER_SCAN = 3560
MAX_SCAN_SIZE_CM = 500
TOTAL_MAP_SCAN_POINTS = 360 * 3

SERIAL_BUFFER_TIME = 500

PORT_NAME = "/dev/ttyUSB0"


class RoverController:
    def __init__(self):
        self.left_motor_pos = 0
        self.right_motor_pos = 0
        self.left_speed = 0
        self.right_speed = 0
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
        self.activate_lidar = False
        
        self.serial_conn = serial.Serial(PORT_NAME, 57600, timeout=10)
        self.arduino = ArduinoController(self.serial_conn)
        
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
        LaserScan
        std_msgs/Header header # timestamp in the header is the acquisition time of
                                     # the first ray in the scan.
                                     #
                                     # in frame frame_id, angles are measured around
                                     # the positive Z axis (counterclockwise, if Z is up)
                                     # with zero angle being forward along the x axis

        float32 angle_min            # start angle of the scan [rad]
        float32 angle_max            # end angle of the scan [rad]
        float32 angle_increment      # angular distance between measurements [rad]

        float32 time_increment       # time between measurements [seconds] - if your scanner
                                     # is moving, this will be used in interpolating position
                                     # of 3d points
        float32 scan_time            # time between scans [seconds]

        float32 range_min            # minimum range value [m]
        float32 range_max            # maximum range value [m]

        float32[] ranges             # range data [m]
                                     # (Note: values < range_min or > range_max should be discarded)
        float32[] intensities        # intensity data [device-specific units].  If your
                                     # device does not provide intensities, please leave
                                     # the array empty.
        """
        scan = LaserScan()
        
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "scan"
        
        scan.angle_min = math.radians(0.0)
        scan.angle_max = math.radians(359.0)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(scans) - 1)
        scan.scan_time = scan_time
        scan.time_increment = scan_time / (len(scans) - 1)
        scan.range_min = 0.1
        scan.range_max = 40.0
        
        ranges = []
        
        for scan_pos, scan_dist in scans:
            ranges.append(float(scan_dist) / 100.0)
        
        scan.ranges = ranges
        
        scan_publisher.publish(scan)
    
    def leftMotorCallback(self, speed):
        self.left_speed = speed.data
        
    def rightMotorCallback(self, speed):
        self.right_speed = speed.data

    def send_motor_command(self, motor, speed):
        motor_power = abs(speed)
        
        if self.reverse_motor_direction:
            motor = MOTOR_LEFT if motor == MOTOR_RIGHT else MOTOR_RIGHT
            motor_direction = DIRECTION_BACKWARDS if speed >= 0 else DIRECTION_FORWARDS
        else:
            motor_direction = DIRECTION_FORWARDS if speed >= 0 else DIRECTION_BACKWARDS

        self.arduino.send_command(motor, motor_direction, motor_power)

    def send_motor_commands(self):
        now = time.time_ns() // 1_000_000
        
        if now - self.last_motor_command_time < 250:
            return
        
        self.last_motor_command_time = now
        
        self.send_motor_command(MOTOR_LEFT, self.left_speed)
        self.send_motor_command(MOTOR_RIGHT, self.right_speed)

    def main(self):
        rospy.init_node('rover_controller')
        
        lwheel_publisher = rospy.Publisher('~lwheel_ticks', Int32, queue_size=10)
        rwheel_publisher = rospy.Publisher('~rwheel_ticks', Int32, queue_size=10)
        imu_publisher = rospy.Publisher('~imu_link', Imu, queue_size=10)
        scan_publisher = rospy.Publisher('~scan_link', LaserScan, queue_size=10)
        
        self.reverse_motor_direction = rospy.get_param('~reverse_motor_direction')
        self.activate_lidar = rospy.get_param('~activate_lidar', False)
        
        rospy.Subscriber('~lwheel_speed', Int32, self.leftMotorCallback)
        rospy.Subscriber('~rwheel_speed', Int32, self.rightMotorCallback)
        
        if self.activate_lidar:
            self.arduino.send_button_command(SCAN_START_COMMAND)
        
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

