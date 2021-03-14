import rclpy
import pygame
import sys
import time
import serial
import math
import numpy

from threading import Thread
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, JointState
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion


SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 1200
SCREEN_CENTER = SCREEN_WIDTH / 2

MOTOR_LEFT = b'L'
MOTOR_RIGHT = b'R'
DIRECTION_FORWARDS = b'F'
DIRECTION_BACKWARDS = b'B'
COMMAND_END = b'.'
MAX_POWER = 255

INCOMING_DATA_TYPE_SCAN = 100
INCOMING_DATA_TYPE_MOTOR = 101
INCOMING_DATA_TYPE_QUATERNION = 102
INCOMING_DATA_TYPE_GYRO = 103
INCOMING_DATA_TYPE_ACCELEROMETER = 104

SCAN_START_COMMAND = b'S'
SCAN_STOP_COMMAND = b'X'

AVG_STEPS_PER_SCAN = 3560
MAX_SCAN_SIZE_CM = 500
TOTAL_MAP_SCAN_POINTS = 360 * 3

SERIAL_BUFFER_TIME = 500

PORT_NAME = "/dev/cu.usbserial-DA011F6D"

COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)
COLOR_RED = (255, 0, 0)

LEFT_MOTOR = "Left"
RIGHT_MOTOR = "Right"


class LIDARPublisher(Node):
    """
    Publishes a LIDAR
    """
    def __init__(self):
        super().__init__('lidar_publisher')
        
        qos_profile = QoSProfile(depth=10)
        
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', qos_profile)
        self.lmotor_publisher = self.create_publisher(Int32, '/lwheel', qos_profile)
        self.rmotor_publisher = self.create_publisher(Int32, '/rwheel', qos_profile)
        self.imu_publisher = self.create_publisher(Imu, '/imu_link', qos_profile)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        self.joint_timer = self.create_timer(0.5, self.publish_joint_states)

    def log_info(self, msg):
        self.get_logger().info(msg)

    def publish_imu(self, quaternion, gyro, accelerometer):
        """
        Publish IMU message
        """
        imu = Imu()
        
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        
        gxf, gyf, gzf = gyro
        imu.angular_velocity.x = gxf
        imu.angular_velocity.y = gyf
        imu.angular_velocity.z = gzf

        axf, ayf, azf = accelerometer
        imu.linear_acceleration.x = axf
        imu.linear_acceleration.y = ayf
        imu.linear_acceleration.z = azf
        
        orientation = Quaternion()

        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        
        imu.orientation = orientation
        
        self.imu_publisher.publish(imu)

    def publish_joint_states(self):
        joint_state = JointState()
        
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['drive_wheel_l_joint', 'drive_wheel_r_joint', 'front_wheel_l_joint', 'front_wheel_r_joint']
        joint_state.position = [0.0, 0.0, 0.0, 0.0]

        self.joint_publisher.publish(joint_state)

    def publish_motor(self, left_encoder, right_encoder):
        """
        Publish encoder value for left and right motor
        """
        lmsg = Int32()
        lmsg.data = left_encoder
        
        rmsg = Int32()
        rmsg.data = right_encoder

        self.lmotor_publisher.publish(lmsg)
        self.rmotor_publisher.publish(rmsg)
        
        # self.get_logger().info('%s motor: %d' % (motor, encoder_value))

    def publish_scan(self, scan_time, scans):
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
        
        scan.header.stamp = self.get_clock().now().to_msg()
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
        
        self.lidar_publisher.publish(scan)
        self.get_logger().info('Published %d scans' % len(scans))
        self.get_logger().info('Angle min: %f' % scan.angle_min)
        self.get_logger().info('Angle max: %f' % scan.angle_max)
        self.get_logger().info('Angle increment: %f' % scan.angle_increment)
        self.get_logger().info('Scan time: %f' % scan.scan_time)
        self.get_logger().info('Time increment: %f' % scan.time_increment)
        self.get_logger().info(str(ranges))


class TextPrint:
    """
    This is a simple class that will help us print to the screen
    """
    def __init__(self):
        self.reset()
        self.x_pos = 10
        self.y_pos = 10
        self.font = pygame.font.Font(None, 20)
 
    def write(self, my_screen, text_string):
        """ Draw text onto the screen. """
        text_bitmap = self.font.render(text_string, True, COLOR_BLACK)
        my_screen.blit(text_bitmap, [self.x_pos, self.y_pos])
        self.y_pos += self.line_height
 
    def reset(self):
        """ Reset text to the top of the screen. """
        self.x_pos = 10
        self.y_pos = 10
        self.line_height = 15
 
    def indent(self):
        """ Indent the next line of text """
        self.x_pos += 10
 
    def unindent(self):
        """ Unindent the next line of text """
        self.x_pos -= 10


class TargetDot:
    """
    A simple visualizer for joystick control
    """
    def __init__(self, x, y, screen):
        self.x = x
        self.y = y
        self.screen = screen
        self.color = (255, 0, 0)
    
    def display(self):
        pygame.draw.circle(
            self.screen, 
            self.color, 
            ((self.x * SCREEN_WIDTH / 2) + (SCREEN_WIDTH / 2), 
            (self.y * SCREEN_HEIGHT / 2) + (SCREEN_HEIGHT / 2)), 
            20, 0)


class ArduinoController:
    def __init__(self, conn, publisher):
        self.conn = conn
        self.publisher = publisher
    
    def read_incoming_data(self):
        if self.conn.in_waiting < 10:
            return None
        
        payload = list(self.conn.read(size=10))
        data_type = payload[0]

        # self.publisher.log_info("{}".format(payload))

        if data_type == INCOMING_DATA_TYPE_SCAN:
            pos = int.from_bytes(payload[1:3], byteorder="big")
            dist = int.from_bytes(payload[3:5], byteorder="big")
        
            return (data_type, (pos, dist))
        elif data_type == INCOMING_DATA_TYPE_MOTOR:
            lmotor_data = int.from_bytes(payload[1:5], byteorder="big", signed=True)
            rmotor_data = int.from_bytes(payload[5:9], byteorder="big", signed=True)
            
            return (data_type, (lmotor_data, rmotor_data))
        elif data_type == INCOMING_DATA_TYPE_QUATERNION:
            w = int.from_bytes(payload[1:3], byteorder="big", signed=True)
            x = int.from_bytes(payload[3:5], byteorder="big", signed=True)
            y = int.from_bytes(payload[5:7], byteorder="big", signed=True)
            z = int.from_bytes(payload[7:9], byteorder="big", signed=True)
            
            wf = float(w) / 16384.0
            xf = float(x) / 16384.0
            yf = float(y) / 16384.0
            zf = float(z) / 16384.0
            
            return (data_type, (xf, yf, zf, wf))
        elif data_type == INCOMING_DATA_TYPE_GYRO:
            gx = int.from_bytes(payload[1:3], byteorder="big", signed=True)
            gy = int.from_bytes(payload[3:5], byteorder="big", signed=True)
            gz = int.from_bytes(payload[5:7], byteorder="big", signed=True)
            
            gxf = float(gx) * (4000.0 / 65536.0) * (math.pi / 180.0) * 25.0
            gyf = float(gy) * (4000.0 / 65536.0) * (math.pi / 180.0) * 25.0
            gzf = float(gz) * (4000.0 / 65536.0) * (math.pi / 180.0) * 25.0
            
            return (data_type, (gxf, gyf, gzf))
        elif data_type == INCOMING_DATA_TYPE_ACCELEROMETER:
            ax = int.from_bytes(payload[1:3], byteorder="big", signed=True)
            ay = int.from_bytes(payload[3:5], byteorder="big", signed=True)
            az = int.from_bytes(payload[5:7], byteorder="big", signed=True)
            
            axf = float(ax) * (8.0 / 65536.0) * 9.81
            ayf = float(ay) * (8.0 / 65536.0) * 9.81
            azf = float(az) * (8.0 / 65536.0) * 9.81
            
            return (data_type, (axf, ayf, azf))
        
        return None
    
    def send_button_command(self, command):
        self.conn.write(command)
        self.conn.write(COMMAND_END)
    
    def send_command(self, motor, direction, speed):
        self.conn.write(motor)
        self.conn.write(direction)
        self.conn.write(str(speed).encode())
        self.conn.write(COMMAND_END)


def quaternion_conjugate(quaternion):
    """Return conjugate of quaternion.
    >>> q0 = random_quaternion()
    >>> q1 = quaternion_conjugate(q0)
    >>> q1[3] == q0[3] and all(q1[:3] == -q0[:3])
    True
    """
    return numpy.array((-quaternion[0], -quaternion[1],
                        -quaternion[2], quaternion[3]), dtype=numpy.float64)

def quaternion_inverse(quaternion):
    """Return inverse of quaternion.
    >>> q0 = random_quaternion()
    >>> q1 = quaternion_inverse(q0)
    >>> numpy.allclose(quaternion_multiply(q0, q1), [0, 0, 0, 1])
    True
    """
    return quaternion_conjugate(quaternion) / numpy.dot(quaternion, quaternion)

def quaternion_multiply(quaternion1, quaternion0):
    """Return multiplication of two quaternions.
    >>> q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> numpy.allclose(q, [-44, -14, 48, 28])
    True
    """
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return numpy.array((
         x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
         x1*y0 - y1*x0 + z1*w0 + w1*z0,
        -x1*x0 - y1*y0 - z1*z0 + w1*w0), dtype=numpy.float64)


def main(args=None):
    rclpy.init(args=args)

    lidar_publisher = LIDARPublisher()
    
    spin_thread = Thread(target=rclpy.spin, args=(lidar_publisher,))
    spin_thread.start()

    pygame.init()

    try:
        controller = pygame.joystick.Joystick(0)
    except:
        lidar_publisher.log_info("No controller found, make sure it's connected")
        return

    controller.init()
    
    serial_conn = serial.Serial(PORT_NAME, 57600, timeout=10)
    arduino = ArduinoController(serial_conn, lidar_publisher)
    
    clock = pygame.time.Clock()

    lidar_publisher.log_info("Controller connected")

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    
    pygame.display.set_caption('Rover Control')
    
    printer = TextPrint()
    target_dot = TargetDot(0, 0, screen)
    vertical_pos = 0.0
    horizontal_pos = 0.0
    last_command_time = 0
    current_left_power = 0.0
    current_right_power = 0.0
    running = True
    scanning = False
    map_points = []
    last_scan_start_time = None
    scan_points = []
    
    left_motor_pos = 0
    right_motor_pos = 0
    scanner_pos = 0
    scanner_dist = 0
    eventbutton = 0
    
    inversed_zero_orientation = None
    zero_orientation = None
    orientation = None
    diff_rotation = None
    accelerometer = None
    gyro = None

    while running:
        events = pygame.event.get()

        for event in events:
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                eventbutton = event.button
                if event.button == 4: # Y button
                    arduino.send_button_command(SCAN_START_COMMAND)
                    scanning = True
                elif event.button == 3: # X button
                    arduino.send_button_command(SCAN_STOP_COMMAND)
                    scanning = False
        
        incoming_payload = arduino.read_incoming_data()
        
        if incoming_payload:
            data_type, payload = incoming_payload

            if data_type == INCOMING_DATA_TYPE_SCAN:
                scanner_pos, scanner_dist = payload
                
                if len(scan_points) > 0:
                    last_scan_pos, _ = scan_points[-1]
                    
                    if scanner_pos < last_scan_pos:
                        scan_end_time = time.time()
                        scan_delta_time = scan_end_time - last_scan_start_time
                        
                        lidar_publisher.publish_scan(scan_delta_time, scan_points)
                        scan_points = []
                
                if len(scan_points) == 0:
                    last_scan_start_time = time.time()
                
                scan_points.append(payload)
            elif data_type == INCOMING_DATA_TYPE_MOTOR:
                left_motor_pos, right_motor_pos = payload
                lidar_publisher.log_info("Motor values: {}".format(left_motor_pos, right_motor_pos))
                lidar_publisher.publish_motor(left_motor_pos, right_motor_pos)
            elif data_type == INCOMING_DATA_TYPE_QUATERNION:
                orientation = payload
                
                if zero_orientation is None:
                    zero_orientation = orientation
                
                if inversed_zero_orientation is None:
                    inversed_zero_orientation = quaternion_inverse(zero_orientation)
                
                diff_rotation = quaternion_multiply(inversed_zero_orientation, orientation)
            elif data_type == INCOMING_DATA_TYPE_GYRO:
                gyro = payload
            elif data_type == INCOMING_DATA_TYPE_ACCELEROMETER:
                accelerometer = payload
                
                lidar_publisher.publish_imu(orientation, accelerometer, gyro)
        
        screen.fill(COLOR_WHITE)
        printer.reset()
        
        printer.write(screen, "Last Event Button: {}".format(eventbutton))
        printer.write(screen, "Left Motor: {}".format(left_motor_pos))
        printer.write(screen, "Right Motor: {}".format(right_motor_pos))
        
        printer.write(screen, "IMU Data:")
        printer.indent()

        if orientation is not None:
            printer.write(screen, "Rotation: x:{:10.2f} y:{:10.2f} z:{:10.2f} w:{:10.2f}".format(orientation[0], orientation[1], orientation[2], orientation[3]))

        if accelerometer is not None:
            printer.write(screen, "Acceleration: x:{:10.2f} y:{:10.2f} z:{:10.2f}".format(accelerometer[0], accelerometer[1], accelerometer[2]))

        if gyro is not None:
            printer.write(screen, "Gyro: x:{:10.2f} y:{:10.2f} z:{:10.2f}".format(gyro[0], gyro[1], gyro[2]))

        printer.unindent()
        
        if scanning:
            printer.write(screen, "Scanning")
            printer.indent()
            angle = 360 * scanner_pos / AVG_STEPS_PER_SCAN
            angle_r = math.radians(angle)
            radius = (SCREEN_WIDTH / 2) * scanner_dist / MAX_SCAN_SIZE_CM
            
            x = SCREEN_CENTER + radius * math.cos(angle_r)
            y = SCREEN_CENTER + radius * math.sin(angle_r)
            
            printer.write(screen, "Angle {} deg | Distance {} cm".format(angle, scanner_dist))
            printer.unindent()
            
            map_points.append((x, y))
            
            if len(map_points) > TOTAL_MAP_SCAN_POINTS:
                map_points.pop(0)

        for index, pos in enumerate(map_points):
            color = 255 - (255 * index / TOTAL_MAP_SCAN_POINTS)
            pygame.draw.circle(screen, (color, color, color), pos, 5)
        
        now = time.time_ns()
        
        if now - last_command_time > SERIAL_BUFFER_TIME:
            last_command_time = now
            
            left_power = controller.get_axis(1)
            right_power = controller.get_axis(3)
            left_speed = int(abs(left_power) * MAX_POWER)
            right_speed = int(abs(right_power) * MAX_POWER)
            
            if left_power > 0.2 and current_left_power != left_power:
                left_direction = DIRECTION_BACKWARDS
                arduino.send_command(MOTOR_LEFT, left_direction, left_speed)
                current_left_power = left_power
            elif left_power < -0.2 and current_left_power != left_power:
                left_direction = DIRECTION_FORWARDS
                arduino.send_command(MOTOR_LEFT, left_direction, left_speed)
                current_left_power = left_power
            elif -0.2 <= left_power <= 0.2 and current_left_power != 0.0:
                arduino.send_command(MOTOR_LEFT, DIRECTION_FORWARDS, 0)
                current_left_power = 0.0
            
            if right_power > 0.2 and current_right_power != right_power:
                right_direction = DIRECTION_BACKWARDS
                arduino.send_command(MOTOR_RIGHT, right_direction, right_speed)
                current_right_power = right_power
            elif right_power < -0.2 and current_right_power != right_power:
                right_direction = DIRECTION_FORWARDS
                arduino.send_command(MOTOR_RIGHT, right_direction, right_speed)
                current_right_power = right_power
            elif -0.2 <= right_power <= 0.2 and  current_right_power != 0.0:
                arduino.send_command(MOTOR_RIGHT, DIRECTION_FORWARDS, 0)
                current_right_power = 0.0
        else:
            left_speed = int(abs(current_left_power) * MAX_POWER)
            right_speed = int(abs(current_right_power) * MAX_POWER)

        if current_left_power > 0.2:
            printer.write(screen, "Left Motor: Backwards {}".format(left_speed))
        elif current_left_power < -0.2:
            printer.write(screen, "Left Motor: Forwards {}".format(left_speed))
        else:
            printer.write(screen, "Left Motor: Neutral")
        
        if current_right_power > 0.2:
            printer.write(screen, "Right Motor: Backwards {}".format(right_speed))
        elif current_right_power < -0.2:
            printer.write(screen, "Right Motor: Forwards {}".format(right_speed))
        else:
            printer.write(screen, "Right Motor: Neutral")
        
        pygame.display.flip()
        clock.tick(60)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_publisher.destroy_node()
    rclpy.shutdown()
    
    # Close serial connection and clean up
    serial_conn.close()
    pygame.quit()


if __name__ == '__main__':
    main()
