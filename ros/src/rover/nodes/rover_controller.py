import rospy
import sys
import time
import serial
import math

from std_msgs.msg import Int32


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

INCOMING_MESSAGE_BEGIN = 24
INCOMING_MESSAGE_END = 23

SCAN_START_COMMAND = b'S'
SCAN_STOP_COMMAND = b'X'

AVG_STEPS_PER_SCAN = 3560
MAX_SCAN_SIZE_CM = 500
TOTAL_MAP_SCAN_POINTS = 360 * 3

PORT_NAME = "/dev/ttyUSB0"


class ArduinoController:
    def __init__(self, conn):
        self.conn = conn
    
    def extract_messages(self, payload):
        messages = []
        message = []
        
        for val in payload:
            if val == INCOMING_MESSAGE_BEGIN:
                message = []
            elif val == INCOMING_MESSAGE_END:
                if len(message) == 10:
                    messages.append(message)
            else:
                message.append(val)
        
        return messages
    
    def parse_message(self, payload):
        data_type = payload[0]

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
    
    def read_incoming_data(self):
        if self.conn.in_waiting < 12:
            return []
        
        payload = list(self.conn.read_until(expected=bytes([INCOMING_MESSAGE_END]), size=24))
        commands = []
        
        for message in self.extract_messages(payload):
            command = self.parse_message(message)
            
            if command:
                commands.append(command)
        
        return commands
    
    def send_button_command(self, command):
        self.conn.write(command)
        self.conn.write(COMMAND_END)
    
    def send_command(self, motor, direction, speed):
        self.conn.write(motor)
        self.conn.write(direction)
        self.conn.write(str(speed).encode())
        self.conn.write(COMMAND_END)


class RoverController:
    def __init__(self):
        self.left_motor_pos = 0
        self.right_motor_pos = 0
        self.left_speed = 0
        self.right_speed = 0
        self.maxMotorSpeed = 3000
        
        self.serial_conn = serial.Serial(PORT_NAME, 57600, timeout=10)
        self.arduino = ArduinoController(serial_conn)
    
    def leftMotorCallback(self, speed):
        self.left_speed = speed
        
        motor_power = MAX_POWER * math.abs(self.left_speed) / self.max_motor_speed
        motor_direction = DIRECTION_FORWARDS if self.left_speed >= 0 else DIRECTION_BACKWARDS
        
        self.arduino.send_command(MOTOR_LEFT, motor_direction, motor_power)
        
    def rightMotorCallback(self, speed):
        self.right_speed = speed
        
        motor_power = MAX_POWER * math.abs(self.right_speed) / self.max_motor_speed
        motor_direction = DIRECTION_FORWARDS if self.right_speed >= 0 else DIRECTION_BACKWARDS
        
        self.arduino.send_command(MOTOR_RIGHT, motor_direction, motor_power)

    
    def main():
        self.maxMotorSpeed = int(rospy.get_param('~max_motor_speed'))
        
        lwheel_publisher = rospy.Publisher('~lwheel_ticks', Int32, queue_size=10)
        rwheel_publisher = rospy.Publisher('~rwheel_ticks', Int32, queue_size=10)
        
        rospy.init_node('rover_controller')
        rate = rospy.Rate(10) # 10hz
        
        rospy.Subscriber('~lwheel_desired_rate', Int32, self.leftMotorCallback)
        rospy.Subscriber('~rwheel_desired_rate', Int32, self.rightMotorCallback)
        
        while not rospy.is_shutdown():
            incoming_commands = self.arduino.read_incoming_data()
            
            for command in incoming_commands:
                data_type, payload = command

                if data_type == INCOMING_DATA_TYPE_MOTOR:
                    self.left_motor_pos, self.right_motor_pos = payload
            
            #rospy.loginfo("Wheels: Left {:03d} | Right {:03d}".format(left_motor_pos, right_motor_pos))

            lwheel_publisher.publish(self.left_motor_pos)
            rwheel_publisher.publish(self.right_motor_pos)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = RoverController()
        node.main()
    except rospy.ROSInterruptException:
        pass

