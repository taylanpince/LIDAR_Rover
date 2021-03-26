import math

from rover.commands import *


class ArduinoController:
    ENCODER_MIN_VALUE = 0
    ENCODER_MAX_VALUE = 65535

    def __init__(self, conn):
        self.conn = conn
        self.last_lmotor = 0
        self.last_rmotor = 0
        self.delta_lmotor = 0
        self.delta_rmotor = 0
        self.motor_range = ArduinoController.ENCODER_MAX_VALUE + 1
        self.motor_low_threshold = self.motor_range * 30 // 100
        self.motor_hi_threshold = self.motor_range * 70 // 100
    
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
    
    def get_motor_delta(self, new_value, last_value):
        if last_value > self.motor_hi_threshold and new_value < self.motor_low_threshold:
            # Wrapped around the upper limit
            return new_value + self.motor_range - last_value
        elif last_value < self.motor_low_threshold and new_value > self.motor_hi_threshold:
            # Wrapped around the lower limit
            return new_value - self.motor_range - last_value
        else:
            return new_value - last_value
    
    def parse_message(self, payload):
        data_type = payload[0]

        if data_type == INCOMING_DATA_TYPE_SCAN:
            pos = int.from_bytes(payload[1:3], byteorder="big")
            dist = int.from_bytes(payload[3:5], byteorder="big")
        
            return (data_type, (pos, dist))
        elif data_type == INCOMING_DATA_TYPE_MOTOR:
            lmotor_data = int.from_bytes(payload[1:3], byteorder="big", signed=True)
            rmotor_data = int.from_bytes(payload[3:5], byteorder="big", signed=True)
            
            self.delta_lmotor += self.get_motor_delta(lmotor_data, self.last_lmotor)
            self.delta_rmotor += self.get_motor_delta(rmotor_data, self.last_rmotor)

            self.last_lmotor = lmotor_data
            self.last_rmotor = rmotor_data
            
            return (data_type, (self.delta_lmotor, self.delta_rmotor))
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
