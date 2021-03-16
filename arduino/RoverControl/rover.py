#!/usr/bin/env python

import pygame
import sys
import time
import serial
import math


SCREEN_WIDTH = 600
SCREEN_HEIGHT = 600
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

INCOMING_MESSAGE_BEGIN = 24
INCOMING_MESSAGE_END = 23

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


def monitor_joystick():
    pygame.init()

    try:
        controller = pygame.joystick.Joystick(0)
    except:
        print("No controller found, make sure it's connected")
        return

    controller.init()
    
    serial_conn = serial.Serial(PORT_NAME, 57600, timeout=10)
    arduino = ArduinoController(serial_conn)
    
    clock = pygame.time.Clock()

    print("Controller connected")

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    
    pygame.display.set_caption('Rover Control')
    
    # pygame.draw.circle(screen, (0, 0, 0), (SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2), (SCREEN_WIDTH / 2) - 100, 2)
    
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
    
    last_motor_time = 0
    left_motor_pos = 0
    right_motor_pos = 0
    scanner_pos = 0
    scanner_dist = 0
    eventbutton = 0
    current_speed = 0
    
    full_power_begin_time = 0
    left_ticks_at_full_power_begin = 0
    right_ticks_at_full_power_begin = 0
    full_power_speed = 0
    full_power_enabled = False

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
        
        incoming_commands = arduino.read_incoming_data()
        
        for command in incoming_commands:
            data_type, payload = command

            if data_type == INCOMING_DATA_TYPE_MOTOR:
                new_left_motor_pos, new_right_motor_pos = payload
                
                left_delta = new_left_motor_pos - left_motor_pos
                right_delta = new_right_motor_pos - right_motor_pos
                new_motor_time = time.time_ns()
                
                if last_motor_time > 0:
                    time_delta = new_motor_time - last_motor_time
                    current_speed = float(left_delta + right_delta) / 2.0 / time_delta
                
                left_motor_pos = new_left_motor_pos
                right_motor_pos = new_right_motor_pos
        
        screen.fill(COLOR_WHITE)
        printer.reset()
        
        printer.write(screen, "Current Speed: {:10.4f}".format(current_speed))
        printer.write(screen, "Last Full Power Speed: {:10.4f}".format(full_power_speed))
        printer.write(screen, "Left Motor: {}".format(left_motor_pos))
        printer.write(screen, "Right Motor: {}".format(right_motor_pos))
        
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
            
            old_power_total = abs(current_left_power + current_right_power)
            
            left_power = controller.get_axis(1)
            right_power = controller.get_axis(3)
            left_speed = int(abs(left_power) * MAX_POWER)
            right_speed = int(abs(right_power) * MAX_POWER)
            
            full_power_enabled = (abs(left_power + right_power) == 2.0)
            
            # Did we just get into full power mode?
            if old_power_total < 2.0 and full_power_enabled:
                full_power_begin_time = time.time_ns()
                left_ticks_at_full_power_begin = left_motor_pos
                right_ticks_at_full_power_begin = right_motor_pos
            elif old_power_total == 2.0 and not full_power_enabled:
                left_delta = left_motor_pos - left_ticks_at_full_power_begin
                right_delta = right_motor_pos - right_ticks_at_full_power_begin
                time_delta = time.time_ns() - full_power_begin_time
                print("Full power done | Left Delta: {} | Right Delta: {} | Time Delta: {}".format(left_delta, right_delta, time_delta))
                full_power_speed = float((left_delta + right_delta) / 2.0 / time_delta)
            
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

        if full_power_enabled:
            printer.write(screen, "Full Power ON")

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
    
    serial_conn.close()
    pygame.quit()


if __name__ == "__main__":
    sys.exit(monitor_joystick())
