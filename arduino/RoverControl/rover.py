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
INCOMING_DATA_TYPE_LEFT_MOTOR = 101
INCOMING_DATA_TYPE_RIGHT_MOTOR = 102

SCAN_START_COMMAND = b'S'
SCAN_STOP_COMMAND = b'.'

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
    
    def read_incoming_data(self):
        if self.conn.in_waiting < 5:
            return None
        
        payload = list(self.conn.read(size=5))
        data_type = payload[0]
        print(payload)
        if data_type == INCOMING_DATA_TYPE_SCAN:
            pos = int.from_bytes(payload[1:3], byteorder="big")
            dist = int.from_bytes(payload[3:], byteorder="big")
        
            return (data_type, (pos, dist))
        elif data_type == INCOMING_DATA_TYPE_LEFT_MOTOR or data_type == INCOMING_DATA_TYPE_RIGHT_MOTOR:
            motor_data = int.from_bytes(payload[1:], byteorder="big")
            
            return (data_type, motor_data)
        
        return None
    
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
    
    left_motor_pos = 0
    right_motor_pos = 0
    scanner_pos = 0
    scanner_dist = 0
    eventbutton = 0

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
            elif data_type == INCOMING_DATA_TYPE_LEFT_MOTOR:
                left_motor_pos = payload
            elif data_type == INCOMING_DATA_TYPE_RIGHT_MOTOR:
                right_motor_pos = payload
        
        screen.fill(COLOR_WHITE)
        printer.reset()
        
        printer.write(screen, "Last Event Button: {}".format(eventbutton))
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
    
    serial_conn.close()
    pygame.quit()


if __name__ == "__main__":
    sys.exit(monitor_joystick())
