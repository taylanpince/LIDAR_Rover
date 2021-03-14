#!/usr/bin/env python

import pygame
import sys
import time
import serial
import math


SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 1200
SCREEN_CENTER = SCREEN_WIDTH / 2

SCAN_START_COMMAND = b'S'
SCAN_STOP_COMMAND = b'.'

AVG_STEPS_PER_SCAN = 3560
MAX_SCAN_SIZE_CM = 500
TOTAL_MAP_SCAN_POINTS = 360 * 3

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


class ArduinoController:
    def __init__(self, conn):
        self.conn = conn
    
    def read_scan(self):
        payload = list(self.conn.read(size=4))
        
        pos = int.from_bytes(payload[:2], byteorder="big")
        dist = int.from_bytes(payload[2:], byteorder="big")
        
        return (pos, dist)
    
    def send_button_command(self, command):
        self.conn.write(command)
    
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
    
    pygame.display.set_caption('LIDAR')
    
    printer = TextPrint()
    running = True
    scanning = False
    map_points = []

    while running:
        events = pygame.event.get()
        
        for event in events:
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 4: # Y button
                    arduino.send_button_command(SCAN_START_COMMAND)
                    scanning = True
                elif event.button == 3: # X button
                    arduino.send_button_command(SCAN_STOP_COMMAND)
                    scanning = False
        
        screen.fill(COLOR_WHITE)
        printer.reset()

        if scanning:
            printer.write(screen, "Scanning")
            printer.indent()
            scan_pos, scan = arduino.read_scan()
            angle = 360 * scan_pos / AVG_STEPS_PER_SCAN
            angle_r = math.radians(angle)
            radius = (SCREEN_WIDTH / 2) * scan / MAX_SCAN_SIZE_CM
            
            x = SCREEN_CENTER + radius * math.cos(angle_r)
            y = SCREEN_CENTER + radius * math.sin(angle_r)
            
            printer.write(screen, "Angle {} deg | Distance {} cm".format(angle, scan))
            printer.unindent()
            
            map_points.append((x, y))
            
            if len(map_points) > TOTAL_MAP_SCAN_POINTS:
                map_points.pop(0)

        for index, pos in enumerate(map_points):
            color = 255 - (255 * index / TOTAL_MAP_SCAN_POINTS)
            pygame.draw.circle(screen, (color, color, color), pos, 5)

        pygame.display.flip()
        clock.tick(60)
    
    serial_conn.close()
    pygame.quit()


if __name__ == "__main__":
    sys.exit(monitor_joystick())
