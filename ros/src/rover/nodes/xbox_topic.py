import rospy
import evdev
import numpy
import time

from threading import Thread
from queue import SimpleQueue

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


MAX_AXIS_VALUE = 65535
CONTROLLER_QUIT_BUTTON_CODE = 158
CONTROLLER_SCAN_START_BUTTON_CODE = 308
CONTROLLER_SCAN_STOP_BUTTON_CODE = 307
CONTROLLER_JOYSTICK_EVENT = 3
CONTROLLER_BUTTON_EVENT = 1


class XboxController:
    def __init__(self):
        self.device = None
        self.cmd_publisher = None
        self.scan_publisher = None
        self.scan_active = False
        self.horizontal_axis_value = int(MAX_AXIS_VALUE / 2)
        self.vertical_axis_value = int(MAX_AXIS_VALUE / 2)
        self.last_message_time = 0
        self.events_queue = SimpleQueue()
    
    def publish_cmd_message(self):
        twist = Twist()

        twist.linear.x = numpy.interp(self.vertical_axis_value, [0, MAX_AXIS_VALUE], [1.0, -1.0])
        twist.angular.z = numpy.interp(self.horizontal_axis_value, [0, MAX_AXIS_VALUE], [-1.0, 1.0])
        
        if abs(twist.linear.x) < 0.1:
            twist.linear.x = 0.0
        
        if abs(twist.angular.z) < 0.1:
            twist.angular.z = 0.0

        self.cmd_publisher.publish(twist)
    
    def publish_scan_message(self):
        msg = Bool()

        msg.data = self.scan_active

        self.scan_publisher.publish(msg)

    def parse_event(self, event):
        if event.type != CONTROLLER_JOYSTICK_EVENT:
            return
        
        if event.code == 1: # Vertical Axis
            self.vertical_axis_value = event.value
        elif event.code == 0: # Horizontal Axis
            self.horizontal_axis_value = event.value

    def watch_events(self):
        for event in self.device.read_loop():
            if event.type in [CONTROLLER_BUTTON_EVENT, CONTROLLER_JOYSTICK_EVENT]:
                if event.type == CONTROLLER_BUTTON_EVENT and event.code == CONTROLLER_QUIT_BUTTON_CODE and event.value == 1:
                    break
                elif rospy.is_shutdown():
                    break
                
                self.events_queue.put(event)

    def main(self):
        rospy.init_node('xbox_controller')
        
        self.cmd_publisher = rospy.Publisher("~cmd_vel", Twist, queue_size=10)
        self.scan_publisher = rospy.Publisher("~scan_active", Bool, queue_size=10)
        
        try:
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            
            self.device = devices[0]
        except IndexError:
            print("Xbox controller not found")
            return
        
        worker = Thread(target=self.watch_events)
        worker.start()
        
        while not rospy.is_shutdown():
            while not self.events_queue.empty():
                event = self.events_queue.get_nowait()
                
                if event.type == CONTROLLER_BUTTON_EVENT and event.value == 1:
                    if event.code == CONTROLLER_SCAN_START_BUTTON_CODE:
                        self.scan_active = True
                    elif event.code == CONTROLLER_SCAN_STOP_BUTTON_CODE:
                        self.scan_active = False
                
                self.parse_event(event)
                
            now = time.time_ns() // 1_000_000
    
            if now - self.last_message_time >= 100: # 10Hz
                self.last_message_time = now
                
                self.publish_cmd_message()
                self.publish_scan_message()

if __name__ == '__main__':
    try:
        node = XboxController()
        node.main()
    except rospy.ROSInterruptException:
        pass

