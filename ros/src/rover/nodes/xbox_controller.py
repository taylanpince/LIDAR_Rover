import rospy
import sys
import time
import evdev
import numpy

from std_msgs.msg import Int32


JOYSTICK_MAX = 65535
MOTOR_MAX_SPEED = 255


class XboxController:
    def __init__(self):
        self.current_left_power = 0.0
        self.current_right_power = 0.0
        self.lwheel_publisher = None
        self.rwheel_publisher = None

    def publish_value(self, value, publisher, current_power):
        target_power = numpy.interp(value, [0, JOYSTICK_MAX], [1.0, -1.0])
        
        if target_power > 0.2 and current_power != target_power:
            publisher.publish(int(target_power * MOTOR_MAX_SPEED))
        elif target_power < -0.2 and current_power != target_power:
            publisher.publish(int(target_power * MOTOR_MAX_SPEED))
        elif -0.2 <= target_power <= 0.2 and current_power != 0.0:
            publisher.publish(0)
            target_power = 0.0
        
        return target_power

    def parse_event(self, event):
        if event.type != 3:
            return
        
        if event.code == 1:
            self.current_left_power = self.publish_value(
                event.value, 
                self.lwheel_publisher, 
                self.current_left_power
            )
        elif event.code == 5:
            self.current_right_power = self.publish_value(
                event.value, 
                self.rwheel_publisher, 
                self.current_right_power
            )

    def main(self):
        rospy.init_node('xbox_controller')
        
        self.lwheel_publisher = rospy.Publisher('~lwheel_speed', Int32, queue_size=10)
        self.rwheel_publisher = rospy.Publisher('~rwheel_speed', Int32, queue_size=10)
        
        try:
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            device = devices[0]
        except IndexError:
            print("Xbox controller not found")
            return
        
        for event in device.read_loop():
            if event.code == 158 and event.type == 1 and event.value == 1:
                print("Xbox controller quitting...")
                break
            
            self.parse_event(event)
            
            if rospy.is_shutdown():
                break

if __name__ == '__main__':
    try:
        node = XboxController()
        node.main()
    except rospy.ROSInterruptException:
        pass

