#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class XboxController:
    def __init__(self):
        self.publisher = None
    
    def joy_callback(data):
        twist = Twist()

        twist.linear.x = 4 * data.axes[1]
        twist.angular.z = 4 * data.axes[0]

        self.publisher.publish(twist)

    def start():
        self.publisher = rospy.Publisher("~cmd_vel", Twist)

        rospy.Subscriber("joy", Joy, callback)

        rospy.init_node('xbox_controller')
        rospy.spin()

if __name__ == '__main__':
    try:
        node = XboxController()
        node.main()
    except rospy.ROSInterruptException:
        pass
