import threading
import rclpy

from math import sin, cos, pi
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):
    def __init__(self):
        rclpy.init()

        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        joint_state = JointState()
        
        wturn = 0.0

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                now = self.get_clock().now()

                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['drive_wheel_l_joint', 'drive_wheel_r_joint', 'front_wheel_l_joint', 'front_wheel_r_joint']
                joint_state.position = [wturn, wturn, wturn, wturn]

                self.joint_pub.publish(joint_state)
                
                wturn += 0.1

                loop_rate.sleep()

        except KeyboardInterrupt:
          pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
