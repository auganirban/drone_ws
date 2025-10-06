#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class RotorSpinner(Node):
    def __init__(self):
        super().__init__('rotor_spinner')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 Hz
        self.angle = 0.0

        # Define the joint names
        self.joint_names = ["rotor_0_joint", "rotor_0_dummy_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"]

    def timer_callback(self):
        self.angle += 10.0  # increase for faster spinning
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [self.angle, self.angle, -self.angle, self.angle, -self.angle]
        self.publisher_.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = RotorSpinner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
