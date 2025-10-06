#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class DroneMover(Node):
    def __init__(self):
        super().__init__('drone_mover')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.angle = 0.0
        self.radius = 1.0  # circle radius

    def timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'      # world or odom frame
        t.child_frame_id = 'base_link' # your drone's root link

        # Make drone move in a circle
        t.transform.translation.x = self.radius * math.cos(self.angle)
        t.transform.translation.y = self.radius * math.sin(self.angle)
        t.transform.translation.z = 0.5  # fixed height
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)
        self.angle += 0.05  # speed of motion

def main(args=None):
    rclpy.init(args=args)
    node = DroneMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
