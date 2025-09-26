#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dualarm_custom_msgs.msg import PoseEuler   # Import custom message
import math
import random

class PoseEulerPublisher(Node):
    def __init__(self):
        super().__init__('pose_euler_publisher')
        self.publisher_ = self.create_publisher(PoseEuler, 'pose_euler', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("PoseEuler Publisher Started...")

    def timer_callback(self):
        msg = PoseEuler()

        # Example: publishing random values (replace with real data later)
        msg.x = random.uniform(-1.0, 1.0)
        msg.y = random.uniform(-1.0, 1.0)
        msg.z = random.uniform(-1.0, 1.0)

        # Euler angles in radians
        msg.alpha = random.uniform(-math.pi, math.pi)
        msg.beta  = random.uniform(-math.pi/2, math.pi/2)
        msg.gamma = random.uniform(-math.pi, math.pi)

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Publishing: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, "
            f"α={msg.alpha:.2f}, β={msg.beta:.2f}, γ={msg.gamma:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseEulerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
