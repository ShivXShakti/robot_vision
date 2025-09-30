#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
#from tf_transformations import quaternion_matrix
from scipy.spatial.transform import Rotation as R
import numpy as np

class TFMatrixNode(Node):
    def __init__(self):
        super().__init__('tf_matrix_node')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to check transforms periodically
        self.timer = self.create_timer(1.0, self.lookup_transform)

    def lookup_transform(self):
        try:
            # Get transform from base_link to camera_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',        # target frame
                'camera_link',      # source frame
                rclpy.time.Time())  # latest available

            # Extract quaternion (w, x, y, z)
            q = [
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z
            ]

            # Extract translation (x, y, z)
            t = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]

            # Build 4x4 matrix
            r = R.from_quat(q) 
            rot_matrix = r.as_matrix()
            #T = quaternion_matrix(q)
            T = np.eye(4)
            T[:3, :3] = rot_matrix
            T[0:3, 3] = t

            self.get_logger().info(f"\n{np.round(T, 4)}")

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def main():
    rclpy.init()
    node = TFMatrixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
