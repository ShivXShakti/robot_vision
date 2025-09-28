import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_ros

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')
        self.sub = self.create_subscription(PoseStamped, '/object_pose_in_camera', self.pose_callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/object_pose_in_base', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',        # target frame
                msg.header.frame_id, # source frame (camera frame)
                rclpy.time.Time())
            
            # transform pose
            from tf2_geometry_msgs import do_transform_pose
            pose_transformed = do_transform_pose(msg, transform)
            
            self.pub.publish(pose_transformed)

        except Exception as e:
            self.get_logger().warn(f'Could not transform pose: {e}')

def main():
    rclpy.init()
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
