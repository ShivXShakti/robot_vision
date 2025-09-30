import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

# Import your YOLO spatial tracklets message type
from depthai_ros_msgs.msg import SpatialDetectionArray  # adjust if different

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer')
        # Subscribe to YOLO Spatial Tracklets
        self.sub = self.create_subscription(
            SpatialDetectionArray,
            '/color/yolo_Spatial_tracklets',
            self.pose_callback,
            10
        )
        self.pub = self.create_publisher(PoseStamped, '/object_pose_in_base', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose_callback(self, msg: SpatialDetectionArray):
        if not msg.detections:
            return  # no detections, skip

        # Take first detection (you can loop for multiple objects)
        detection = msg.detections[0]
        if not detection.results:
            return

        hypothesis = detection.results[0].hypothesis
        pose = hypothesis.pose.pose

        # Build PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = msg.header.frame_id
        pose_stamped.pose = pose

        try:
            # Lookup transform from camera frame to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',          # target frame
                pose_stamped.header.frame_id,  # source frame (camera)
                rclpy.time.Time()
            )

            # Apply transform
            pose_transformed = do_transform_pose(pose_stamped, transform)
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
