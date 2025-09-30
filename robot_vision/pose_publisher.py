#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import time
from depthai_ros_msgs.msg import SpatialDetectionArray  # adjust if different


class ObjectPoseTransformer(Node):
    def __init__(self):
        super().__init__('object_pose_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            SpatialDetectionArray,
            '/color/yolo_Spatial_tracklets',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, 'object_pose_torso', 10)
        self.get_logger().info("ObjectPoseTransformer initialized")

    def listener_callback(self, msg):
        if not msg.detections:
            return
        detection = msg.detections[0]
        if not detection.results:
            return
        hypothesis = detection.results[0].hypothesis
        
        pose = hypothesis.pose.pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rclpy.time.Time().to_msg()
        pose_stamped.header.frame_id = msg.header.frame_id
        pose_stamped.pose = pose
        try:
            #msg.header.stamp = rclpy.time.Time().to_msg()

            transformed_pose = self.tf_buffer.transform(
                pose_stamped,
                'torso',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            transformed_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(transformed_pose)

            p = transformed_pose.pose.position
            self.get_logger().info(
                f"Pose wrt torso: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}"
            )

        except Exception as e:
            self.get_logger().warn(f"Could not transform pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
