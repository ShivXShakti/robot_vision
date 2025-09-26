#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dualarm_custom_msgs.msg import PoseEuler 
from depthai_ros_msgs.msg import TrackDetection2DArray

class PoseEulerPublisher(Node):
    def __init__(self):
        super().__init__('pose_euler_publisher')
        self.do_not_modify()
        self.get_logger().info("PoseEuler Publisher Started...")
        
    def do_not_modify(self):
        self.publisher_ = self.create_publisher(PoseEuler, 'pose_euler', 10)
        self.create_subscription(TrackDetection2DArray, '/color/yolo_Spatial_tracklets',
            self.listener_callback, 10)
    
    def listener_callback(self, msg):
        for detection in msg.detections:
            if not detection.results:
                continue

            result = detection.results[0]
            class_id = int(result.hypothesis.class_id)
            score = result.hypothesis.score
            pos = result.pose.pose.position
            
            bbox = detection.bbox
            cx, cy = bbox.center.position.x, bbox.center.position.y
            w, h = bbox.size_x, bbox.size_y
            # Tracking info
            tid = detection.tracking_id
            age = detection.tracking_age
            status = detection.tracking_status

            pose = PoseEuler()
            pose.x = pos.x
            pose.y = pos.y
            pose.z = pos.z
            pose.alpha = 0.0
            pose.beta  = 0.0
            pose.gamma = 0.0

            self.publisher_.publish(pose)
            self.get_logger().info(
                f"Publishing: x={pose.x:.2f}, y={pose.y:.2f}, z={pose.z:.2f}, "
                f"α={pose.alpha:.2f}, β={pose.beta:.2f}, γ={pose.gamma:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = PoseEulerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()