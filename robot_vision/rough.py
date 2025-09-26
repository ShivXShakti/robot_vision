#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import TrackDetection2DArray

# COCO class labels
COCO_CLASSES = [
    "person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat","traffic light",
    "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
    "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant",
    "bed","diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave",
    "oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
]

class YoloTrackSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_track_subscriber')
        self.subscription = self.create_subscription(
            TrackDetection2DArray,
            '/color/yolov4_Spatial_tracklets',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        for detection in msg.detections:
            if not detection.results:
                continue

            result = detection.results[0]
            class_id = int(result.hypothesis.class_id)
            score = result.hypothesis.score
            class_name = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else str(class_id)

            # 3D pose
            pos = result.pose.pose.position
            x, y, z = pos.x, pos.y, pos.z

            # 2D bbox
            bbox = detection.bbox
            cx, cy = bbox.center.position.x, bbox.center.position.y
            w, h = bbox.size_x, bbox.size_y

            # Tracking info
            tid = detection.tracking_id
            age = detection.tracking_age
            status = detection.tracking_status

            self.get_logger().info(
                f"[ID={tid}] {class_name} (score={score:.2f}) "
                f"3D(x={x:.2f}, y={y:.2f}, z={z:.2f}) "
                f"BBox(cx={cx:.1f}, cy={cy:.1f}, w={w:.1f}, h={h:.1f}) "
                f"Age={age}, Status={status}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
