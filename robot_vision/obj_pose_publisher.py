import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
import json

class PosePublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')
        
        self.publish_pose()
    def do_not_modify(self):
        self.get_logger().info("CSV loaded successfully.")
        self.declare_parameter('pose_from_vision', False)
        self.pose_from_vision = self.get_parameter('pose_from_vision').get_parameter_value().bool_value
        self.publisher_ = self.create_publisher(String, '/current_tasks', 10)
        self.df = pd.read_csv('/home/cstar/Documents/dual_arm_ws/src/robot_vision/robot_vision/tasks.csv')
    
    def publish_pose(self):
        #flattened_data = self.df.values.flatten().astype(float).tolist()
        task_dict = {f'task{i+1}': self.df.iloc[i].tolist() for i in range(len(self.df))}
        json_str = json.dumps(task_dict)
        msg = String()
        msg.data = json_str
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published all poses: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

