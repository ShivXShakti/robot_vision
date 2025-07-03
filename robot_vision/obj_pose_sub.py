import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class CSVDictSubscriber(Node):
    def __init__(self):
        super().__init__('csv_dict_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/current_tasks',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Parse JSON string into dictionary
        row_dict = json.loads(msg.data)
        self.get_logger().info(f"Received dict: {row_dict}")

        # Assign each row to its own variable
        row1 = row_dict.get("task1", [])
        row2 = row_dict.get("task2", [])
        row3 = row_dict.get("task3", [])
        row4 = row_dict.get("task4", [])

        # Example: unpack row1 elements to individual variables
        if len(row1) >= 6:
            a1, a2, a3, a4, a5, a6 = row1
            self.get_logger().info(f"raw row:{row1},  Row1 unpacked: {a1}, {a2}, {a3}, {a4}, {a5}, {a6}")
        else:
            self.get_logger().warn("Row1 doesn't have enough elements.")

        # You can also unpack other rows similarly if needed

def main(args=None):
    rclpy.init(args=args)
    node = CSVDictSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
