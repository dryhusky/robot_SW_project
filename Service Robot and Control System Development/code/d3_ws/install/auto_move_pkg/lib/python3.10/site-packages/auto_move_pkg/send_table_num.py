import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class KitchenNode(Node):
    def __init__(self):
        super().__init__('kitchen_node')
        # Publisher to send table number
        self.table_publisher = self.create_publisher(Int32, 'table_goal', 10)
        
        # Timer to periodically check for input
        self.timer = self.create_timer(1.0, self.trigger_publish_table_number)  # 1 second interval
    
    
    def trigger_publish_table_number(self):
        """Simulate service call with input"""
        table_number = input("Enter table number to send (or 'q' to quit): ").strip()
        if table_number.lower() == 'q':
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()
            return

        try:
            table_number = int(table_number)
            msg = Int32()
            msg.data = table_number
            self.table_publisher.publish(msg)
            self.get_logger().info(f"Published table number: {table_number}")
        except ValueError:
            self.get_logger().warn("Invalid table number entered. Please enter a valid integer.")


def main(args=None):
    rclpy.init(args=args)
    kitchen_node = KitchenNode()
    rclpy.spin(kitchen_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()