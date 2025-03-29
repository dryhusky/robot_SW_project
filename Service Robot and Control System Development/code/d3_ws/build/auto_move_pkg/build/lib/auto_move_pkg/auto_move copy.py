import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import math


class TableNavigator(Node):
    def __init__(self):
        super().__init__('table_navigator')

        # 테이블 위치 매핑
        self.table_waypoints = {
            1: (2.5882797241210938, 1.6086732149124146, 0.0),  # 테이블 1
            2: (2.558715581893921, 0.5037817358970642, 0.0),  # 테이블 2
            3: (2.5818209648132324, -0.6096141934394836, 0.0),  # 테이블 3
        }

        # 초기 위치 설정
        self.initial_position = (-1.3453491476411727e-07, -0.0156390480697155, 0.0)

        # 액션 클라이언트 생성
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 명령 구독
        self.subscription = self.create_subscription(
            String,
            '/robot/command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        try:
            table_id = int(msg.data)
            if table_id in self.table_waypoints:
                self.navigate_to_table(table_id)
            else:
                self.get_logger().warn(f"Invalid table ID: {table_id}")
        except ValueError:
            self.get_logger().error(f"Invalid command received: {msg.data}")

    def navigate_to_table(self, table_id):
        x, y, theta = self.table_waypoints[table_id]

        # 목표 테이블로 이동
        self.get_logger().info(f"Navigating to Table {table_id} at x={x}, y={y}, theta={theta}")
        if self.navigate_to_position(x, y, theta):
            self.get_logger().info(f"Successfully reached Table {table_id}.")
        else:
            self.get_logger().warn(f"Failed to reach Table {table_id}. Cannot attempt to return to initial position.")
            return  # 테이블 이동 실패 시 초기 위치로 복귀 시도하지 않음

        # 초기 위치로 복귀
        self.get_logger().info(f"Attempting to return to initial position: {self.initial_position}")
        if self.navigate_to_position(*self.initial_position):
            self.get_logger().info("Successfully returned to initial position.")
        else:
            self.get_logger().warn("Failed to return to initial position. Check the initial position or Action Server.")

    def navigate_to_position(self, x, y, theta):
        self.get_logger().info(f"Sending goal to position: x={x}, y={y}, theta={theta}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, theta)

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available.')
            return False

        self.get_logger().info("Action server is available. Sending goal...")

        try:
            send_goal_future = self.action_client.send_goal_async(goal_msg)
            self.get_logger().info("Goal sent successfully. Waiting for server to accept...")
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error("Goal rejected by the Action Server.")
                return False

            self.get_logger().info("Goal accepted by the Action Server. Waiting for result...")

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result()

            if result:
                self.get_logger().info(f"Goal result received. Status: {result.status}")
            else:
                self.get_logger().warn("Goal result is None.")

            if result and result.status == 4:
                self.get_logger().info("Successfully reached the goal position.")
                return True
            else:
                self.get_logger().warn(f"Failed to reach the goal position. Status: {result.status if result else 'Unknown'}")
                return False

        except Exception as e:
            self.get_logger().error(f"An exception occurred while sending goal: {e}")
            return False

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = TableNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

