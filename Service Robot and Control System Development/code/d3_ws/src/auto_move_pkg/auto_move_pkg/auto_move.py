import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import math
from action_msgs.msg import GoalStatus


class TableNavigator(Node):
    def __init__(self):
        super().__init__('table_navigator')

        # 테이블 위치 매핑
        self.table_waypoints = {
            1: (2.5882797241210938, 1.6086732149124146, 0.0),  # 테이블 1
            2: (2.558715581893921, 0.5037817358970642, 0.0),  # 테이블 2
            3: (2.5818209648132324, -0.6096141934394836, 0.0),  # 테이블 3
            4: (1.5268861055374146, 1.6209053993225098, 0.0),          # 테이블 4
            5: (1.4883310794830322, 0.5598081350326538, 0.0),          # 테이블 5
            6: (1.450302004814148, -0.6064997315406799, 0.0),          # 테이블 6
            7: (0.45698440074920654, 1.5804883241653442, 0.0),          # 테이블 7
            8: (0.4273757338523865, 0.48436465859413147, 0.0),          # 테이블 8
            9: (0.4590336084365845, -0.5851494073867798, 0.0),          # 테이블 9
        }

        # 초기 위치 설정
        self.initial_position = (-1.3453491476411727e-07, -0.0156390480697155, 0.0)

        # 상태 플래그
        self.returning_to_init = False
        self.goal_completed = False

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
        if self.returning_to_init:
            self.get_logger().info("Currently returning to initial position. Ignoring new commands.")
            return

        try:
            table_id = int(msg.data)
            if table_id in self.table_waypoints:
                self.get_logger().info(f"Command received for Table {table_id}")
                self.goal_completed = False  # 새로운 명령 시 초기화
                self.navigate_to_table(table_id)
            else:
                self.get_logger().warn(f"Invalid table ID: {table_id}")
        except ValueError:
            self.get_logger().error(f"Invalid command received: {msg.data}")

    def navigate_to_table(self, table_id):
        if self.returning_to_init:
            self.get_logger().info("Currently returning to initial position. Ignoring new commands.")
            return

        x, y, theta = self.table_waypoints[table_id]

        # 목표 테이블로 이동
        self.get_logger().info(f"Navigating to Table {table_id} at x={x}, y={y}, theta={theta}")
        self.navigate_to_position(x, y, theta)

    def navigate_to_init(self):
        if self.returning_to_init:
            self.get_logger().info("Already returning to initial position. Ignoring additional calls.")
            return

        self.returning_to_init = True  # 플래그 설정
        self.get_logger().info(f"Attempting to return to initial position: {self.initial_position}")
        self.navigate_to_position(*self.initial_position)

    def navigate_to_position(self, x, y, theta):
        self.get_logger().info(f"Sending goal to position: x={x}, y={y}, theta={theta}")

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, theta)

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.get_action_feedback)
        self.send_goal_future.add_done_callback(self.get_action_goal)
        self.get_logger().info("Action server is available. Sending goal...")

    def get_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Action goal rejected.')
            return
        self.get_logger().info('Action goal accepted.')
        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.get_action_result)

    def get_action_feedback(self, feedback_msg):
        pass

    def get_action_result(self, future):
        if self.goal_completed:
            self.get_logger().info("Goal already completed. Ignoring duplicate result.")
            return

        action_status = future.result().status
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action succeeded!')
            if not self.returning_to_init:
                self.get_logger().info("Successfully reached the goal position. Returning to initial position...")
                self.navigate_to_init()
            else:
                self.get_logger().info("Successfully returned to initial position.")
                self.returning_to_init = False  # 복귀 완료 후 플래그 해제
                self.goal_completed = True  # 결과 처리 완료
        else:
            self.get_logger().warn(f"Failed to reach the goal position. Status: {action_status if action_status else 'Unknown'}")

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

