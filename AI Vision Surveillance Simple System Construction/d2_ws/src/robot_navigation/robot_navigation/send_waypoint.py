import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
import math
import threading
import sys
import select
import termios
import tty

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self):
        # 세 개의 웨이포인트 정의
        waypoints = []

        # 첫 번째 웨이포인트
        waypoint1 = PoseStamped()
        waypoint1.header.stamp.sec = 0
        waypoint1.header.stamp.nanosec = 0
        waypoint1.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint1.pose.position.x = 0.35624730587005615
        waypoint1.pose.position.y = -0.7531262636184692
        waypoint1.pose.position.z = 0.0

        waypoint1_yaw = 0.0  # Target orientation in radians
        waypoint1.pose.orientation = self.euler_to_quaternion(0, 0, waypoint1_yaw)
        
        # waypoint1.pose.orientation.x = 0.0
        # waypoint1.pose.orientation.y = 0.0
        # waypoint1.pose.orientation.z = -0.9999865408184966
        # waypoint1.pose.orientation.w = 0.005188273494832019
        waypoints.append(waypoint1)

        # 두 번째 웨이포인트
        waypoint2 = PoseStamped()
        waypoint2.header.stamp.sec = 1736835180

        waypoint2.header.stamp.nanosec = 351218566

        waypoint2.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint2.pose.position.x = 0.4317740797996521

        waypoint2.pose.position.y = 0.021892864257097244
        waypoint2.pose.position.z = 0.002471923828125
        
        waypoint2_yaw = 0.0  # Target orientation in radians
        waypoint2.pose.orientation = self.euler_to_quaternion(0, 0, waypoint2_yaw)
        
        # waypoint2.pose.orientation.x = 0.0
        # waypoint2.pose.orientation.y = 0.0
        # waypoint2.pose.orientation.z = -0.9999330665398213
        # waypoint2.pose.orientation.w = 0.01156989370173046
        waypoints.append(waypoint2)

        # 세 번째 웨이포인트
        waypoint3 = PoseStamped()
        waypoint3.header.stamp.sec = 1736835180
        waypoint3.header.stamp.nanosec = 351218566
        waypoint3.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint3.pose.position.x = 0.4317740797996521
        waypoint3.pose.position.y = 0.021892864257097244
        waypoint3.pose.position.z = 0.002471923828125
        
        waypoint3_yaw = 0.0  # Target orientation in radians
        waypoint3.pose.orientation = self.euler_to_quaternion(0, 0, waypoint3_yaw)
                
        # waypoint3.pose.orientation.x = 0.0
        # waypoint3.pose.orientation.y = 0.0
        # waypoint3.pose.orientation.z = -0.6938991006274311
        # waypoint3.pose.orientation.w = 0.7200722450896453
        waypoints.append(waypoint3)
        
        # 네 번째 웨이포인트
        waypoint4 = PoseStamped()
        waypoint4.header.stamp.sec = 1736835529
        waypoint4.header.stamp.nanosec = 145196822
        waypoint4.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint4.pose.position.x = -0.5882161855697632
        waypoint4.pose.position.y = -0.6036583781242371
        waypoint4.pose.position.z = -0.001434326171875
        
        waypoint4_yaw = 0.0  # Target orientation in radians
        waypoint4.pose.orientation = self.euler_to_quaternion(0, 0, waypoint4_yaw)
                
        # waypoint4.pose.orientation.x = 0.0
        # waypoint4.pose.orientation.y = 0.0
        # waypoint4.pose.orientation.z = -0.6938991006274311
        # waypoint4.pose.orientation.w = 0.7200722450896453
        waypoints.append(waypoint4)
        
        # 다섯 번째 웨이포인트
        waypoint5 = PoseStamped()
        waypoint5.header.stamp.sec = 1736835751
        waypoint5.header.stamp.nanosec = 75242492
        waypoint5.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint5.pose.position.x = -1.476792335510254
        waypoint5.pose.position.y = -0.009377971291542053
        waypoint5.pose.position.z = -0.001434326171875
        
        waypoint5_yaw = 0.0  # Target orientation in radians
        waypoint5.pose.orientation = self.euler_to_quaternion(0, 0, waypoint5_yaw)
                
        # waypoint5.pose.orientation.x = 0.0
        # waypoint5.pose.orientation.y = 0.0
        # waypoint5.pose.orientation.z = -0.6938991006274311
        # waypoint5.pose.orientation.w = 0.7200722450896453
        waypoints.append(waypoint5)
        
        
        # 여섯 번째 웨이포인트
        waypoint6 = PoseStamped()
        waypoint6.header.stamp.sec = 1736835953
        waypoint6.header.stamp.nanosec = 389229378
        waypoint6.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint6.pose.position.x = -1.5331106185913086
        waypoint6.pose.position.y = 0.0031334750819951296
        waypoint6.pose.position.z = -0.001434326171875
        
        waypoint6_yaw = 0.0  # Target orientation in radians
        waypoint6.pose.orientation = self.euler_to_quaternion(0, 0, waypoint6_yaw)
                
        # waypoint6.pose.orientation.x = 0.0
        # waypoint6.pose.orientation.y = 0.0
        # waypoint6.pose.orientation.z = -0.6938991006274311
        # waypoint6.pose.orientation.w = 0.7200722450896453
        waypoints.append(waypoint6)
        
        
         # 일곱 번째 웨이포인트
        waypoint7 = PoseStamped()
        waypoint7.header.stamp.sec = 1736835963
        waypoint7.header.stamp.nanosec = 506091140
        waypoint7.header.frame_id = "map"  # 프레임 ID를 설정 (예: "map")
        waypoint7.pose.position.x = -1.5331130027770996
        waypoint7.pose.position.y = -0.6411882042884827
        waypoint7.pose.position.z = -0.001434326171875
        
        waypoint7_yaw = 0.0  # Target orientation in radians
        waypoint7.pose.orientation = self.euler_to_quaternion(0, 0, waypoint7_yaw)
                
        # waypoint7.pose.orientation.x = 0.0
        # waypoint7.pose.orientation.y = 0.0
        # waypoint7.pose.orientation.z = -0.6938991006274311
        # waypoint7.pose.orientation.w = 0.7200722450896453
        waypoints.append(waypoint7)
        
        
        
        # FollowWaypoints 액션 목표 생성 및 전송
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        # 서버 연결 대기
        self.action_client.wait_for_server()

        # 목표 전송 및 피드백 콜백 설정
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Attempting to cancel the goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    # def cancel_done_callback(self, future):
    #     cancel_response = future.result()
    #     if cancel_response.accepted:
    #         self.get_logger().info('Goal cancellation accepted. Exiting program...')
    #         self.destroy_node()
    #         rclpy.shutdown()
    #         sys.exit(0)  # Exit the program after successful cancellation
    #     else:
    #         self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_cancelled) > 0:
            self.get_logger().info('Goal cancellation accepted. Exiting program...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)  # Exit the program after successful cancellation
        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'g':
                    node.get_logger().info('Key "g" pressed. Sending goal...')
                    node.send_goal()
                elif key.lower() == 's':
                    node.get_logger().info('Key "s" pressed. Cancelling goal...')
                    node.cancel_goal()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()
    
    rclpy.spin(node)


if __name__ == '__main__':
    main()
