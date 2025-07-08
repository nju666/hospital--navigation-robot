import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class CruiseNode(Node):
    def __init__(self):
        super().__init__('cruise_node')

        self.goals = [
            (0.5, 0.0),
            (2.5, 0.0),
        ]
        self.index = 0
        self.direction = 1  # 1=正序，-1=反序

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('waiting...')
        self.action_client.wait_for_server()

        # 启动巡航
        self.send_next_goal()

    def send_next_goal(self):
        x, y = self.goals[self.index]
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        self.get_logger().info(f'reach_goal: {self.index} ({x:.2f}, {y:.2f})')

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal_refused')
            return

        self.get_logger().info('goal_accepted_waiting')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'{dist:.2f}m_distance')

    def result_callback(self, future):
        status = future.result().status

        if status == 4:  # ABORTED
            self.get_logger().warn('abolish,trying_next_goal')
        elif status == 3:  # SUCCEEDED
            self.get_logger().info('arrived')
        elif status == 5:  # CANCELED
            self.get_logger().info('Goal was canceled externally.')
            return  # 不再发布下一个目标，等待节点终止
        
        # 更新下一个目标索引（反序循环）
        self.index += self.direction
        if self.index >= len(self.goals):
            self.index = len(self.goals) - 2
            self.direction = -1
        elif self.index < 0:
            self.index = 1
            self.direction = 1

        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = CruiseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
