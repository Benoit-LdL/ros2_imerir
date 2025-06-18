"""
Node stub to reach a single Nav2 goal triggered from Python code.
That code must be properly copied into a ROS package and an entry point declared for main().
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient


class GoToNode(Node):
    def __init__(self):
        super().__init__('go_to_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the navigate_to_pose action server...')

    def send_goal(self, posToGo):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = posToGo[0]
        goal_pose.pose.position.y = posToGo[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = posToGo[2]
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: ' + str(result))

positionsToGo = [[1.11,-0.272,0.0],     # pos 1
                 [0.261,3.22,0.0],      # pos 2
                 [-1.59,3.76,0.0],      # pos 3
                 [0.544,5.71,0.0],      # pos 4
                 [2.18,3.6,0.0],        # pos 5
                 [3.02,1.3,0.0]]        # pos 6

def main():
    rclpy.init()
    node = GoToNode()
    for pos in positionsToGo :
        node.send_goal(pos)
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()