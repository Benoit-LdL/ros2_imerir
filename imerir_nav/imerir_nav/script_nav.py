import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time

class GoToNode(Node):
    def __init__(self):
        super().__init__('go_to_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the navigate_to_pose action server...')

    def send_goal(self, posToGo):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = posToGo[0]
        goal_pose.pose.position.y = posToGo[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = posToGo[2]
        goal_pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending goal...')

        # Send goal asynchronously
        goal_future = self.action_client.send_goal_async(goal_msg)

        # Wait for the future to complete
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get result asynchronously, then wait for it
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info('Navigation result: ' + str(result))



positionsToGo = [[-4.22, -0.87, 0.00],      # pos 1
                 [-4.22, 3.0, 0.0],         # pos 2
                 [-0.60, 2.4, 0.0],        # pos 3
                 [6.87, 2.4, 0.0],         # pos 4
                 [8.72, -0.5, 0.0],        # pos 5    
                 [3.30, 3.70, 0.0]]         # pos 6

def main():
    rclpy.init()
    
    node = GoToNode()

    for i in range(len(positionsToGo)) :
        node.get_logger().info(f'POS {i} --> {positionsToGo[i]}')
        node.send_goal(positionsToGo[i])
        node.destroy_node()
        node.get_logger().info('waiting 1 sec...')
        time.sleep(1)
    
    rclpy.shutdown()
