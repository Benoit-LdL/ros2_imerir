import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
import time

class GoToNode(Node):
    def __init__(self):
        super().__init__('go_to_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for the navigate_to_pose action server...')

        # Publisher for initial pose
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,'initialpose',20)

    def set_initial_pose(self, initial_position):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Position
        msg.pose.pose.position.x = initial_position[0]
        msg.pose.pose.position.y = initial_position[1]
        msg.pose.pose.position.z = 0.0

        # Orientation (simplified to yaw only; assumes quaternion z + w)
        msg.pose.pose.orientation.z = initial_position[2]
        msg.pose.pose.orientation.w = 1.0

        # Covariance (example values for AMCL)
        msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.25, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0685389
        ]

        self.get_logger().info('Publishing initial pose...')
        self.initial_pose_publisher.publish(msg)

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



positionsToGo = [[-4.22, -0.87, -1.2],     # pos 1
                 [-4.22, 3.0, 1.2],        # pos 2
                 [-0.60, 2.4, 0.0],         # pos 3
                 [6.87, 2.4, 0.0],          # pos 4
                 [8.72, -0.5, 0.0],         # pos 5    
                 [3.30, 3.70, 0.0]]         # pos 6

def main():
    rclpy.init()
    
    node = GoToNode()

    # position robot on map origin
    initial_pose = [0.00, 1.45, 0.00]  # x, y, yaw
    node.set_initial_pose(initial_pose)
    time.sleep(8)  # Let AMCL process the initial pose

    for i in range(len(positionsToGo)) :
        node.get_logger().info(f'POS {i} --> {positionsToGo[i]}')
        node.send_goal(positionsToGo[i])
        node.destroy_node()
        node.get_logger().info('waiting 1 sec...')
        time.sleep(1)
    
    rclpy.shutdown()
