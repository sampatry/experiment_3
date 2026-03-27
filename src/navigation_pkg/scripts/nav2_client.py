import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Nav2Client(Node):
  
    # ROS 2 Node that sends navigation goals to Nav2

    def __init__(self):
        # Initialize the ROS 2 node with the name of nav2_client
        super().__init__('nav2_client')

        # Create an ActionClient for the 'navigate_to_pose' action
        self._client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def go_to(self, pose: PoseStamped) -> bool:

        # Send a navigation goal to Nav2
        # Wait for Nav2 action server to become available
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False

        # Create navigation goal message
        goal = NavigateToPose.Goal()

        # Set target pose
        goal.pose = pose

        # Log goal position (x, y)
        self.get_logger().info(
            f'Sending goal: x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}'
        )

        # Send goal asynchronously
        send_future = self._client.send_goal_async(goal)

        # Wait until goal is accepted or rejected
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        # Check if goal was accepted by Nav2
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        self.get_logger().info('Goal accepted, navigating...')

        # Wait for navigation result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # Get final navigation status
        status = result_future.result().status

        # Evaluate navigation result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            return True

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted')
            return False

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled')
            return False

        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            return False
