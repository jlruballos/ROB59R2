import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rob599_msgs.action import LaunchRocket
from action_msgs.msg import GoalStatus 

class LaunchRocketClient(Node):
    def __init__(self):
        super().__init__('launch_client')
        self.client = ActionClient(self, LaunchRocket, 'launch_rocket')
        self.goal_handle = None  # Store the goal handle

    def send_goal(self, t):
        goal_msg = LaunchRocket.Goal()
        goal_msg.time = t
        self.client.wait_for_server()
        self.result = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.result.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()  # Store the goal handle
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Launch progress: {feedback.progress} seconds')

    def get_result_callback(self, future):
        result = future.result().result
        launch_status = result.launch 

        if future.result().status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'Launch {launch_status}')
        elif future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Launch {launch_status}')  # Print out the actual launch status
        else:
            self.get_logger().info('Launch did not complete successfully')

    def cancel_goal(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        else:
            self.get_logger().info('No active launch to cancel.')

def main(args=None):
    rclpy.init(args=args)
    client = LaunchRocketClient()

    import sys
    if len(sys.argv) > 1:
        countdown_time = int(sys.argv[1])
    else:
        countdown_time = 10  # Default to 10 seconds if no argument provided
    if len(sys.argv) > 2:
        abort_time = int(sys.argv[2])
    else:
        abort_time = 999999  # Default to 999999 seconds if no argument provided

    
    # Send a goal with a specific time
    client.send_goal(countdown_time)

    # Create a timer to request cancellation
    timer = client.create_timer(abort_time, client.cancel_goal)

    # Spin to process incoming messages and callbacks
    rclpy.spin(client)

    # Cleanup and shutdown
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
