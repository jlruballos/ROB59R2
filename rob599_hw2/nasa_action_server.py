import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from time import sleep

from rob599_msgs.action import LaunchRocket

class NASAActionServer(Node):
    def __init__(self):
        super().__init__('nasa_launch_controller')
        self._action_server = ActionServer(
            self,
            LaunchRocket,
            'launch_rocket',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received launch request for countdown {goal_request.time}')
        # Accept all goals
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Launch aborted!')
        return rclpy.action.CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Countdown initiated...')
        feedback_msg = LaunchRocket.Feedback()

        # Start the countdown
        for i in range(goal_handle.request.time, 0, -1):
            # Check if there's a cancellation request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Rocket launch aborted.')
                return LaunchRocket.Result(launch='Aborted')  # Return indicating launch aborted

            feedback_msg.progress = i
            self.get_logger().info(f'Countdown: {i} seconds')
            goal_handle.publish_feedback(feedback_msg)
            sleep(1)  # Delay to simulate countdown

        goal_handle.succeed()
        self.get_logger().info('Rocket has launched!')
        return LaunchRocket.Result(launch='Success')  # Return indicating successful launch

def main(args=None):
    rclpy.init(args=args)
    launch_rocket_action_server = NASAActionServer()

    launch_rocket_action_server.get_logger().info('NASA Action Server has been started.')

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(launch_rocket_action_server, executor=executor)

    launch_rocket_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()