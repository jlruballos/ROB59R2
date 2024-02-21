#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
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
            callback_group=ReentrantCallbackGroup(),
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
                # Return a result indicating the launch was aborted
                return LaunchRocket.Result(launch=False)

            feedback_msg.progress = i
            self.get_logger().info(f'Countdown: {i} seconds')
            goal_handle.publish_feedback(feedback_msg)
            #  delay
            sleep(1)

        # If the loop completes without cancellation, the rocket is launched
        goal_handle.succeed()
        self.get_logger().info('Rocket has launched!')
        # Return a result indicating the rocket has successfully launched
        return LaunchRocket.Result(launch=True)

def main(args=None):
    rclpy.init(args=args)
    launch_rocket_action_server = NASAActionServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(launch_rocket_action_server, executor=executor)

    launch_rocket_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
