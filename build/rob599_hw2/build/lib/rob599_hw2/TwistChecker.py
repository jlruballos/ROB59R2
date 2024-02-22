#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistChecker(Node):
    def __init__(self):
        super().__init__('twist_checker')

        # Declare parameters for the limits
        self.declare_parameter('linear_x_max', 1.0)
        self.declare_parameter('angular_z_max', 1.0)

        # Retrieve the parameter values
        self.linear_x_max = self.get_parameter('linear_x_max').get_parameter_value().double_value
        self.angular_z_max = self.get_parameter('angular_z_max').get_parameter_value().double_value

        # Initialize counters
        self.total_received = 0
        self.total_out_of_bounds = 0

        # Create a subscriber for Twist messages on the 'twist_topic' topic
        self.subscription = self.create_subscription(Twist, 'speed_in', self.twist_callback, 10)

        # Set up a timer to log information every 30 seconds
        self.log_timer = self.create_timer(30, self.log_info)

    def twist_callback(self, msg):
        # Increment the total received messages counter
        self.total_received += 1

        # Check if the message values are within the specified bounds
        if abs(msg.linear.x) > self.linear_x_max or abs(msg.angular.z) > self.angular_z_max:
            self.total_out_of_bounds += 1

    def log_info(self):
        # Calculate the proportion of out-of-bounds messages
        if self.total_received > 0:
            proportion_out_of_bounds = self.total_out_of_bounds / self.total_received
        else:
            proportion_out_of_bounds = 0

        # Log the information
        self.get_logger().info(
            f'In the last 30 seconds, received {self.total_received} messages, '
            f'with {self.total_out_of_bounds} ({proportion_out_of_bounds:.2%}) out of bounds.'
        )

        # Reset counters
        self.total_received = 0
        self.total_out_of_bounds = 0

def main(args=None):
    rclpy.init(args=args)

    twist_checker = TwistChecker()

    rclpy.spin(twist_checker)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
