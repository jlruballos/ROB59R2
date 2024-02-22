#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')

        # Create a publisher, specifying the Twist message type and topic name
        self.publisher = self.create_publisher(Twist, 'speed_in', 10)

        # Set up a timer to call the publish_twist method at a regular interval
        self.timer = self.create_timer(1, self.publish_twist)

        # Initialize variables to vary the published velocities
        self.linear_velocity = 0.5  # Starting linear velocity
        self.angular_velocity = 0.5  # Starting angular velocity

    def publish_twist(self):
        # Create a new Twist message
        msg = Twist()

        # Set the linear and angular velocities
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity

        # Publish the Twist message
        self.publisher.publish(msg)

        # Log the published message for debugging purposes
        self.get_logger().info(f'Publishing Twist: Linear Velocity = {self.linear_velocity}, Angular Velocity = {self.angular_velocity}')

        # Update the velocities for the next publication
        self.linear_velocity += 0.1
        self.angular_velocity += 0.1

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the TwistPublisher node
    twist_publisher = TwistPublisher()

    # Run the node until it's shut down
    rclpy.spin(twist_publisher)

    # Cleanly shut down the ROS 2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
