#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

 #Types for the service call.  We only need the base type, which corresponds to the
# Doubler.srv interface definition file.
from rob599_msgs.srv import ApplyBrakes

class VelocityLimiter(Node):
    def __init__(self):
        super().__init__('velocity_limiter')

        # Declare parameters for maximum linear and angular velocities with default values
        self.declare_parameter('linear_max', 1.0)
        self.declare_parameter('angular_max', 1.0)

        #declare parameter for watchdog
        self.declare_parameter('with_watchdog', True)
        self.declare_parameter('watchdog_period',2) #in seconds

        # Get the parameter values
        self.max_linear_speed = self.get_parameter('linear_max').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('angular_max').get_parameter_value().double_value
        self.with_watchdog = self.get_parameter('with_watchdog').get_parameter_value().bool_value
        self.watchdog_period = self.get_parameter('watchdog_period').get_parameter_value().integer_value

        # Subscriber to the 'speed_in' topic
        self.subscription = self.create_subscription(Twist, 'speed_in', self.velocity_callback, 10)

        # Publisher to the 'speed_out' topic
        self.publisher = self.create_publisher(Twist, 'speed_out', 10)

         # Initialize the last received message time
        self.last_msg_time = self.get_clock().now()
        # Initialize the watchdog timer
        self.watchdog_timer = self.create_timer(self.watchdog_period / 2, self.watchdog_callback)

        # Add a service server for applying brakes
        self.brake_service = self.create_service(ApplyBrakes, 'apply_brakes', self.apply_brakes_callback)

        # Attribute to track whether brakes are applied
        self.brakes_applied = False

        # Timer for publishing zero velocity when brakes are applied
        self.brake_timer = None

    def apply_brakes_callback(self, request, response):
        self.brakes_applied = request.brakes

        operation_successful = False  # Initialize a flag to track the success of the operation

        if self.brakes_applied:
            # If brakes are applied, start a timer to publish zero velocity at 10Hz
            if not self.brake_timer:
                self.brake_timer = self.create_timer(0.1, self.publish_zero_velocity)
                operation_successful = True  # Consider the operation successful if the timer is started
        else:
            # If brakes are released, stop the timer and resume normal operation
            if self.brake_timer:
                self.brake_timer.cancel()
                self.brake_timer = None
                operation_successful = True  # Consider the operation successful if the timer is stopped

        # Log the brake status and the outcome of the operation
        self.get_logger().info(f'Brakes applied: {self.brakes_applied}, Operation successful: {operation_successful}')

        # Set the success field in the response
        response.success = operation_successful

        return response

    def publish_zero_velocity(self):
        zero_msg = Twist()
         # Limit linear velocity
        zero_msg.linear.x = 0.0

        # Limit angular velocity
        zero_msg.angular.z = 0.0

        # Publish the limited velocities
        self.publisher.publish(zero_msg)

        # Log the limited velocities
        self.get_logger().info(f'Published zero velocities: Linear = {zero_msg.linear.x:.2f}, Angular = {zero_msg.angular.z:.2f}')

    def velocity_callback(self, msg):

        if not self.brakes_applied:
            # Update the last received message time
            self.last_msg_time = self.get_clock().now()

            # Create a new Twist message for output
            out_msg = Twist()

            # Limit linear velocity
            out_msg.linear.x = max(min(msg.linear.x, self.max_linear_speed), -self.max_linear_speed)

            # Limit angular velocity
            out_msg.angular.z = max(min(msg.angular.z, self.max_angular_speed), -self.max_angular_speed)

            # Publish the limited velocities
            self.publisher.publish(out_msg)

            # Log the limited velocities
            self.get_logger().info(f'Published limited velocities: Linear = {out_msg.linear.x:.2f}, Angular = {out_msg.angular.z:.2f}')

    def watchdog_callback(self):
            # Calculate the elapsed time since the last received message
        elapsed_time = self.get_clock().now() - self.last_msg_time
        elapsed_seconds = elapsed_time.nanoseconds / 1e9  # Convert nanoseconds to seconds and add to whole seconds

        # Check if watchdog is enabled and the elapsed time exceeds the watchdog period
        if self.with_watchdog and elapsed_seconds > self.watchdog_period:
            # Publish a zero-velocity Twist message
            zero_msg = Twist()
            self.publisher.publish(zero_msg)
            # Log the action
            self.get_logger().info('Watchdog triggered: Published zero velocity')

# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
    # Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
    rclpy.init(args=args)

    velocity_limiter = VelocityLimiter()

    # The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
    rclpy.spin(velocity_limiter)

   # Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node. # Shutdown ROS 2 upon node termination
    rclpy.shutdown()

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
    # The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
    main()
