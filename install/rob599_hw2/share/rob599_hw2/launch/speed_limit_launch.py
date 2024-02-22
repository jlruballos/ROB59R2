import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # VelocityLimiter node
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='VelocityLimiter',
            name='VelocityLimiter',
            remappings=[
               
            ],

            parameters=[
                {'linear_max': 25.0},
                {'angular_max': 20.0},
                {'with_watchdog': True},
                {'watchdog_period': 5},
            ],
        ),

        # The Publish Twist node.
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='PublishTwist',
            name='PublishTwist',
            remappings=[
            
            ],
            parameters=[
               
            ],
            ),

        # The Twist Checker node.
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='TwistChecker',
            name='TwistChecker',
            remappings=[
            
            ],

            parameters=[
                {'linear_x_max': 10.0},
                {'angular_z_max': 15.0},
            ],

            ),

        ])