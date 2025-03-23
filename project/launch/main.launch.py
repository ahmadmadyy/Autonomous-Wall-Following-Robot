from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project',
            executable='services_node',
            name='services_node',
            output='screen'),
        TimerAction(
            period=2.0,  # Delay in seconds
            actions=[
                Node(
                    package='project',
                    executable='client_node',
                    name='client_node',
                    output='screen'
                )
            ]
        ),
    ])
