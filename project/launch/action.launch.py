from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project',
            executable='action_node',
            name='action_node_project',
            output='screen'),
        TimerAction(
            period=2.0,  # Delay in seconds
            actions=[
                Node(
                    package='project',
                    executable='action_client_node',
                    name='action_client_node',
                    output='screen'
                )
            ]
        ),
    ])
