from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project',
            executable='client_node',
            name='client_node_project',
            output='screen'),
    ])
