from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project',
            executable='services_node',
            name='services_node_project',
            output='screen'),
    ])
