from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='originbot_cruising',
            executable='cruise_node',
            name='cruise_node',
            output='screen'
        )
    ])
