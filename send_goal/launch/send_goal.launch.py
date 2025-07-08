from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='originbot_send_goal',
            executable='send_goal_node',
            name='send_goal_node',
            output='screen'
        )
    ])

