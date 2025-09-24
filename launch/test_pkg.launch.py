from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_pkg',
            executable='turtle_move',
            name='turtle_move'
        ),
        Node(
            package='test_pkg',
            executable='gampad_node',
            name='gampad_node'
        )
        ])


    