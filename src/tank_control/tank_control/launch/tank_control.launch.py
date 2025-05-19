from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='tank_package',
             executable='bridge_node',
             name='bridge_node',
             output='screen'),

        Node(package='tank_control',
             executable='control_node',
             name='control_node',
             output='screen'),
    ])
