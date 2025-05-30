from launch import LaunchDescription
from launch_ros.actions import Node
poll_hz = 25
server_ip = '192.168.0.200'
def generate_launch_description():
    return LaunchDescription([

        Node(package='tank_package',
             executable='bridge_node',
             name='bridge_node',
             output='screen',
             ),

        Node(package='tank_control',
             executable='speed_control_node',
             name='speed_control_node',
             output='screen',
             parameters=[{
                'server_ip': server_ip,
                'poll_hz': poll_hz,
                'timeout': 2.0,
            }],
             ),

        Node(package='tank_control',
             executable='steering_control_node',
             name='steering_control_node',
             output='screen',
             parameters=[{
                'server_ip': server_ip,
                'poll_hz': poll_hz,
                'timeout': 2.0,
            }],
             ),

        Node(
            package='tank_control',
            executable='turret_yaw_control_node',
            name='turret_yaw_control_node',
            output='screen',
            parameters=[{
                'server_ip': server_ip,
                'poll_hz': poll_hz,
                'timeout': 2.0,
            }],
        ),

        Node(
            package='tank_control',
            executable='turret_pitch_control_node',
            name='turret_pitch_control_node',
            output='screen',
            parameters=[{
                'server_ip': server_ip,
                'poll_hz': poll_hz,
                'timeout': 2.0,
            }],
        ),

        Node(
            package='tank_control',
            executable='command_node',
            name='command_node',
            output='screen',
            parameters=[{
                'server_ip': server_ip,
                'poll_hz': poll_hz,
                'timeout': 2.0,
            }],
        ),



    ])
