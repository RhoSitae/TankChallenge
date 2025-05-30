from launch import LaunchDescription
from launch_ros.actions import Node
from tank_package.common.param_file import param_file
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

pkg_share = Path(get_package_share_directory('tank_package'))
server_ip = '192.168.0.200'
poll_hz = 25
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tank_package',
            executable='bridge_node',
            name='bridge_node',
            parameters=[{
                'server_ip': server_ip,
                'poll_hz': poll_hz,
                'timeout': 2.0,
            }],
            output="screen",
        ),
        Node(
            package='tank_package', 
            executable='ffmpeg_camera',
            name='ffmpeg_camera',
            output="screen",
        ),
        Node(
            package='tank_package',
            executable='rviz_publisher',       # 실행 스크립트 이름
            name='rviz_publisher',
            output='screen',
            parameters=[{
                'server_ip': server_ip,
                'poll_hz': poll_hz,
                'timeout': 2.0,
            }],
        ),

])
