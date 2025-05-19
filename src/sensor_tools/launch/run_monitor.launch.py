from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 기존 SensorBridge 노드 수정된 executable 이름 사용
        Node(
            package='tank_package',
            executable='sensor_bridge_node',
            name='sensor_bridge',
            output='screen',
            parameters=[{
                'server_ip': '192.168.0.22',
                'poll_hz':   0.3,
                'timeout':   2.0,
            }],
        ),

        # 센서 모니터 노드
        Node(
            package='sensor_tools',
            executable='sensor_monitor',
            name='sensor_monitor',
            output='screen',
            emulate_tty=True,
        ),

    ])