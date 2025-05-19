from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 병렬로 실행할 필요 없이 tank_package 그대로 실행된 후,
        # 별도 터미널에서 아래 명령으로 YOLO 노드만 띄울 수 있습니다.
        Node(
            package='yolo_detector_pkg',
            executable='yolo_detector_node',
            name='yolo_detector',
            output='screen',
            parameters=[{'model_path':'/home/strho/tank_ros2_ws/src/yolo_detector_pkg/best.pt'}]
        ),
    ])