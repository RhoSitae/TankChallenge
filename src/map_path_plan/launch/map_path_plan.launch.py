from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    share_dir = get_package_share_directory('map_path_plan')
    npz_file = str(Path(share_dir) / 'flat_and_hills_whole_OGM(0.5)_with_meta.npz')
    ply_file = str(Path(share_dir) / 'flat_and_hills_pcd(0.5).ply')

    return LaunchDescription([
        
        Node(
            package='map_path_plan',
            executable='ply_loader',
            name='ply_loader',
            parameters=[{'ply_path': ply_file}],
            output='screen',
        ),

        Node(
            package='map_path_plan',
            executable='ogm_loader',
            name='ogm_loader',
            output='screen',
            parameters=[{
                'npz_path': npz_file,
            }],
        ),


    ])
