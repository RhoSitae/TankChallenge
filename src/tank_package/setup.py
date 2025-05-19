# tank_package/setup.py
from setuptools import setup, find_packages

package_name = 'tank_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['tank_package/launch/tank_system.launch.py']),
        ('share/' + package_name + '/params',
            ['tank_package/params/bridge.yaml']),
    ],
    install_requires=['setuptools', 'requests', 'numpy'],
    zip_safe=True,
    maintainer='strho',
    maintainer_email='fun3545@gmail.com',
    description='ROS 2 bridge & control nodes for Unity tank simulator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = tank_package.sensor.bridge_node:main',
            'rviz_publisher = tank_package.visualization.rviz_publisher:main',
            'ffmpeg_camera = tank_package.sensor.ffmpeg_camera_node:main',
        ],
    },
    include_package_data=True,
)
