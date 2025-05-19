from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['ultralytics', 'opencv-python', 'rclpy', 'cv_bridge'],
    zip_safe=True,
    maintainer='strho',
    maintainer_email='fun3545@gmail.com',
    description='YOLOv8 detector as separate ROS2 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = yolo_detector_pkg.yolo_detector_node:main',
        ],
    },
)
