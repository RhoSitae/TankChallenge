from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'tank_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # tank_control 포함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('tank_control/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='strho',
    maintainer_email='fun3545@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speed_control_node = tank_control.speed_control_node:main',
            'steering_control_node = tank_control.steering_control_node:main',
            'command_node = tank_control.command_node:main',
            'turret_yaw_control_node = tank_control.turret_yaw_control_node:main',
            'turret_pitch_control_node = tank_control.turret_pitch_control_node:main',
        ],
    },
)
