from setuptools import find_packages, setup
from glob import glob

package_name = 'map_path_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
               
        ('share/' + package_name, [
            'map_path_plan/flat_and_hills_whole_OGM(0.5)_with_meta.npz',
            'map_path_plan/interpolated_path_3d_flat_and_hills_whole.csv',
            'map_path_plan/flat_and_hills_pcd(0.5).ply',
        ]),
    ],
    install_requires=['setuptools', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='strho',
    maintainer_email='fun3545@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ogm_loader   = map_path_plan.ogm_loader:main',
        #'path_loader     = map_path_plan.path_loader:main',
        'ply_loader    = map_path_plan.ply_loader:main',

        ],
    },
)
