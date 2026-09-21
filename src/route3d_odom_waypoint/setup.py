from glob import glob

from setuptools import find_packages, setup


package_name = 'route3d_odom_waypoint'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='wei',
    maintainer_email='wei@example.com',
    description='Build Route3D topology from odometry without point clouds.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_waypoint_node = route3d_odom_waypoint.odom_waypoint_node:main',
            'pose_file_to_topology = route3d_odom_waypoint.pose_file_to_topology:main',
            'visualize_file_result = route3d_odom_waypoint.visualize_file_result:main',
        ],
    },
)
