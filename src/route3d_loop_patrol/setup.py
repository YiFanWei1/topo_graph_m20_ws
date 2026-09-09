from glob import glob
from setuptools import find_packages, setup


package_name = 'route3d_loop_patrol'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='wei',
    maintainer_email='wei@example.com',
    description='Arrival-gated loop patrol coordinator for Route3D.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'loop_patrol_node = route3d_loop_patrol.loop_patrol_node:main',
        ],
    },
)
