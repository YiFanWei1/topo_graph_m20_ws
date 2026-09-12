from setuptools import find_packages, setup

package_name = 'route3d_vertex_initializer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vertex_initializer.yaml']),
        ('share/' + package_name + '/launch', ['launch/vertex_initializer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Route3D Maintainer',
    maintainer_email='maintainer@example.com',
    description='Publish /initialpose from a selected Route3D topology vertex.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vertex_initializer_node = route3d_vertex_initializer.vertex_initializer_node:main',
            'set_initial_pose_by_vertex = route3d_vertex_initializer.send_vertex_id:main',
        ],
    },
)
