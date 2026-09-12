from glob import glob
from setuptools import find_packages, setup

package_name = 'rov_mrt_sim'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
        (
            'share/' + package_name + '/rviz',
            glob('rviz/*.rviz'),
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qyan',
    maintainer_email='qyan@users.noreply.github.com',
    description='ROS 2 simulation for the ROV-MRT mining vehicle.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vehicle_sim_node = rov_mrt_sim.vehicle_sim_node:main',
            'visualization_node = rov_mrt_sim.visualization_node:main',
            'obstacle_node = rov_mrt_sim.obstacle_node:main',
            'path_publisher_node = rov_mrt_sim.path_publisher_node:main',
            'path_tracking_controller_node = '
            'rov_mrt_sim.path_tracking_controller_node:main',
            'nmpc_controller_node = rov_mrt_sim.nmpc_controller_node:main',
            'nmpc_avoidance_controller_node = '
            'rov_mrt_sim.nmpc_avoidance_controller_node:main',
        ],
    },
)
