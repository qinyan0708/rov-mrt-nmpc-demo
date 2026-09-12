import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'rov_mrt_sim'
    )

    rviz_config = os.path.join(
        package_share,
        'rviz',
        'rov_mrt_demo.rviz',
    )
    parameter_file = os.path.join(
        package_share,
        'config',
        'opposite_side.yaml',
    )
    use_rviz = LaunchConfiguration('use_rviz')

    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'map',
        ],
        output='screen',
    )

    visualization = Node(
        package='rov_mrt_sim',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
        parameters=[parameter_file],
    )

    reference_path = Node(
        package='rov_mrt_sim',
        executable='path_publisher_node',
        name='path_publisher_node',
        output='screen',
        parameters=[parameter_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # Start NMPC immediately so solver construction overlaps RViz startup.
    nmpc_avoidance_controller = Node(
        package='rov_mrt_sim',
        executable='nmpc_avoidance_controller_node',
        name='nmpc_avoidance_controller_node',
        output='screen',
        parameters=[parameter_file],
    )

    vehicle = Node(
        package='rov_mrt_sim',
        executable='vehicle_sim_node',
        name='vehicle_sim_node',
        output='screen',
        parameters=[parameter_file],
    )

    obstacles = Node(
        package='rov_mrt_sim',
        executable='obstacle_node',
        name='obstacle_node',
        output='screen',
        parameters=[parameter_file],
    )

    # Delay plant and obstacles until the NMPC solver is constructed.
    delayed_simulation = TimerAction(
        period=6.0,
        actions=[
            vehicle,
            obstacles,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the bundled configuration.',
        ),
        world_to_map,
        visualization,
        reference_path,
        rviz,
        nmpc_avoidance_controller,
        delayed_simulation,
    ])
