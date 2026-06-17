#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')
    nav2_dir = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(pkg_dir, 'world', 'scout.world')
    map_file = os.path.join(pkg_dir, 'map', 'my_map_2.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_file = os.path.join(pkg_dir, 'config', 'navigation.rviz')
    goals_file = os.path.join(pkg_dir, 'csv', 'goals.csv')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    stage_bridge_package = LaunchConfiguration('stage_bridge_package')
    stage_bridge_executable = LaunchConfiguration('stage_bridge_executable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from Stage.'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate the Nav2 lifecycle nodes.'
        ),
        DeclareLaunchArgument(
            'stage_bridge_package',
            default_value='second_project',
            description='Package that provides the Stage ROS 2 bridge executable.'
        ),
        DeclareLaunchArgument(
            'stage_bridge_executable',
            default_value='stageros',
            description='Stage ROS 2 bridge executable.'
        ),

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package=stage_bridge_package,
            executable=stage_bridge_executable,
            name='stageros',
            output='screen',
            arguments=[world_file],
            cwd=os.path.join(pkg_dir, 'world'),
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_model_names': False,
                'delay_odom_tf_by_one_update': True,
                'base_watchdog_timeout': 0.5,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
            }.items(),
        ),

        Node(
            package='second_project',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'csv_path': goals_file,
                'goal_frame': 'map',
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
