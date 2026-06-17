#!/usr/bin/env python3
# mapping.launch.py
# Second Robotics Project (Task 1: Mapping)

#
# Launch:
#   1. pointcloud_to_laserscan —— convert /ugv/rslidar_points to /scan
#   2. slam_toolbox            —— use /scan + tf to build map, output /map
#   3. rviz                    —— visualize map / laser / tf
#
# Usage:
#   Terminal 1: ros2 launch second_project mapping.launch.py
#   Terminal 2: ros2 bag play ~/bags/<bagname> --clock --loop
#
# Note: All paths use relative package paths (get_package_share_directory), not absolute paths!

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_slam_node(context, *args, **kwargs):
    slam_mode = LaunchConfiguration('slam_mode').perform(context).lower()
    if slam_mode == 'sync':
        executable = 'sync_slam_toolbox_node'
    elif slam_mode == 'async':
        executable = 'async_slam_toolbox_node'
    else:
        raise RuntimeError("slam_mode must be either 'sync' or 'async'")

    return [
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mode': 'mapping',
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                },
            ],
        )
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')

    # Configuration file paths (installed with the package, relative paths)
    pc2ls_config = os.path.join(pkg_dir, 'config', 'pointcloud_to_laserscan.yaml')
    slam_config  = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    rviz_config  = os.path.join(pkg_dir, 'config', 'mapping.rviz')

    # Use bag/simulation time. The --clock flag is added to ros2 bag play, so this must be true
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use bag/simulation time (true when using ros2 bag play --clock)'
        ),
        DeclareLaunchArgument(
            'slam_mode',
            default_value='async',
            description='slam_toolbox executable: async or sync'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic generated from the RSLidar point cloud'
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='UGV_odom',
            description='Odometry frame from the bag'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame published by slam_toolbox'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='UGV_base_link',
            description='Robot base frame from the bag'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_config,
            description='slam_toolbox parameter file'
        ),

        # ---- 1. Pointcloud to Laserscan ----
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[pc2ls_config, {'use_sim_time': use_sim_time}],
            remappings=[
                # Input point cloud topic from the bag.
                ('cloud_in', '/ugv/rslidar_points'),
                # Output LaserScan topic subscribed by slam_toolbox.
                ('scan', scan_topic),
            ],
        ),

        # ---- 2. slam_toolbox mapping ----
        OpaqueFunction(function=make_slam_node),

        # ---- 3. RViz visualization ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
