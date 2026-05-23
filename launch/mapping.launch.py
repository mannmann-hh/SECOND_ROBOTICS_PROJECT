#!/usr/bin/env python3
# mapping.launch.py
# 第二个机器人项目 (Task 1: 建图)
#
# 启动:
#   1. pointcloud_to_laserscan —— 把 /ugv/rslidar_points 转成 /scan
#   2. slam_toolbox            —— 用 /scan + tf 建图,输出 /map
#   3. rviz                    —— 可视化 map / 激光 / tf
#
# 用法:
#   终端1: ros2 bag play ~/bags/<bag名> --clock --loop
#   终端2: ros2 launch second_project mapping.launch.py
#
# 注意:所有路径都用相对包路径(get_package_share_directory),不用绝对路径!

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('second_project')

    # 配置文件路径(随包安装,相对路径)
    pc2ls_config = os.path.join(pkg_dir, 'config', 'pointcloud_to_laserscan.yaml')
    slam_config  = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    rviz_config  = os.path.join(pkg_dir, 'config', 'mapping.rviz')

    # 用 bag/仿真时间。bag play 加了 --clock,这里必须 true
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='用 bag/仿真时间(bag play --clock 时为 true)'
        ),

        # ---- 1. 点云转激光 ----
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[pc2ls_config, {'use_sim_time': use_sim_time}],
            remappings=[
                # 输入:bag 里的点云话题
                ('cloud_in', '/ugv/rslidar_points'),
                # 输出:slam_toolbox 订阅的激光话题
                ('scan', '/scan'),
            ],
        ),

        # ---- 2. slam_toolbox 建图 ----
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': use_sim_time}],
        ),

        # ---- 3. rviz 可视化 ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
