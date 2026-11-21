"""Replay profile: re-run the pipeline on recorded sensor topics (design §10.3).

The final command is redirected to /replay/cmd_vel_blocked so a replay can never drive a robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('line_follower_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Path to a rosbag2 recording'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nodes.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'camera_bridge': 'false',
                'obstacles': 'false',
                'startup_delay_sec': '0.0',
                'cmd_vel_topic': '/replay/cmd_vel_blocked',
            }.items()),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--clock',
                 '--rate', LaunchConfiguration('rate'),
                 '--topics', '/camera/image_raw', '/imu', '/odom'],
            output='screen'),
    ])
