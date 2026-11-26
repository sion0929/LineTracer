"""Real-robot launch for the Python line_follower node (no Gazebo, no ros_gz).

Installed into the line_follower package share by
experiments/jetson/Dockerfile.jazzy-l4t, so on the Orin the command is:

    ros2 launch line_follower robot.launch.py

The camera driver is expected to publish sensor_msgs/Image on /camera/image_raw.
The base driver is expected to consume /cmd_vel and publish /odom and /imu.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Same perception/IMU settings as the evaluated V0-NearRule640 (rule.yaml minus sim time and
    # the track-specific speed_profile). Command-line args below override the file.
    params_file = os.path.join(get_package_share_directory('line_follower'), 'config', 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw',
                              description='sensor_msgs/Image from the CSI-2 or USB camera'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('perception', default_value='rule',
                              description='rule | depth | hybrid (see docs/VERSIONS.md)'),
        DeclareLaunchArgument('controller', default_value='rule'),
        DeclareLaunchArgument('restore_model', default_value='',
                              description='ONNX line-restoration model; empty (default) = off'),
        Node(
            package='line_follower',
            executable='follower_node',
            name='follower_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': False,
                'perception': LaunchConfiguration('perception'),
                'controller': LaunchConfiguration('controller'),
                'restore_model': LaunchConfiguration('restore_model'),
            }],
            remappings=[
                ('/camera/image_raw', LaunchConfiguration('camera_topic')),
                ('/odom', LaunchConfiguration('odom_topic')),
                ('/imu', LaunchConfiguration('imu_topic')),
                ('/cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ],
        ),
    ])
