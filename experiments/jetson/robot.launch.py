"""Real-robot launch for the Python line_follower node (no Gazebo, no ros_gz).

Installed into the line_follower package share by
experiments/jetson/Dockerfile.jazzy-l4t, so on the Orin the command is:

    ros2 launch line_follower robot.launch.py

The camera driver is expected to publish sensor_msgs/Image on /camera/image_raw.
The base driver is expected to consume /cmd_vel and publish /odom and /imu.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw',
                              description='sensor_msgs/Image from the CSI-2 or USB camera'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('perception', default_value='rule',
                              description='rule (implemented) or dl (stub until a model is wired)'),
        DeclareLaunchArgument('controller', default_value='rule'),
        DeclareLaunchArgument('restore_model',
                              default_value='/ws/experiments/jetson/models/V1-Restore128.onnx',
                              description='ONNX line-restoration model; empty string disables it'),
        Node(
            package='line_follower',
            executable='follower_node',
            name='follower_node',
            output='screen',
            parameters=[{
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
