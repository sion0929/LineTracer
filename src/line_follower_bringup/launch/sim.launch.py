"""Gazebo Harmonic with the UNMODIFIED provided world + ROS-Gazebo bridges (design §3.2 steps 1-3)."""
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _gazebo(context):
    share = get_package_share_directory('line_follower_bringup')
    world = os.path.join(share, 'worlds', 'line_track.sdf')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    # gz_sim.launch.py runs `gz sim` through a shell: quote the path (it may contain spaces).
    gz_args = f"-r {'-s ' if headless else ''}{shlex.quote(world)}"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    share = get_package_share_directory('line_follower_bringup')
    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('camera_bridge', default_value='true',
                              description='External image bridge for sim-only/baseline runs'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo server only (no GUI)'),
        # Provided models are located through the resource path; their content is never changed.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        OpaqueFunction(function=_gazebo),
        # Rover topics only: /clock, /imu, /odom, /camera/camera_info, /cmd_vel (R-07).
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'ros_gz_bridge.yaml')},
                         use_sim_time],
             output='screen'),
        Node(package='ros_gz_image', executable='image_bridge', name='camera_bridge',
             condition=IfCondition(LaunchConfiguration('camera_bridge')),
             arguments=['/camera/image_raw'],
             parameters=[use_sim_time],
             output='screen'),
    ])
