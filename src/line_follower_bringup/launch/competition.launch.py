"""Official one-line entry point (R-03):

    ros2 launch line_follower_bringup competition.launch.py

Gazebo + bridges + application nodes. No GCS, rosbag, Wi-Fi or Jetson dependencies.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('line_follower_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('compose', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('line_follower_bringup'), 'config', 'competition.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={'headless': LaunchConfiguration('headless'),
                              'camera_bridge': PythonExpression(["'", LaunchConfiguration('compose'), "'.lower() != 'true'"])}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nodes.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'camera_bridge': 'true', 'obstacles': 'false',
                              'compose': LaunchConfiguration('compose'),
                              'params_file': LaunchConfiguration('params_file')}.items()),
    ])
