"""Development profile: competition + optional rosbag2 recording and GCS nodes (design §3.1, §10)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Shared with the dev recorder; no dependency from bringup to dev-only code.
import json
RECORD_TOPICS = json.load(open(os.path.join(
    get_package_share_directory('line_follower_bringup'), 'config', 'record_topics.json')))



def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('line_follower_bringup'), 'launch')
    record = LaunchConfiguration('record')
    gcs = LaunchConfiguration('gcs')
    obstacles = LaunchConfiguration('obstacles')

    ui = Node(package='line_follower_gcs', executable='gcs_mission_manager',
              parameters=[{'use_sim_time': True}], condition=IfCondition(gcs), output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('compose', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('stop_on_gcs_exit', default_value='false'),
        DeclareLaunchArgument('recorder', default_value='true'),
        DeclareLaunchArgument('bag_root', default_value='/ws/bags'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('gcs', default_value='false'),
        DeclareLaunchArgument('obstacles', default_value='false',
                              description='Extension: obstacle_detector (never in competition)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
            launch_arguments={'headless': LaunchConfiguration('headless'),
                              'camera_bridge': PythonExpression(["'", LaunchConfiguration('compose'), "'.lower() != 'true'"])}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nodes.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'camera_bridge': 'true',
                              'compose': LaunchConfiguration('compose'), 'obstacles': obstacles, 'auto_start': LaunchConfiguration('auto_start'),
                              'gcs_snapshot_enabled': PythonExpression(["'", gcs, "' == 'true' or '", record, "' == 'true'"])}.items()),
        Node(package='line_follower_gcs', executable='gcs_recorder',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('recorder'),
                 "' == 'true' and ('", gcs, "' == 'true' or '", record, "' == 'true')"])),
             parameters=[{'use_sim_time': True, 'output_root': LaunchConfiguration('bag_root'),
                          'record_on_start': ParameterValue(record, value_type=bool)}], output='screen'),
        Node(package='line_follower_gcs', executable='gcs_bridge',
             parameters=[{'use_sim_time': True}], condition=IfCondition(gcs), output='screen'),
        Node(package='line_follower_gcs', executable='gcs_command',
             parameters=[{'use_sim_time': True}], condition=IfCondition(gcs), output='screen'),
        ui,
        RegisterEventHandler(OnProcessExit(target_action=ui,
            on_exit=[Shutdown(reason='GCS window closed')]),
            condition=IfCondition(LaunchConfiguration('stop_on_gcs_exit'))),
    ])
