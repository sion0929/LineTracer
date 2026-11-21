"""Dev-only moving mapper check; no rover motion controller or safety publisher."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('line_follower_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__),'d05_capture.launch.py'))),
        Node(package='line_follower_mission',executable='mission_manager',parameters=[
            os.path.join(share,'config/competition.yaml'),{'use_sim_time':True,'auto_start':False,
            'startup_delay_sec':20.,'gcs_snapshot_enabled':True}],output='screen')])
