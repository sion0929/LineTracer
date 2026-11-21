"""Dev autonomous sections: production graph, manual mission start after placement."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share=get_package_share_directory('line_follower_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch/sim.launch.py')),launch_arguments={'headless':'true','camera_bridge':'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch/nodes.launch.py')),launch_arguments={'camera_bridge':'true','auto_start':'false','startup_delay_sec':'1.0'}.items())])
