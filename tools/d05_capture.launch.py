"""Dev-only stationary perception capture: Gazebo and the production image components."""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share=get_package_share_directory('line_follower_bringup')
    components=[ComposableNode(package='ros_gz_bridge',plugin='ros_gz_bridge::RosGzBridge',
        name='camera_bridge',parameters=[{'config_file':os.path.join(share,'config/camera_bridge.yaml'),
                                        'use_sim_time':True}],extra_arguments=[{'use_intra_process_comms':True}])]
    for package,plugin,name in [('line_follower_perception','LineDetectorNode','line_detector'),
                                ('line_follower_navigation','PathPlannerNode','path_planner')]:
        components.append(ComposableNode(package=package,plugin=package+'::'+plugin,name=name,
            parameters=[os.path.join(share,'config/competition.yaml'),{'use_sim_time':True,'imu_yaw_enabled':os.environ.get('D07_IMU_YAW','true')=='true'}],
            extra_arguments=[{'use_intra_process_comms':True}]))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch/sim.launch.py')),
                                 launch_arguments={'headless':'true','camera_bridge':'false'}.items()),
        ComposableNodeContainer(additional_env={'LD_PRELOAD':' '.join(filter(None,[
            os.path.join(get_package_prefix('ros_gz_bridge'),'lib','libros_gz_bridge.so'),
            os.environ.get('LD_PRELOAD','')]))},package='rclcpp_components',executable='component_container_mt',
            name='perception_container',namespace='',parameters=[{'thread_num':4}],
            composable_node_descriptions=components,output='screen')])
