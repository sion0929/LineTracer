"""Five logical rover nodes; image/perception/planning share a multithreaded container by default."""
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def _perception(context):
    share = get_package_share_directory('line_follower_bringup')
    params = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    common = [params, {'use_sim_time': use_sim_time}]
    compose = LaunchConfiguration('compose').perform(context).lower() == 'true'
    obstacles = LaunchConfiguration('obstacles').perform(context).lower() == 'true'
    camera = LaunchConfiguration('camera_bridge').perform(context).lower() == 'true'
    plugins = [('line_follower_perception', 'line_follower_perception::LineDetectorNode', 'line_detector'),
               ('line_follower_navigation', 'line_follower_navigation::PathPlannerNode', 'path_planner')]
    if obstacles:
        plugins.append(('line_follower_perception', 'line_follower_perception::ObstacleDetectorNode', 'obstacle_detector'))
    if not compose:
        return [Node(package=pkg, executable=name, name=name, parameters=common, output='screen')
                for pkg, _, name in plugins]
    components = []
    if camera:
        components.append(ComposableNode(
            package='ros_gz_bridge', plugin='ros_gz_bridge::RosGzBridge', name='camera_bridge',
            parameters=[{'config_file': os.path.join(share, 'config', 'camera_bridge.yaml'),
                         'use_sim_time': use_sim_time}],
            extra_arguments=[{'use_intra_process_comms': True}]))
    for package, plugin, name in plugins:
        components.append(ComposableNode(package=package, plugin=plugin, name=name, parameters=common,
                                        extra_arguments=[{'use_intra_process_comms': True}]))
    # Four executor threads; each component keeps its own mutually-exclusive callback
    # group. Planner pose/memory/watchdog stay serialized, independently of image work.
    environment = {}
    if camera:
        # Gazebo transport has process-global workers. Keep its bridge DSO mapped
        # until process exit, after those workers stop (Jazzy/Harmonic teardown).
        library = os.path.join(get_package_prefix('ros_gz_bridge'), 'lib', 'libros_gz_bridge.so')
        environment['LD_PRELOAD'] = ' '.join(filter(None, [library, os.environ.get('LD_PRELOAD', '')]))
    return [ComposableNodeContainer(additional_env=environment,
        package='rclcpp_components', executable='component_container_mt',
        name='perception_container', namespace='', parameters=[{'thread_num': 4}],
        composable_node_descriptions=components, output='screen')]


def generate_launch_description():
    share = get_package_share_directory('line_follower_bringup')
    common = [LaunchConfiguration('params_file'), {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'competition.yaml')),
        DeclareLaunchArgument('startup_delay_sec', default_value='20.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('gcs_snapshot_enabled', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('compose', default_value='true', description='False only for baseline measurement'),
        DeclareLaunchArgument('camera_bridge', default_value='false',
                              description='Enable the image-only Gazebo component (false for replay/fake sensors)'),
        DeclareLaunchArgument('obstacles', default_value='false'),
        Node(package='line_follower_mission', executable='mission_manager', parameters=common + [{
            'startup_delay_sec': ParameterValue(LaunchConfiguration('startup_delay_sec'), value_type=float),
            'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
            'gcs_snapshot_enabled': ParameterValue(LaunchConfiguration('gcs_snapshot_enabled'), value_type=bool)}],
             output='screen'),
        OpaqueFunction(function=_perception),
        Node(package='line_follower_control', executable='motion_controller', parameters=common, output='screen'),
        Node(package='line_follower_control', executable='safety_guard', parameters=common,
             remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))], output='screen'),
    ])
