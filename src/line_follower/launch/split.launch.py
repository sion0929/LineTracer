"""Split mode: perception / map / planning (dummy) / control (dummy) as four nodes.

Same world, bridge and 20 s start delay as line_follower.launch.py (docs/architecture/NODE_SPLIT.md).
The params file's `follower_node:` section is applied to all four nodes.

    ros2 launch line_follower split.launch.py [mode:=rule|dl] [params_file:=<yaml>]
                                              [sim:=true] [gui:=true] [start_delay:=20.0]
                                              [planning:=dummy|none] [control:=dummy|none]

planning:=none / control:=none leave that node out so a team runs its own node in its place
(same topics: /lf/map/patch + /lf/pose -> /lf/path -> /cmd_vel; docs/NODE_DEV_GUIDE.md).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from line_follower.split.params import SPLIT_NODES, write_all_nodes


def split_nodes(context):
    params = write_all_nodes(LaunchConfiguration('params_file').perform(context))
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    skip = {f'{role}_node' for role in ('planning', 'control')
            if LaunchConfiguration(role).perform(context) == 'none'}
    nodes = [Node(package='line_follower', executable=name, name=name,
                  parameters=[params, {'use_sim_time': sim_time}], output='screen')
             for name in SPLIT_NODES if name not in skip]
    return [TimerAction(period=LaunchConfiguration('start_delay'), actions=nodes)]


def generate_launch_description():
    pkg = get_package_share_directory('line_follower')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'line_track.sdf')
    gui = LaunchConfiguration('gui')
    sim = IfCondition(LaunchConfiguration('sim'))
    gz_args = PythonExpression([
        "'-r -v 2 ' + ('' if '", gui, "' == 'true' else '-s --headless-rendering ') + '", world, "'"])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_delay', default_value='20.0'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='start Gazebo + bridge (false: robot / external simulator)'),
        DeclareLaunchArgument('use_sim_time', default_value=LaunchConfiguration('sim')),
        # mode:=rule (default) or mode:=dl picks config/<mode>.yaml; params_file:=<file> overrides it
        DeclareLaunchArgument('mode', default_value='rule'),
        DeclareLaunchArgument('planning', default_value='dummy',
                              description='dummy (current RulePlanner) or none (run your own planning node)'),
        DeclareLaunchArgument('control', default_value='dummy',
                              description='dummy (current controller) or none (run your own control node)'),
        DeclareLaunchArgument('params_file', default_value=PythonExpression(
            ["'", os.path.join(pkg, 'config'), "/' + '", LaunchConfiguration('mode'), "' + '.yaml'"])),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'), condition=sim),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(), condition=sim),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}],
             output='screen', condition=sim),
        OpaqueFunction(function=split_nodes),
    ])
