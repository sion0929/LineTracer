import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('line_follower')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'line_track.sdf')
    config = os.path.join(pkg, 'config')

    gui = LaunchConfiguration('gui')
    delay = LaunchConfiguration('start_delay')
    gz_args = PythonExpression([
        "'-r -v 2 ' + ('' if '", gui, "' == 'true' else '-s --headless-rendering ') + '", world, "'"])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('start_delay', default_value='20.0'),
        # mode:=rule (default) or mode:=dl picks config/<mode>.yaml; params:=<file> overrides it
        DeclareLaunchArgument('mode', default_value='rule'),
        DeclareLaunchArgument('params', default_value=PythonExpression(
            ["'", config, "/' + '", LaunchConfiguration('mode'), "' + '.yaml'"])),
        # safety:=sim (default, generous limits) or safety:=real (Waffle Pi limits) picks
        # config/safety_<safety>.yaml; safety_params:=<file> overrides it
        DeclareLaunchArgument('safety', default_value='sim'),
        DeclareLaunchArgument('safety_params', default_value=PythonExpression(
            ["'", config, "/safety_' + '", LaunchConfiguration('safety'), "' + '.yaml'"])),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(config, 'bridge.yaml')}],
             output='screen'),
        # the only /cmd_vel publisher: starts at once and holds zero for start_delay seconds of
        # wall time (its own gate, independent of the timer below) and until all sensors are up
        Node(package='line_follower', executable='safety_guard', name='safety_guard',
             parameters=[LaunchConfiguration('safety_params'),
                         {'use_sim_time': True,
                          'start_delay': PythonExpression(['float(', delay, ')'])}],
             output='screen'),
        TimerAction(period=delay, actions=[
            Node(package='line_follower', executable='follower_node', name='follower_node',
                 parameters=[LaunchConfiguration('params'), {'use_sim_time': True}],
                 remappings=[('/cmd_vel', '/line_follower/cmd_raw')],
                 output='screen')]),
    ])
