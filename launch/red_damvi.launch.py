"""red_damvi - co_driver arbitrating PPcontroller (/drive_main) vs gap_follow (/drive_gf).

  ros2 launch co_driver red_damvi.launch.py

Arguments
  gap_follow:=true|false   also launch gap_follow, forced onto /drive_gf.
                           gap_follow's own default is drive_topic:=/drive, which
                           collides with co_driver's output, so we always override it.
  monitor:=true|false      also launch drive_monitor (switch / oscillation logging)
  csv:=<path>              monitor writes a per-status-sample CSV here
  config:= / topics:=      override the yaml / topics JSON
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('co_driver')
    default_config = os.path.join(share, 'config', 'co_driver_red_damvi.yaml')
    default_topics = os.path.join(share, 'config', 'co_driver_red_damvi_topics.jsonc')

    args = [
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('topics', default_value=default_topics),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gap_follow', default_value='true'),
        DeclareLaunchArgument('monitor', default_value='true'),
        DeclareLaunchArgument('csv', default_value=''),
        # gap_follow subscribes to the scan RELIABLE, while LiDAR drivers and rosbag
        # replays publish it BEST_EFFORT. Set scan_bridge:=true to relay across the
        # mismatch instead of patching gap_follow.
        DeclareLaunchArgument('scan_bridge', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        # RViz markers above the vehicle: which controller is selected and why.
        DeclareLaunchArgument('markers', default_value='true'),
    ]

    co_driver = Node(
        package='co_driver',
        executable='co_driver_node',
        name='co_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config'),
            {
                'topics_file': LaunchConfiguration('topics'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    monitor = Node(
        package='co_driver',
        executable='drive_monitor.py',
        name='drive_monitor',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('monitor')),
        parameters=[{
            'status_topic': '/co_driver_node/status',
            'output_topic': '/drive',
            'watch_input': 'localization',
            'csv_path': LaunchConfiguration('csv'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    scan_bridge = Node(
        package='co_driver',
        executable='scan_qos_bridge.py',
        name='scan_qos_bridge',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('scan_bridge')),
        parameters=[{'in_topic': LaunchConfiguration('scan_topic'),
                     'out_topic': '/scan_reliable'}],
    )

    markers = Node(
        package='co_driver',
        executable='status_markers.py',
        name='co_driver_markers',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('markers')),
    )

    actions = [*args, co_driver, monitor, scan_bridge, markers]

    # gap_follow lives in its own repo; launching it is optional and best-effort.
    try:
        gf_share = get_package_share_directory('gap_follow')
    except PackageNotFoundError:
        actions.append(LogInfo(
            msg='gap_follow is not in this workspace - launching co_driver only. '
                'Start gap_follow yourself with drive_topic:=/drive_gf.'))
    else:
        actions.append(Node(
            package='gap_follow',
            executable='gap_follow',
            name='gap_follow',
            # gap_follow's config yaml is keyed gap_follow/gap_follow, so the node
            # must run in the gap_follow namespace or none of its parameters are
            # declared and it aborts on the first get_parameter().
            namespace='gap_follow',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('gap_follow')),
            parameters=[
                os.path.join(gf_share, 'config', 'gap_follow.yaml'),
                # Must override: gap_follow's default drive_topic is /drive, which is
                # co_driver's output topic.
                {'drive_topic': '/drive_gf',
                 # follows scan_bridge: read the relayed scan when the bridge is on
                 'lidar_scan_topic': PythonExpression([
                     "'/scan_reliable' if '",
                     LaunchConfiguration('scan_bridge'),
                     "'.lower() in ('true', '1') else '",
                     LaunchConfiguration('scan_topic'), "'"]),
                 'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ))

    return LaunchDescription(actions)
