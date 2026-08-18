from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('co_driver')
    default_config = os.path.join(pkg_share, 'config', 'co_driver.yaml')
    default_topics = os.path.join(pkg_share, 'config', 'co_driver_topics.jsonc')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Main config yaml (output/context/scoring/selection/postprocess)',
    )
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value=default_topics,
        description='Input/drive topic-list JSON (inputs/drives + influence matrix)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time',
    )

    co_driver_node = Node(
        package='co_driver',
        executable='co_driver_node',
        name='co_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config'),
            {
                # Override the yaml's topics_file with the launch argument.
                'topics_file': LaunchConfiguration('topics'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    ld = LaunchDescription()
    ld.add_action(config_arg)
    ld.add_action(topics_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(co_driver_node)
    return ld
