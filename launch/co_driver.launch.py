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
        description='기본 설정 yaml (output/context/scoring/selection/postprocess)',
    )
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value=default_topics,
        description='입력/드라이브 토픽 목록 JSON (inputs/drives + 영향 행렬)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부',
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
                # yaml 의 topics_file 을 launch 인자로 덮어씁니다.
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
