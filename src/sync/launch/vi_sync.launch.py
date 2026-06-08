import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory


tree_config = os.path.join(
    get_package_share_directory('tree'), 'config', 'offset.yml'
)
sync_config = os.path.join(
    get_package_share_directory('sync'), 'config', 'sync.yml'
)
bag_name = 'test'
bag_file_path = os.path.join(
    get_package_share_directory('data'), bag_name, bag_name + '.db3'
)


def generate_launch_description():

    # 1. 播放 bag，发布 /clock 以驱动虚拟时间
    play_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_file_path,
            '--clock',
            '--topics', '/detect/detections', '/stm32'
        ],
        output='screen'
    )

    # 2. 组件容器：pos / tree / sync 共享同一进程
    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pos',
                plugin='PosNode',
                name='pos',
                parameters=[{'use_sim_time': True}]
            ),
            ComposableNode(
                package='tree',
                plugin='TreeNode',
                name='tree',
                parameters=[tree_config, {'use_sim_time': True}]
            ),
            ComposableNode(
                package='sync',
                plugin='SyncNode',
                name='sync',
                parameters=[sync_config, {'use_sim_time': True}]
            ),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        play_bag,
        container,
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='fg',
            parameters=[{'use_sim_time': True}]
        )
    ])
