import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory

tree_config = os.path.join(
    get_package_share_directory('tree'), 'config', 'offset.yml'
)
bag_name = 'test'
bag_file_path = os.path.join(
    get_package_share_directory('data'), bag_name, bag_name + '.db3'
)

def generate_launch_description():

    # 1. 定义播放 bag 包的进程
    play_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_file_path,
            '--clock',   # 发布 /clock 话题
            '--topics', '/detect/detections', '/stm32'  # 只播放这两个话题
        ],
        output='screen'
    )

    # 2. 定义容器，注意给每个节点都加上 use_sim_time
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
                # 开启虚拟时间
                parameters=[{'use_sim_time': True}] 
            ),
            ComposableNode(
                package='tree',
                plugin='TreeNode',
                name='tree',
                # 这里的参数列表包含了配置文件和虚拟时间设置
                parameters=[tree_config, {'use_sim_time': True}] 
            )
        ],
        output='screen',
        # 这一行通常建议加上，确保容器本身也感知时间（虽然主要靠里面的 Node 感知）
        parameters=[{'use_sim_time': True}] 
    )

    return LaunchDescription([
        play_bag,  # <--- 把 bag 播放进程加入启动列表
        container,
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='fg',
            parameters=[{'use_sim_time': True}] 
        )
    ])