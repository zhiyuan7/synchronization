import os

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory

tree_config = os.path.join(
    get_package_share_directory('tree'), 'config', 'offset.yml'
)

def generate_launch_description():

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='tree',
                plugin='TreeNode',
                name='tree',
                parameters=[tree_config]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        container,
        # Node(
        #     package='foxglove_bridge',
        #     executable='foxglove_bridge',
        #     name='fg'
        # )
    ])