import os

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory

camera_config = os.path.join(
    get_package_share_directory('camera'), 'config', 'camera.yml'
)
detect_config = os.path.join(
    get_package_share_directory('detect'), 'config', 'detect.yml'
)
can_config = os.path.join(
    get_package_share_directory('can'), 'config', 'can.yml'
)
tree_config = os.path.join(
    get_package_share_directory('tree'), 'config', 'offset.yml'
)
trigger_config = os.path.join(
    get_package_share_directory('trigger'), 'config', 'trigger.yml'
)

def generate_launch_description():

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='trigger',
                plugin='TriggerNode',
                name='trigger',
                parameters=[trigger_config]
            ),
            ComposableNode(
                package='pos',
                plugin='PosNode',
                name='pos'
            ),
            ComposableNode(
                package='detect',
                plugin='DetectNode',
                name='detect',
                parameters=[detect_config]
            ),
            ComposableNode(
                package='camera',
                plugin='CameraNode',
                name='camera',
                parameters=[camera_config]
            ),
            ComposableNode(
                package='can',
                plugin='CanNode',
                name='can',
                parameters=[can_config]
            ),
            ComposableNode(
                package='tree',
                plugin='TreeNode',
                name='tree',
                parameters=[tree_config]
            ),
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