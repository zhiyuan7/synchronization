import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


sync_config = os.path.join(
    get_package_share_directory('sync'), 'config', 'sync.yml'
)


def generate_launch_description():

    sync_node = Node(
        package='sync',
        executable='sync',
        name='sync',
        parameters=[sync_config],
        output='screen'
    )

    return LaunchDescription([
        sync_node,
    ])
