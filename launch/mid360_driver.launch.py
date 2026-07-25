import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('mid360_driver')
    config_path = os.path.join(pkg_path, 'config', 'param.yaml')

    driver_node = Node(
        package='mid360_driver',
        executable='mid360_driver_node',
        name='mid360_driver',
        output='screen',
        parameters=[config_path]
    )

    return LaunchDescription([
        driver_node
    ])
