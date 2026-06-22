import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rom_wit_wtgahrs3_ttl')
    params_file = os.path.join(pkg_share, 'config', 'wtgahrs3.yaml')

    return LaunchDescription([
        Node(
            package='rom_wit_wtgahrs3_ttl',
            executable='wit_wtgahrs3_ttl',
            name='wit_wtgahrs3_ttl',
            output='screen',
            parameters=[params_file],
        ),
    ])
