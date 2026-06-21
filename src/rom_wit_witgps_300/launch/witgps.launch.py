import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rom_wit_witgps_300'),
        'config',
        'witgps.yaml',
    )

    return LaunchDescription([
        Node(
            package='rom_wit_witgps_300',
            executable='wit_gps_300_ttl',
            name='wit_gps_300_ttl',
            output='screen',
            parameters=[config],
        ),
    ])
