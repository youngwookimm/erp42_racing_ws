from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ISRO_P2_Driver'),
                'launch',
                'ISRO_P2_Driver.launch.py'
            )
        )
    )

    ntrip_node = Node(
        package='ISRO_P2_Driver',
        executable='ntrip.py',
        # output='screen'
        arguments=['--ros-args', '--log-level', 'warn']
    )

    utm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('erp42_racing_localization'),
                'launch',
                'ros2_lla_utm.launch.py'
            )
        )
    )

    return LaunchDescription([
        driver_launch,
        ntrip_node,
        utm_launch
    ])