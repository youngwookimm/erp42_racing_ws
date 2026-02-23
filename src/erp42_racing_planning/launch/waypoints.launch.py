from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'erp42_racing_planning'

    return LaunchDescription([
        Node(
            package=package_name,
            executable='waypoints_L1_node',
            name='waypoints_node',
            namespace='L1',
            parameters=[
                {'csv_name': 'fix_utm_v2.csv'},
                {'frame_id': 'map'},
                {'lane_prefix': 'L1'},
                {'origin_lane_prefix': 'L1'},
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package=package_name,
            executable='waypoints_R1_node',
            name='waypoints_node',
            namespace='R1',
            parameters=[
                {'csv_name': 'fix_utm_v2.csv'},
                {'frame_id': 'map'},
                {'lane_prefix': 'R1'},
                {'origin_lane_prefix': 'L1'},
            ],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        )
    ])
