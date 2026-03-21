from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    origin_csv = LaunchConfiguration('origin_csv')
    origin_x_column = LaunchConfiguration('origin_x_column')
    origin_y_column = LaunchConfiguration('origin_y_column')

    return LaunchDescription([
        DeclareLaunchArgument('origin_csv', default_value='fix_utm_v2.csv'),
        DeclareLaunchArgument('origin_x_column', default_value='L1_UTM_X'),
        DeclareLaunchArgument('origin_y_column', default_value='L1_UTM_Y'),
        Node(
            package='erp42_racing_localization',
            executable='lla_utm_node',
            name='lla_utm_node',
            output='screen',
            parameters=[
                {'utm_zone': 52},
                {'hemisphere': 'North'},
            ],
            remappings=[
                ('/gps/fix', '/fix'),
            ],
        ),
        Node(
            package='erp42_racing_localization',
            executable='utm_origin_shift_node.py',
            name='utm_origin_shift_node',
            output='screen',
            parameters=[
                {'origin_package': 'erp42_racing_planning'},
                {'origin_csv': origin_csv},
                {'origin_x_column': origin_x_column},
                {'origin_y_column': origin_y_column},
                {'input_topic': '/utm'},
                {'output_topic': '/utm_tm'},
                {'output_frame_id': 'map'},
            ],
        ),
        Node(
            package='erp42_racing_localization',
            executable='vehicle_pose_viz_node.py',
            name='vehicle_pose_viz_node',
            # output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[
                {'pose_topic': '/utm_tm'},
                {'imu_topic': '/imu/data'},
                {'output_pose_topic': '/vehicle_pose'},
                {'output_frame_id': 'map'},
            ],
        ),
        # Node(
        #     package='erp42_racing_localization',
        #     executable='csv_to_utm_pub.py',
        #     name='csv_to_utm_pub',
        #     output='screen'
        # )
    ])
