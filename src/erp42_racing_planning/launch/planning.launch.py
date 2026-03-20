from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="erp42_racing_planning",
            executable="waypoint_loader",
            name="waypoint_loader",
            output="screen",
        ),

        Node(
            package="erp42_racing_planning",
            executable="scene_builder_node",
            name="scene_builder_node",
            output="screen",
        ),

        Node(
            package="erp42_racing_planning",
            executable="behavior_planner_node",
            name="behavior_planner_node",
            output="screen",
        ),

        Node(
            package="erp42_racing_planning",
            executable="local_planner_node",
            name="local_planner_node",
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),

    ])
