from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pure_pursuit_global = Node(
        package='erp42_racing_control',
        executable='pure_pursuit_global_node',
        name='pure_pursuit_global_node',
        output='screen',
    )

    aeb_node = Node(
        package='erp42_racing_control',
        executable='aeb_node',
        name='aeb_node',
        output='screen',
    )

    vehicle_cmd_gate = Node(
        package='erp42_racing_control',
        executable='vehicle_cmd_gate_node',
        name='vehicle_cmd_gate_node',
        output='screen',
    )

    return LaunchDescription([
        pure_pursuit_global,
        aeb_node,
        vehicle_cmd_gate,
    ])
