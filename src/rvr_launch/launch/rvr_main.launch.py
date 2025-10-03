from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rvr_hardware',
            executable='camera_node',
            name='camera_node',
        ),
        Node(
            package='rvr_hardware',
            executable='bridge_node',
            name='bridge_node',
        ),
        Node(
            package='rvr_ui',
            executable='joystick_node',
            name='joystick_node',
        ),
    ])