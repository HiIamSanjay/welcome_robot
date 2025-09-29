from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the main GUI and the eye animation node.
    Both are critical for user interaction and will respawn on failure.
    """
    return LaunchDescription([
        Node(
            package='ui',
            executable='main_gui',
            name='main_gui_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0, # Wait 2 seconds before restarting
        ),
        Node(
            package='ui',
            executable='eye_animation',
            name='eye_animation_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])

