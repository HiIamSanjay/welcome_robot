from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Launches the main GUI on the primary monitor and the eye animation
    on the secondary monitor by explicitly setting window positions.
    """
    # The main_gui_node will position itself using Tkinter's geometry method.
    # It will inherit the default DISPLAY environment variable.
    main_gui_node = Node(
        package='ui',
        executable='main_gui',
        name='main_gui_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        emulate_tty=True,
        parameters=[{'use_sim_time': False}],
    )

    # For the eye_animation_node, we set an environment variable BEFORE it starts.
    # This tells the underlying SDL library where to create the window.
    # The coordinates 1920,557 come from your 'xrandr' output for HDMI-1-0.
    eye_animation_node = Node(
        package='ui',
        executable='eye_animation',
        name='eye_animation_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        emulate_tty=True,
        parameters=[{'use_sim_time': False}],
        prefix="env SDL_VIDEO_WINDOW_POS=1920,557"
    )

    return LaunchDescription([
        main_gui_node,
        eye_animation_node,
    ])


