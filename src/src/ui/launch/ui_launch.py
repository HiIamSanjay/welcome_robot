from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Launches the main GUI, eye animation, and the new interaction manager.
    """
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

    # --- ADDED INTERACTION MANAGER ---
    # This new node will manage the overall interaction flow.
    interaction_manager_node = Node(
        package='ui', # Assuming the new node is in your 'ui' package
        executable='interaction_manager', # This must match your script's executable name
        name='interaction_manager_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        main_gui_node,
        eye_animation_node,
        interaction_manager_node, # Add the new node to the launch list
    ])
