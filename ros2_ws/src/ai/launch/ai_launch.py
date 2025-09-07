from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the ai package.
    """
    return LaunchDescription([
        Node(
            package='ai',
            executable='gemini_node',
            name='gemini_node',
            output='screen'
        ),
        Node(
            package='ai',
            executable='emotion_recognition',
            name='emotion_recognition_node',
            output='screen'
        ),
        Node(
            package='ai',
            executable='person_detection',
            name='person_detection_node',
            output='screen'
        ),
    ])

