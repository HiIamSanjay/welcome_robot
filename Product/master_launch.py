import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    This is the master launch file for the entire robot system.
    It launches the individual launch files from each package.
    
    To run your entire robot, you will now only need to execute:
    ros2 launch master_launch.py
    """
    
    # Get the share directory for each package
    ai_share_dir = get_package_share_directory('ai')
    input_share_dir = get_package_share_directory('input')
    ui_share_dir = get_package_share_directory('ui')
    remote_control_share_dir = get_package_share_directory('remote_control')

    # Include each package's launch file
    ai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ai_share_dir, 'launch', 'ai_launch.py')
        )
    )
    
    input_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(input_share_dir, 'launch', 'input_launch.py')
        )
    )

    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ui_share_dir, 'launch', 'ui_launch.py')
        )
    )

    remote_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(remote_control_share_dir, 'launch', 'remote_control_launch.py')
        )
    )

    return LaunchDescription([
        ai_launch,
        input_launch,
        ui_launch,
        remote_control_launch,
    ])

