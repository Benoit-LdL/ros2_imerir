from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the paths to the launch files
    bringup_dir = get_package_share_directory('open_manipulator_bringup')
    moveit_config_dir = get_package_share_directory('open_manipulator_moveit_config')

    gazebo_launch = os.path.join(bringup_dir, 'launch', 'gazebo.launch.py')
    moveit_launch = os.path.join(moveit_config_dir, 'launch', 'moveit_core.launch.py')

    # Set LC_NUMERIC to ensure MoveIt works properly
    lc_numeric_env = SetEnvironmentVariable('LC_NUMERIC', 'en_US.UTF-8')

    # Define your custom node
    custom_node = Node(
        package='imerir_manip',
        executable='imerir_manip',
        output='screen'
    )

    delayed_custom_node = TimerAction(
        period=15.0,  # Wait 15 seconds
        actions=[custom_node]
    )

    return LaunchDescription([
        # Set locale for RViz/MoveIt
        lc_numeric_env,

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # Launch MoveIt (with RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch)
        ),

        # Launch your custom node
        custom_node
    ])
