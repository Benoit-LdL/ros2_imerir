from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "launch", "turtlebot3_house.launch.py"]
    )
    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_navigation2"), "launch", "navigation2.launch.py"]
    )
    map_path = os.path.expanduser("~/ros_workshop_ws/src/map/map_gazebo.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            name='y_pose',
            default_value='1.0',
            description='Y position for spawning the robot in Gazebo'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'y_pose': LaunchConfiguration('y_pose')
            }.items()
        ),

        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'map': map_path,
                        'use_sim_time': 'True'
                    }.items()
                )
            ]
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'imerir_nav', 'script_nav'],
            output='screen'
        )
    ])

