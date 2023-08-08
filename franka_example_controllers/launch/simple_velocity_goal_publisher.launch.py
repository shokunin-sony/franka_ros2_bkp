#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    goal_velocity_value = LaunchConfiguration('goal_velocity')
    goal_velocity_value_launch_arg = DeclareLaunchArgument(
        'goal_velocity',
        description="goal joint/goal velocity",
        default_value="[-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
    )

    runtime_vel_demo = Node(
        package="franka_example_controllers",
        executable="velocity_goal_pub",
        name="Velocity_goal_publisher",
        output="both",
        parameters=[
            {'goal_velocity': goal_velocity_value},
        ],
    )

    return LaunchDescription(
        [
            goal_velocity_value_launch_arg,
            runtime_vel_demo,
        ],
    )
