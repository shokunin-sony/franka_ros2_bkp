#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    goal_position_value = LaunchConfiguration('goal_position')
    goal_position_value_launch_arg = DeclareLaunchArgument(
        'goal_position',
        description="goal joint/goal position",
        default_value="[-1.5708, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]",
    )

    runtime_pos_demo = Node(
        package="franka_example_controllers",
        executable="goal_pub",
        name="Goal_publisher",
        output="both",
        parameters=[
            {'goal_position': goal_position_value},
        ],
    )

    return LaunchDescription(
        [
            goal_position_value_launch_arg,
            runtime_pos_demo,
        ],
    )
