import multiprocessing
import subprocess
import sys
import threading
from typing import List, Optional, Tuple, Union

import IPython
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from robot_interface.core.robot_modules.arm_controller.arm_controller import ArmController
from robot_interface.core.types import ArmControllerType, ControlMode, FloatSequenceType, StrSequenceType
from robot_interface.utils.concrete.concrete import concrete
from trajectory_msgs.msg import JointTrajectory


def launch_controller():
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    ls.run()


def generate_launch_description():
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                robot_ip_parameter_name, default_value='0.0.0.0', description='Hostname or IP address of the robot.'
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value='false',
                description='Visualize the robot in Rviz',
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value='true',
                description='Use fake hardware',
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value='false',
                description='Fake sensor commands. Only valid when "{}" is true'.format(
                    use_fake_hardware_parameter_name
                ),
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value='true',
                description=(
                    'Use Franka Gripper as an end-effector, otherwise, the robot is loaded ' 'without an end-effector.'
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])]
                ),
                launch_arguments={
                    robot_ip_parameter_name: robot_ip,
                    load_gripper_parameter_name: load_gripper,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    fake_sensor_commands_parameter_name: fake_sensor_commands,
                    use_rviz_parameter_name: use_rviz,
                }.items(),
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['move_to_start_example_controller'],
                output='screen',
            ),
        ]
    )


def task1():
    list_dir = subprocess.Popen(
        [
            "ros2",
            "launch",
            "franka_bringup",
            "move_to_start_example_controller.launch.py",
            "robot_ip:=0.0.0.0",
            "use_fake_hardware:=true",
        ]
    )
    list_dir.wait()


def task2():
    IPython.embed(
        colors="neutral",
        banner1="-- Entering robot console for test --",
        banner2="started",
        exit_msg="Exiting robot console. BYE!!",
    )
    # i = 0
    # while True:
    #     if i % 100000000 == 0:
    #         print("lack, good")
    #     i += 1


# Create processes
process1 = threading.Thread(target=task1)
process2 = threading.Thread(target=task2)

# Start processes
process1.start()
process2.start()

# Wait for processes to finish
process1.join()
process2.join()


# Code after both tasks are completed
