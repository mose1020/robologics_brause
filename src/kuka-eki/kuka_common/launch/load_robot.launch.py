import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_package",
        default_value="kuka_kr3_description",
        description="Robot description package.",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_file",
        default_value="robot.urdf.xacro",
        description="Robot description file located in <robot_description_package>/urdf/ .",
    ))

    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), "urdf", robot_description_file]),
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description]
    )
    return LaunchDescription(
        declared_arguments + [
            node_robot_state_publisher,
            joint_state_publisher_gui
        ])
