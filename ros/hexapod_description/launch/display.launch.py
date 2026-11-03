"""The miniHexa in RViz.

    ros2 launch hexapod_description display.launch.py                # standing pose
    ros2 launch hexapod_description display.launch.py source:=gui    # a slider per joint
    ros2 launch hexapod_description display.launch.py source:=robot  # /joint_states from the firmware

source:=stand joint_state_publisher holds the gait's standing pose (config/stand_pose.yaml).
source:=gui   joint_state_publisher_gui: move each joint by hand. Needs the
              joint_state_publisher_gui package (not in every image).
source:=robot nothing publishes joint states here; the firmware's base application does
              (sim mode on a bare ESP32, or the robot), through a micro-ROS agent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("hexapod_description")
    xacro_file = os.path.join(share, "urdf", "minihexa.urdf.xacro")
    rviz_file = os.path.join(share, "rviz", "minihexa.rviz")
    stand_file = os.path.join(share, "config", "stand_pose.yaml")
    description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("source", default_value="stand", choices=["stand", "gui", "robot"],
                              description="who publishes /joint_states"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             parameters=[stand_file], condition=IfCondition(EqualsSubstitution(LaunchConfiguration("source"), "stand"))),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("source"), "gui"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_file],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
