"""The miniHexa in RViz, walking: the firmware's /joint_states and /odom/gait through a micro-ROS agent.

    ros2 launch hexapod_bringup view_robot.launch.py

robot_state_publisher turns /joint_states into the legs' TF; gait_odom_tf turns /odom/gait into
odom -> base_footprint. RViz's fixed frame is odom, so the robot walks across the grid and leaves a
trail of /odom/gait arrows. The agent is not started here (serial or UDP is the bench's choice).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = get_package_share_directory("hexapod_description")
    rviz_file = os.path.join(get_package_share_directory("hexapod_bringup"), "rviz", "walk.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, "launch", "display.launch.py")),
            launch_arguments={"source": "robot", "rviz": LaunchConfiguration("rviz"),
                              "rviz_config": rviz_file}.items()),
        Node(package="hexapod_bringup", executable="gait_odom_tf"),
    ])
