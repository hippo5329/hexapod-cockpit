"""The miniHexa in RViz, walking: the firmware's /joint_states, /odom/gait and /imu/data through a micro-ROS agent.

    ros2 launch hexapod_bringup view_robot.launch.py [odom_tf:=ekf|gait] [rviz:=true]

robot_state_publisher turns /joint_states into the legs' TF. odom -> base_footprint comes from
one node, never two (a frame has one parent):
  ekf   robot_localization's EKF (config/ekf.yaml): the gait's twist with the gyro's yaw rate,
        filtered odometry on /odom. The default.
  gait  gait_odom_tf: /odom/gait's pose as it is, for a board without an IMU.
RViz's fixed frame is odom, so the robot walks across the grid. The agent is not started here
(serial or UDP is the bench's choice). hexapod_sim's sim.launch.py includes this with its own
rviz_config (adds the ground truth trail).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = get_package_share_directory("hexapod_description")
    bringup = get_package_share_directory("hexapod_bringup")
    rviz_file = os.path.join(bringup, "rviz", "walk.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_file),
        DeclareLaunchArgument("odom_tf", default_value="ekf", choices=["ekf", "gait"],
                              description="who publishes odom -> base_footprint"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, "launch", "display.launch.py")),
            launch_arguments={"source": "robot", "rviz": LaunchConfiguration("rviz"),
                              "rviz_config": LaunchConfiguration("rviz_config")}.items()),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[os.path.join(bringup, "config", "ekf.yaml")],
             remappings=[("odometry/filtered", "odom")],
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("odom_tf"), "ekf"))),
        Node(package="hexapod_bringup", executable="gait_odom_tf",
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration("odom_tf"), "gait"))),
    ])
