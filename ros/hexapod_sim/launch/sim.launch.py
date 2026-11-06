"""The miniHexa simulated on the host, walking in RViz: no board, no agent.

    ros2 launch hexapod_sim sim.launch.py [slip:=0.05] [rviz:=true] [gait:=tripod|quad]

hexapod_sim_node stands in for the firmware (/cmd_vel in; /joint_states, /odom/gait,
/imu/data, /sonar and /sim/ground_truth out) in a walled 2 m x 2 m room. The robot is drawn where
its gait odometry says, as on the real robot; the green trail is where it really went (ground
truth), the orange one where it thinks it went; the sonar cone and the room's walls are drawn too.
Drive it with any /cmd_vel publisher (Twist on Jazzy, TwistStamped on Lyrical).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory("hexapod_bringup")
    rviz_file = os.path.join(get_package_share_directory("hexapod_sim"), "rviz", "sim.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("slip", default_value="0.05", description="fraction of stance motion lost"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gait", default_value="tripod", description="tripod or quad (middle legs up)"),
        Node(package="hexapod_sim", executable="hexapod_sim_node",
             parameters=[{"slip": ParameterValue(LaunchConfiguration("slip"), value_type=float),
                          "gait": LaunchConfiguration("gait")}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "view_robot.launch.py")),
            launch_arguments={"rviz": LaunchConfiguration("rviz"), "rviz_config": rviz_file}.items()),
    ])
