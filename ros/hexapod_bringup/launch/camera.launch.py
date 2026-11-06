"""The miniHexa's camera board on ROS: camera_bridge (scripts/camera_bridge.py).

    ros2 launch hexapod_bringup camera.launch.py url:=http://<camera ip>:81/stream [camera_info_file:=...]

camera_info_file defaults to config/hexacam1_qvga.yaml (this robot's camera at qvga); empty: nominal
intrinsics from hfov_deg.

Publishes camera/image_raw/compressed and camera/camera_info. For a raw image (a consumer that needs
one): ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=camera/image_raw/compressed -r out:=camera/image_raw
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("url", description="the camera board's stream: http://<camera ip>:81/stream"),
        DeclareLaunchArgument("camera_info_file", default_value=PathJoinSubstitution(
            [FindPackageShare("hexapod_bringup"), "config", "hexacam1_qvga.yaml"])),
        DeclareLaunchArgument("hfov_deg", default_value="58.4"),
        Node(package="hexapod_bringup", executable="camera_bridge", name="camera_bridge", output="screen",
             parameters=[{"url": LaunchConfiguration("url"),
                          "camera_info_file": LaunchConfiguration("camera_info_file"),
                          "hfov_deg": LaunchConfiguration("hfov_deg")}]),
    ])
