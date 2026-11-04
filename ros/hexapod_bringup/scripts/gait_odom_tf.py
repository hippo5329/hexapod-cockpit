#!/usr/bin/env python3
"""Broadcasts the firmware's gait odometry as a TF: odom -> base_footprint.

/odom/gait (nav_msgs/Odometry, best effort) carries the body pose the gait has walked, integrated
from commanded steps. This node republishes each message's pose as a transform, with the message's
own stamp and frames, so RViz (fixed frame odom) shows the robot walking across the floor.

It is the only parent base_footprint may have: once an EKF fuses /odom/gait with the IMU and
publishes odom -> base_footprint itself, this node must not run.

Parameters:
  child_frame  overrides the message's child_frame_id (default: empty = use the message's).
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class GaitOdomTf(Node):
    def __init__(self):
        super().__init__("gait_odom_tf")
        self.child = self.declare_parameter("child_frame", "").value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "odom/gait", self.on_odom, qos_profile_sensor_data)

    def on_odom(self, m):
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = self.child or m.child_frame_id
        p = m.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GaitOdomTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
