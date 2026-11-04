"""gait_odom_tf: an /odom/gait message becomes the odom -> base_footprint transform, stamp and all."""
import math
import os
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

NODE = os.environ["HEXA_ODOM_TF"]


def odom(x, y, yaw, stamp):
    m = Odometry()
    m.header.stamp = stamp
    m.header.frame_id = "odom"
    m.child_frame_id = "base_footprint"
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    m.pose.pose.orientation.z = math.sin(yaw / 2)
    m.pose.pose.orientation.w = math.cos(yaw / 2)
    return m


def test_odom_becomes_tf():
    proc = subprocess.Popen([sys.executable, NODE])
    rclpy.init()
    node = rclpy.create_node("test_gait_odom_tf")
    pub = node.create_publisher(Odometry, "odom/gait", qos_profile_sensor_data)
    buf = Buffer()
    TransformListener(buf, node)
    try:
        poses = [(0.0, 0.0, 0.0), (0.12, -0.03, 0.4), (0.30, 0.05, -1.2)]
        for x, y, yaw in poses:
            stamp = node.get_clock().now().to_msg()
            got = None
            end = time.monotonic() + 10.0
            while time.monotonic() < end and got is None:
                pub.publish(odom(x, y, yaw, stamp))
                rclpy.spin_once(node, timeout_sec=0.05)
                if buf.can_transform("odom", "base_footprint", Time.from_msg(stamp)):
                    got = buf.lookup_transform("odom", "base_footprint", Time.from_msg(stamp))
            assert got is not None, f"no odom -> base_footprint at the message's stamp for pose {x, y, yaw}"
            tr, q = got.transform.translation, got.transform.rotation
            assert abs(tr.x - x) < 1e-9 and abs(tr.y - y) < 1e-9 and tr.z == 0.0
            assert abs(2 * math.atan2(q.z, q.w) - yaw) < 1e-9
            assert got.header.stamp == stamp
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        proc.wait(timeout=10)
