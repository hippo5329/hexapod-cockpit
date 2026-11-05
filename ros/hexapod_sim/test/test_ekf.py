"""The EKF (hexapod_bringup's view_robot.launch.py, odom_tf:=ekf) on the sim node.

odom -> base_footprint has one parent, the EKF, and the TF is /odom's pose. Standing, the heading
holds (the gait declares its zero exact and the board takes the gyro's bias out). Walking, the
position is the gait's. Turning, the heading follows the gyro and not the slipping feet: much
closer to ground truth than /odom/gait. The sim is deterministic (seed 5329), so the bounds sit
past the numbers measured in both robot images (comments).
"""
import math
import os
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

NODE = os.environ["HEXA_SIM_NODE"]
STAMPED = os.environ.get("ROS_DISTRO") != "jazzy"
if STAMPED:
    from geometry_msgs.msg import TwistStamped as CmdType
else:
    from geometry_msgs.msg import Twist as CmdType


def yaw_of(m):
    q = m.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Stack:
    def __init__(self):
        self.sim = subprocess.Popen([NODE], start_new_session=True)
        self.launch = subprocess.Popen(["ros2", "launch", "hexapod_bringup", "view_robot.launch.py", "rviz:=false"],
                                       start_new_session=True)
        rclpy.init()
        self.node = rclpy.create_node("test_ekf")
        self.ekf = self.gait = self.truth = None
        self.node.create_subscription(Odometry, "odom", lambda m: setattr(self, "ekf", m), 10)
        self.node.create_subscription(Odometry, "odom/gait", lambda m: setattr(self, "gait", m),
                                      qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "sim/ground_truth", lambda m: setattr(self, "truth", m),
                                      qos_profile_sensor_data)
        self.pub = self.node.create_publisher(CmdType, "cmd_vel", 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        for p, sig in ((self.launch, signal.SIGINT), (self.sim, signal.SIGTERM)):
            p.send_signal(sig)
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()

    def send(self, vx, wz=0.0):
        m = CmdType()
        t = m.twist if STAMPED else m
        if STAMPED:
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
        t.linear.x = vx
        t.angular.z = wz
        self.pub.publish(m)

    def spin_for(self, seconds, vx=None, wz=0.0):
        end = time.monotonic() + seconds
        nxt = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if vx is not None and time.monotonic() >= nxt:
                self.send(vx, wz)
                nxt += 0.05

    def poses(self):
        return {k: (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m))
                for k, m in (("ekf", self.ekf), ("gait", self.gait), ("truth", self.truth))}


def test_ekf_on_the_sim():
    s = Stack()
    try:
        end = time.monotonic() + 30
        while time.monotonic() < end and (s.ekf is None or s.truth is None):
            s.spin_for(0.5)
        assert s.ekf is not None, "no /odom from the EKF in 30 s"
        assert (s.ekf.header.frame_id, s.ekf.child_frame_id) == ("odom", "base_footprint")
        nodes = s.node.get_node_names()
        assert "ekf_filter_node" in nodes and "gait_odom_tf" not in nodes, f"odom_tf nodes: {nodes}"
        s.spin_for(2.0)
        assert s.gait.twist.covariance[35] < 1e-6, "gait twist not declared exact while standing"

        a = s.poses()
        s.spin_for(10.0)
        b = s.poses()
        drift = abs(wrap(b["ekf"][2] - a["ekf"][2]))
        assert drift < 0.01, f"EKF heading drifted {drift:.4f} rad in 10 s standing"  # 30 s: 0.001-0.003

        a = s.poses()
        s.spin_for(4.0, vx=0.05)
        assert s.gait.twist.covariance[35] > 1e-3, "gait twist declared exact while walking"
        s.send(0.0)
        s.spin_for(3.0)
        b = s.poses()
        ekf_dx, gait_dx = b["ekf"][0] - a["ekf"][0], b["gait"][0] - a["gait"][0]
        assert abs(ekf_dx - gait_dx) < 0.01, f"walk: EKF +{ekf_dx:.3f} m, gait +{gait_dx:.3f} m"  # 0.197 / 0.200

        a = s.poses()
        s.spin_for(3.0, vx=0.0, wz=0.4)
        s.send(0.0)
        s.spin_for(3.0)
        b = s.poses()
        d = {k: wrap(b[k][2] - a[k][2]) for k in b}
        ekf_err, gait_err = abs(wrap(d["ekf"] - d["truth"])), abs(wrap(d["gait"] - d["truth"]))
        msg = f"turn: truth +{d['truth']:.3f}, EKF +{d['ekf']:.3f}, gait +{d['gait']:.3f} rad"
        assert d["truth"] > 0.9, msg
        assert ekf_err < 0.03 and ekf_err < gait_err / 2, msg  # EKF 0.011, gait 0.067

        t = s.tf.lookup_transform("odom", "base_footprint", Time())
        o = s.ekf.pose.pose
        off = math.hypot(t.transform.translation.x - o.position.x, t.transform.translation.y - o.position.y)
        q = t.transform.rotation
        yaw_off = abs(wrap(2 * math.atan2(q.z, q.w) - yaw_of(s.ekf)))
        assert off < 0.002 and yaw_off < 0.002, f"TF off /odom by {off * 1000:.1f} mm, {yaw_off:.4f} rad"
    finally:
        s.close()
