"""hexapod_sim_node stands in for the firmware: same topics, rates and gait, plus ground truth.

Walks 0.05 m/s for 4 s, then one zero, as the board check does: /odom/gait gains the command,
/sim/ground_truth trails it by the slip, and the robot stands again.
"""
import math
import os
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

NODE = os.environ["HEXA_SIM_NODE"]
STAMPED = os.environ.get("ROS_DISTRO") != "jazzy"
if STAMPED:
    from geometry_msgs.msg import TwistStamped as CmdType
else:
    from geometry_msgs.msg import Twist as CmdType

STAND = {"coxa": 0.0, "femur": 0.297447, "tibia": -1.772154}  # the gait's neutral stance


class Probe:
    def __init__(self, slip):
        self.proc = subprocess.Popen([NODE, "--ros-args", "-p", f"slip:={slip}"])
        rclpy.init()
        self.node = rclpy.create_node("test_sim_node")
        self.js, self.js_count, self.odom, self.truth = None, 0, None, None
        self.node.create_subscription(JointState, "joint_states", self.on_js, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "odom/gait", lambda m: setattr(self, "odom", m),
                                      qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "sim/ground_truth", lambda m: setattr(self, "truth", m), 10)
        self.pub = self.node.create_publisher(CmdType, "cmd_vel", 10)

    def on_js(self, m):
        self.js = m
        self.js_count += 1

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.proc.terminate()
        self.proc.wait(timeout=10)

    def spin_for(self, seconds, vx=None):
        end = time.monotonic() + seconds
        nxt = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if vx is not None and time.monotonic() >= nxt:
                self.send(vx)
                nxt += 0.05

    def send(self, vx):
        m = CmdType()
        if STAMPED:
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x = vx
        else:
            m.linear.x = vx
        self.pub.publish(m)

    def stand_error(self):
        return max(abs(p - STAND[n.split("_")[1]]) for n, p in zip(self.js.name, self.js.position))

    def walk(self):
        """4 s at 0.05 m/s, one zero, 3 s to stand. Returns (odom dx, truth dx, truth dy)."""
        o0, t0 = self.odom.pose.pose.position, self.truth.pose.pose.position
        o0x, t0x, t0y = o0.x, t0.x, t0.y
        self.spin_for(4.0, vx=0.05)
        self.send(0.0)
        self.spin_for(3.0)
        o, t = self.odom.pose.pose.position, self.truth.pose.pose.position
        return o.x - o0x, t.x - t0x, t.y - t0y


def test_topics_and_walk():
    p = Probe(0.05)
    try:
        p.spin_for(3.0)
        assert p.js is not None and p.odom is not None and p.truth is not None, "sim node publishes nothing"
        c0 = p.js_count
        p.spin_for(4.0)
        rate = (p.js_count - c0) / 4.0
        assert 20 <= rate <= 30, f"joint_states at {rate:.1f} Hz (25 expected)"
        assert list(p.js.name)[0] == "rf_coxa" and list(p.js.name)[-1] == "lf_tibia" and len(p.js.name) == 18
        assert p.stand_error() < 0.01, f"not standing: {p.stand_error():.4f} rad"
        assert (p.odom.header.frame_id, p.odom.child_frame_id) == ("odom", "base_footprint")
        assert abs(time.time() - (p.js.header.stamp.sec + p.js.header.stamp.nanosec * 1e-9)) < 1.0
        odx, tdx, tdy = p.walk()
        assert abs(odx - 0.20) <= 0.012, f"/odom/gait +{odx:.3f} m for 0.05 m/s x 4 s"
        assert 0.85 <= tdx / odx <= 0.99, f"ground truth {tdx:.3f} m vs odom {odx:.3f} m (slip 0.05 + servo lag)"
        assert abs(tdy) < 0.005, f"ground truth drifted sideways {tdy:+.3f} m"
        assert p.stand_error() < 0.01, f"not standing 3 s after the zero: {p.stand_error():.4f} rad"
    finally:
        p.close()


def test_slip_parameter():
    p = Probe(0.5)
    try:
        p.spin_for(3.0)
        assert p.odom is not None and p.truth is not None
        odx, tdx, _ = p.walk()
        assert 0.40 <= tdx / odx <= 0.55, f"slip 0.5: ground truth {tdx:.3f} m vs odom {odx:.3f} m"
    finally:
        p.close()
