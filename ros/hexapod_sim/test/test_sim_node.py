"""hexapod_sim_node stands in for the firmware: same topics, rates and gait, plus ground truth.

Walks 0.05 m/s for 4 s, then one zero, as the board check does: /odom/gait gains the command,
/sim/ground_truth trails it by the slip, and the robot stands again. The simulated IMU and
ultrasonic publish what the models make, declare that noise, and see the room; the board's
AHRS gives /imu/data the sensor's tilt and follows a turn.
"""
import math
import os
import subprocess
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, Range
from visualization_msgs.msg import Marker

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
        self.imus, self.ranges, self.walls = [], [], None
        self.node.create_subscription(JointState, "joint_states", self.on_js, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "odom/gait", lambda m: setattr(self, "odom", m),
                                      qos_profile_sensor_data)
        self.node.create_subscription(Odometry, "sim/ground_truth", lambda m: setattr(self, "truth", m),
                                      qos_profile_sensor_data)
        self.node.create_subscription(Imu, "imu/data", self.imus.append, qos_profile_sensor_data)
        self.node.create_subscription(Range, "sonar", self.ranges.append, qos_profile_sensor_data)
        self.node.create_subscription(Marker, "sim/walls", lambda m: setattr(self, "walls", m),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.node.create_publisher(CmdType, "cmd_vel", 10)

    def on_js(self, m):
        self.js = m
        self.js_count += 1

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.proc.terminate()
        self.proc.wait(timeout=10)

    def spin_for(self, seconds, vx=None, wz=0.0):
        end = time.monotonic() + seconds
        nxt = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if vx is not None and time.monotonic() >= nxt:
                self.send(vx, wz)
                nxt += 0.05

    def send(self, vx, wz=0.0):
        m = CmdType()
        if STAMPED:
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.twist.linear.x = vx
            m.twist.angular.z = wz
        else:
            m.linear.x = vx
            m.angular.z = wz
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


def test_imu_and_sonar():
    p = Probe(0.05)
    try:
        p.spin_for(3.0)
        assert p.imus and p.ranges, "no /imu/data or /sonar"
        assert p.walls is not None and len(p.walls.points) == 8, "no latched /sim/walls (4 segments)"
        n_imu, n_range = len(p.imus), len(p.ranges)
        p.spin_for(4.0)
        imu_rate, range_rate = (len(p.imus) - n_imu) / 4.0, (len(p.ranges) - n_range) / 4.0
        assert 45 <= imu_rate <= 55, f"/imu/data at {imu_rate:.1f} Hz (50 expected)"
        assert 8 <= range_rate <= 12, f"/sonar at {range_rate:.1f} Hz (10 expected)"
        m = p.imus[-1]
        assert m.header.frame_id == "imu_link"
        assert m.linear_acceleration_covariance[0] == pytest.approx(8e-5, rel=1e-3)  # QMI8658, declared = made
        assert m.angular_velocity_covariance[8] == pytest.approx(2e-6, rel=1e-3)
        still = p.imus[n_imu:]
        az = sum(i.linear_acceleration.z for i in still) / len(still)
        ax = sum(i.linear_acceleration.x for i in still) / len(still)
        gz = sum(i.angular_velocity.z for i in still) / len(still)
        assert 9.80 <= az <= 9.98, f"standing az {az:.3f}"
        assert 0.2 <= ax <= 0.3, f"standing ax {ax:.3f}: the 1.5 deg mount pitch must leak gravity"
        assert abs(gz) < 0.005, f"standing gz {gz:.4f}"
        r = p.ranges[-1]
        assert r.header.frame_id == "sonar_link" and r.radiation_type == Range.ULTRASOUND
        assert r.range == pytest.approx(0.92, abs=0.01), f"sonar {r.range:.3f} m from the room centre (0.92)"
        _, tdx, _ = p.walk()
        expect = 1.0 - 0.08 - p.truth.pose.pose.position.x
        assert p.ranges[-1].range == pytest.approx(expect, abs=0.01), \
            f"sonar {p.ranges[-1].range:.3f} m after the walk, wall at {expect:.3f} m by ground truth"
    finally:
        p.close()


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def yaw_of(o):
    return 2 * math.atan2(o.pose.pose.orientation.z, o.pose.pose.orientation.w)


def test_ahrs_orientation():
    p = Probe(0.05)
    try:
        p.spin_for(3.0)
        assert p.imus and p.truth is not None, "no /imu/data or /sim/ground_truth"
        m = p.imus[-1]
        q = m.orientation
        assert abs(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z - 1) < 1e-4, "not a unit quaternion"
        # Roll and pitch declared at the filter's measured accuracy; yaw unanchored.
        assert m.orientation_covariance[0] == pytest.approx(1.22e-5, rel=1e-3)
        assert m.orientation_covariance[4] == pytest.approx(1.22e-5, rel=1e-3)
        assert m.orientation_covariance[8] >= 1e5, "yaw has no magnetometer: it must not be declared absolute"
        # Standing: the sensor's own tilt, the mount's (roll +1.0, pitch -1.5 deg).
        n = len(p.imus)
        p.spin_for(2.0)
        tilts = [rpy(i.orientation) for i in p.imus[n:]]
        roll = math.degrees(sum(t[0] for t in tilts) / len(tilts))
        pitch = math.degrees(sum(t[1] for t in tilts) / len(tilts))
        assert roll == pytest.approx(1.0, abs=0.3) and pitch == pytest.approx(-1.5, abs=0.3), \
            f"standing roll {roll:+.2f} pitch {pitch:+.2f} deg, the mount is +1.0 / -1.5"
        # A 3 s turn: the AHRS yaw follows the true yaw (the gyro alone: 1 % scale + bias).
        y0, t0 = rpy(p.imus[-1].orientation)[2], yaw_of(p.truth)
        p.spin_for(3.0, vx=0.0, wz=0.4)
        p.send(0.0)
        p.spin_for(2.0)
        dyaw, dtruth = rpy(p.imus[-1].orientation)[2] - y0, yaw_of(p.truth) - t0
        assert dtruth > 0.6, f"the robot did not turn: truth {dtruth:.3f} rad"
        assert abs(dyaw - dtruth) < 0.05, f"AHRS turned {dyaw:.3f} rad, ground truth {dtruth:.3f} rad"
        r, pt, _ = rpy(p.imus[-1].orientation)
        assert math.degrees(r) == pytest.approx(1.0, abs=0.5) and math.degrees(pt) == pytest.approx(-1.5, abs=0.5)
    finally:
        p.close()
