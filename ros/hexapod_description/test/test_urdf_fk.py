"""The URDF and the firmware must agree on where every foot and sensor is.

Expands the xacro, walks base_link -> <leg>_foot by the URDF's own joint origins and axes,
and compares with the firmware's hexa_kin leg_fk (compiled here from firmware/lib) at
random joint angles inside the servo range. A length or an axis changed in one and not
the other fails this.
"""
import math
import os
import random
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
FW_LIB = os.environ.get("HEXA_FIRMWARE_LIB", os.path.join(HERE, "..", "..", "..", "firmware", "lib"))
XACRO = os.environ.get("HEXA_XACRO", os.path.join(HERE, "..", "urdf", "minihexa.urdf.xacro"))
STAND = os.path.join(HERE, "..", "config", "stand_pose.yaml")
LEGS = ["rf", "rm", "rr", "lr", "lm", "lf"]  # hexa_kin order


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rpy_xyz(rpy, xyz):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0, 0, 0, 1],
    ]


def axis_angle(axis, q):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
        [0, 0, 0, 1],
    ]


def floats(s, default):
    return [float(v) for v in s.split()] if s else default


@pytest.fixture(scope="module")
def urdf():
    xml = subprocess.run(["xacro", XACRO], check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(xml)
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        joints[j.find("child").get("link")] = {
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "T": rpy_xyz(floats(o.get("rpy") if o is not None else None, [0, 0, 0]),
                         floats(o.get("xyz") if o is not None else None, [0, 0, 0])),
            "axis": floats(a.get("xyz") if a is not None else None, [1, 0, 0]),
            "lower": float(lim.get("lower")) if lim is not None else None,
            "upper": float(lim.get("upper")) if lim is not None else None,
        }
    return joints


def urdf_foot(joints, leg, q):
    """Foot position in base_link for joint values q = {joint name: angle}."""
    link = f"{leg}_foot"
    chain = []
    while link != "base_link":
        j = joints[link]
        chain.append(j)
        link = j["parent"]
    T = [[1 if i == k else 0 for k in range(4)] for i in range(4)]
    for j in reversed(chain):
        T = matmul(T, j["T"])
        if j["type"] == "revolute":
            T = matmul(T, axis_angle(j["axis"], q[j["name"]]))
    return T[0][3], T[1][3], T[2][3]


@pytest.fixture(scope="module")
def fk_dump():
    with tempfile.TemporaryDirectory() as d:
        exe = os.path.join(d, "fk_dump")
        subprocess.run(["g++", "-std=c++17", "-O1", "-I", os.path.join(FW_LIB, "hexa_kin"),
                        os.path.join(HERE, "fk_dump.cpp"), os.path.join(FW_LIB, "hexa_kin", "hexa_kin.cpp"),
                        "-o", exe], check=True)
        yield exe


def test_joint_names_match_firmware(urdf):
    names = {j["name"] for j in urdf.values() if j["type"] == "revolute"}
    expected = {f"{leg}_{part}" for leg in LEGS for part in ("coxa", "femur", "tibia")}
    assert names == expected


def test_urdf_fk_matches_firmware(urdf, fk_dump):
    rng = random.Random(5329)
    cases = []
    for _ in range(300):
        leg = rng.randrange(6)
        q = {}
        for part in ("coxa", "femur", "tibia"):
            j = next(v for v in urdf.values() if v["name"] == f"{LEGS[leg]}_{part}")
            q[j["name"]] = rng.uniform(j["lower"], j["upper"])
        cases.append((leg, q))
    stdin = "".join(f"F {leg} {q[f'{LEGS[leg]}_coxa']} {q[f'{LEGS[leg]}_femur']} {q[f'{LEGS[leg]}_tibia']}\n"
                    for leg, q in cases)
    out = subprocess.run([fk_dump], input=stdin, check=True, capture_output=True, text=True).stdout.split("\n")
    worst = 0.0
    for (leg, q), line in zip(cases, out):
        fw = [float(v) for v in line.split()]
        ur = urdf_foot(urdf, LEGS[leg], q)
        worst = max(worst, max(abs(a - b) for a, b in zip(fw, ur)))
    assert worst < 1e-5, f"URDF and firmware FK differ by {worst * 1000:.3f} mm"


def test_servo_range_matches_urdf_limits(urdf):
    # The usable ranges inside the servos' 280 deg (hexa_servo default_cal, centred at coxa 0,
    # femur 0, tibia -90 deg): femur +-90 (the vendor's clamp), coxa +-90, tibia folded 0..180.
    for leg in LEGS:
        lim = {p: next(v for v in urdf.values() if v["name"] == f"{leg}_{p}") for p in ("coxa", "femur", "tibia")}
        assert lim["coxa"]["lower"] == pytest.approx(-math.pi / 2) and lim["coxa"]["upper"] == pytest.approx(math.pi / 2)
        assert lim["femur"]["lower"] == pytest.approx(-math.pi / 2) and lim["femur"]["upper"] == pytest.approx(math.pi / 2)
        assert lim["tibia"]["lower"] == pytest.approx(-math.pi) and lim["tibia"]["upper"] == pytest.approx(0)


def test_stand_pose_is_the_gaits_neutral_stance(urdf, fk_dump):
    with open(STAND) as f:
        zeros = yaml.safe_load(f)["joint_state_publisher"]["ros__parameters"]["zeros"]
    assert set(zeros) == {f"{leg}_{p}" for leg in LEGS for p in ("coxa", "femur", "tibia")}
    stdin = "".join(f"N {i}\n" for i in range(6))
    out = subprocess.run([fk_dump], input=stdin, check=True, capture_output=True, text=True).stdout.split("\n")
    for i, leg in enumerate(LEGS):
        neutral = [float(v) for v in out[i].split()]
        foot = urdf_foot(urdf, leg, zeros)
        assert max(abs(a - b) for a, b in zip(neutral, foot)) < 1e-5, f"{leg}: stand pose foot {foot} != {neutral}"


def test_sensor_frames_match_the_simulated_sensors(urdf):
    """imu_link at the body centre, level; sonar_link where hexa_sensors' range model looks from."""
    src = r"""
#include <cstdio>
#include "hexa_sensors.h"
int main() { hexa::RangeParams p; std::printf("%.7f %.7f %.7f\n", p.x, p.y, p.yaw); }
"""
    with tempfile.TemporaryDirectory() as d:
        c, exe = os.path.join(d, "s.cpp"), os.path.join(d, "s")
        with open(c, "w") as f:
            f.write(src)
        inc = [a for lib in ("hexa_sensors", "hexa_world", "hexa_gait", "hexa_kin", "hexa_servo")
               for a in ("-I", os.path.join(FW_LIB, lib))]
        subprocess.run(["g++", "-std=c++17", *inc, c, "-o", exe], check=True)
        sx, sy, syaw = (float(v) for v in subprocess.run([exe], check=True, capture_output=True,
                                                          text=True).stdout.split())
    imu, sonar, base = urdf["imu_link"], urdf["sonar_link"], urdf["base_link"]
    assert imu["parent"] == "base_link" and sonar["parent"] == "base_link" and base["parent"] == "base_footprint"
    assert imu["T"] == rpy_xyz([0, 0, 0], [0, 0, 0])
    # sonar in base_footprint = base_footprint -> base_link -> sonar_link
    T = matmul(base["T"], sonar["T"])
    assert T[0][3] == pytest.approx(sx, abs=1e-6) and T[1][3] == pytest.approx(sy, abs=1e-6)
    assert math.atan2(T[1][0], T[0][0]) == pytest.approx(syaw, abs=1e-6)
    assert T[2][3] > 0.05  # above the floor, at the front of the body


def test_camera_frame_is_the_measured_mount(urdf):
    """camera_link 0.065 m ahead, 0.005 m right, 0.11 m above the body's underside; the optical frame looks along +x."""
    cam, opt = urdf["camera_link"], urdf["camera_optical_frame"]
    assert cam["parent"] == "base_link" and opt["parent"] == "camera_link"
    assert [cam["T"][i][3] for i in range(3)] == pytest.approx([0.065, -0.005, 0.110 - 0.025 / 2], abs=1e-9)
    T = matmul(cam["T"], opt["T"])
    # optical z (the view) is body +x, optical x (image right) is body -y, optical y (image down) is body -z
    assert [T[i][2] for i in range(3)] == pytest.approx([1, 0, 0], abs=1e-9)
    assert [T[i][0] for i in range(3)] == pytest.approx([0, -1, 0], abs=1e-9)
    assert [T[i][1] for i in range(3)] == pytest.approx([0, 0, -1], abs=1e-9)
