#!/usr/bin/env python3
"""Write the miniHexa's NVS partition image from a robot settings file.

    make_nvs.py <settings file> <out.bin>

The settings file holds KEY=VALUE lines (# comments allowed); it carries the Wi-Fi key and
the OTA password, so keep it private (mode 0600) and out of any public repository. Keys:

    app      base | console                    (default base)
    sim      1 | 0                             (default 1; real mode is refused until the board is verified)
    link     serial | wifi                     (default serial)
    ssid     Wi-Fi network (2.4 GHz)           psk      its key
    agent    micro-ROS agent IPv4 address      port     its UDP port (default 8888)
    ota_pw   OTA password (none: no OTA)
    trims    18 comma-separated servo trims in us (by servo id 1-18, each within +-200): the
             vendor's "deviation" values, read off the robot before its firmware is replaced
    idle_s   seconds without a command that moves before the robot lies down (default 30; 0: never)
    low_bat_mv  the final shutdown's battery floor in mV: under it for 30 s with the robot still,
             it lies down and the ESP32 sleeps (default 6400; 0: never; else 5600-8000)
    imu_level  roll,pitch of the IMU on its board, degrees (each within +-10), read with the body
             level (a level on the top plate) from the console's `imu`: every sample is turned
             into the body frame

The image goes to the `nvs` partition (0x9000, 0x5000 bytes) with the firmware, so every
flash writes the whole robot description fresh. OTA updates leave it in place. Needs the
esp-idf-nvs-partition-gen package (pip). Nothing from the file is printed.
"""
import csv
import ipaddress
import os
import subprocess
import sys
import tempfile

NVS_SIZE = 0x5000
STRINGS = {"app", "link", "ssid", "psk", "agent", "ota_pw", "trims", "imu_level", "idle_s", "low_bat_mv"}
CHOICES = {"app": {"base", "console"}, "link": {"serial", "wifi"}, "sim": {"0", "1"}}


def load(path):
    vals = {}
    with open(path) as f:
        for n, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            key, sep, val = line.partition("=")
            key = key.strip()
            if not sep or key not in STRINGS | {"sim", "port"}:
                sys.exit(f"{path}:{n}: not a known KEY=VALUE line")  # the line itself is not shown
            vals[key] = val.strip()
    for key, allowed in CHOICES.items():
        if key in vals and vals[key] not in allowed:
            sys.exit(f"{key} must be one of {sorted(allowed)}")
    if "port" in vals and not (vals["port"].isdigit() and 0 < int(vals["port"]) < 65536):
        sys.exit("port must be 1-65535")
    if vals.get("link") == "wifi":
        if not vals.get("ssid"):
            sys.exit("link=wifi needs ssid")
        try:
            ipaddress.IPv4Address(vals.get("agent", ""))
        except ValueError:
            sys.exit("link=wifi needs agent as an IPv4 address")
    if "idle_s" in vals and not (vals["idle_s"].isdigit() and int(vals["idle_s"]) <= 3600):
        sys.exit("idle_s must be 0-3600 (seconds)")
    if "low_bat_mv" in vals and not (vals["low_bat_mv"].isdigit() and
                                     (int(vals["low_bat_mv"]) == 0 or 5600 <= int(vals["low_bat_mv"]) <= 8000)):
        sys.exit("low_bat_mv must be 0 or 5600-8000 (mV)")
    if "imu_level" in vals:
        f = [t.strip() for t in vals["imu_level"].split(",")]
        try:
            ok = len(f) == 2 and all(abs(float(t)) <= 10 for t in f)
        except ValueError:
            ok = False
        if not ok:
            sys.exit("imu_level must be roll,pitch in degrees, each within +-10")
        vals["imu_level"] = ",".join("%.2f" % float(t) for t in f)
    if "trims" in vals:
        f = [t.strip() for t in vals["trims"].split(",")]
        if len(f) != 18 or not all(t.lstrip("+-").isdigit() and abs(int(t)) <= 200 for t in f):
            sys.exit("trims must be 18 comma-separated integers within +-200")
        vals["trims"] = ",".join(str(int(t)) for t in f)
    for key in ("ssid", "psk"):
        if len(vals.get(key, "")) > {"ssid": 32, "psk": 63}[key]:
            sys.exit(f"{key} is too long")
    return vals


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    vals = load(sys.argv[1])
    rows = [["key", "type", "encoding", "value"], ["hexa", "namespace", "", ""]]
    for key in sorted(vals):
        if key == "sim":
            rows.append([key, "data", "u8", vals[key]])  # Preferences stores a bool as u8
        elif key == "port":
            rows.append([key, "data", "u16", vals[key]])
        else:
            rows.append([key, "data", "string", vals[key]])
    old = os.umask(0o077)
    try:
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "nvs.csv")
            with open(src, "w", newline="") as f:
                csv.writer(f).writerows(rows)
            r = subprocess.run([sys.executable, "-m", "esp_idf_nvs_partition_gen", "generate", src, sys.argv[2],
                                hex(NVS_SIZE)], capture_output=True, text=True)
            if r.returncode != 0:
                sys.exit("nvs_partition_gen failed (rc %d)" % r.returncode)
    finally:
        os.umask(old)
    os.chmod(sys.argv[2], 0o600)
    print(f"{sys.argv[2]}: {len(vals)} keys ({', '.join(sorted(vals))}), {NVS_SIZE} bytes")


if __name__ == "__main__":
    main()
