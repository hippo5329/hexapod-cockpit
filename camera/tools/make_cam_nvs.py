#!/usr/bin/env python3
"""Write the camera board's NVS partition image from a settings file.

    make_cam_nvs.py <settings file> <out.bin>

KEY=VALUE lines (# comments allowed), NVS namespace `hexacam` (camera/src/main.cpp):

    ssid, psk     the robot's Wi-Fi network and its key (2.4 GHz)
    ota_pw        the OTA password (none: no OTA)
    name          hostname (default hexacam)
    frame         qqvga | qvga | vga | svga | xga | hd   (default qvga)
    quality       JPEG quality 10-40, lower is better      (default 12)
    fps           1-30                                     (default 10)
    vflip, hmirror  1 | 0: flip the image for the mount    (default 0)
    syslog_ip, syslog_port  the syslog sink (port default 5140)
    xclk          the sensor clock, MHz 5-24                 (default 20)
    tx_dbm        Wi-Fi transmit power, dBm 2-20             (default 20; cheap boards lose frames at 20)
    lidar_rx      the GPIO wired to the LiDAR's TX (UART2)   (default -1: no forwarder)
    lidar_ip, lidar_port  the robot computer's ldlidar udp_server (port default 8889)
    lidar_baud    the LiDAR's baud                           (default 230400, LD19)

The file holds the Wi-Fi key and the OTA password: keep it private (0600) and out of any public
repository. The image goes to the `nvs` partition (0x9000, 0x5000 bytes). Needs the
esp-idf-nvs-partition-gen package (pip). Nothing from the file is printed.
"""
import csv
import ipaddress
import os
import subprocess
import sys
import tempfile

NVS_SIZE = 0x5000
KEYS = {"ssid", "psk", "ota_pw", "name", "frame", "quality", "fps", "vflip", "hmirror", "syslog_ip", "syslog_port",
        "xclk", "tx_dbm", "lidar_rx", "lidar_ip", "lidar_port", "lidar_baud"}
FRAMES = {"qqvga", "qvga", "vga", "svga", "xga", "hd"}


def load(path):
    vals = {}
    with open(path) as f:
        for n, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            key, sep, val = line.partition("=")
            key = key.strip()
            if not sep or key not in KEYS:
                sys.exit(f"{path}:{n}: not a known KEY=VALUE line")  # the line itself is not shown
            vals[key] = val.strip()
    if not vals.get("ssid"):
        sys.exit("ssid is needed")
    if len(vals["ssid"]) > 32 or not 8 <= len(vals.get("psk", "")) <= 63:
        sys.exit("ssid must be <= 32 characters and psk 8-63")
    if "frame" in vals and vals["frame"] not in FRAMES:
        sys.exit(f"frame must be one of {sorted(FRAMES)}")
    for key, lo, hi in (("quality", 10, 40), ("fps", 1, 30), ("syslog_port", 1, 65535), ("xclk", 5, 24)):
        if key in vals and not (vals[key].isdigit() and lo <= int(vals[key]) <= hi):
            sys.exit(f"{key} must be {lo}-{hi}")
    if "tx_dbm" in vals:
        try:
            ok = 2 <= float(vals["tx_dbm"]) <= 20
        except ValueError:
            ok = False
        if not ok:
            sys.exit("tx_dbm must be 2-20")
    for key in ("vflip", "hmirror"):
        if key in vals and vals[key] not in ("0", "1"):
            sys.exit(f"{key} must be 0 or 1")
    if "syslog_ip" in vals:
        try:
            ipaddress.IPv4Address(vals["syslog_ip"])
        except ValueError:
            sys.exit("syslog_ip must be an IPv4 address")
    if "name" in vals and not (vals["name"].replace("-", "").isalnum() and len(vals["name"]) <= 32):
        sys.exit("name must be letters, digits and '-', at most 32")
    return vals


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    vals = load(sys.argv[1])
    rows = [["key", "type", "encoding", "value"], ["hexacam", "namespace", "", ""]]
    rows += [[key, "data", "string", vals[key]] for key in sorted(vals)]
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
