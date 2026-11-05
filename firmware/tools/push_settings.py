#!/usr/bin/env python3
"""Change the miniHexa's robot settings over Wi-Fi, without USB.

    push_settings.py <board IP> <settings file> <password file> [espota.py]

The settings file has make_nvs.py's KEY=VALUE lines, but only the keys to change: the others
stay as they are in the board's NVS. An empty value (`trims=`) removes a key that has a
default (app, sim, port, trims, imu_level, idle_s, low_bat_mv, syslog_ip, syslog_port,
yaw_hold). Over Wi-Fi, link can only be wifi, and ssid, psk, agent and ota_pw cannot be removed: a robot reached
only over Wi-Fi would be lost.

It goes through ArduinoOTA (port 3232) with the OTA password, read from the password file's
first line (never the command line), into the board's unused `spiffs` partition. The board
reboots (the robot stands: its boot pose), checks every line, writes the keys to NVS and
erases the upload; a refused upload changes nothing (the console's `info` says why). A
change of link, ssid or psk is kept only if the board joins the network within 60 s of
that boot; if not, it puts the old values back and reboots. With a new ota_pw, the next
upload needs the new password. Nothing from the settings file is printed.
"""
import ipaddress
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ota_upload  # noqa: E402

MAGIC = "hexa-settings 1\n"
MAX = 4096
KEYS = {"app", "sim", "link", "ssid", "psk", "agent", "port", "ota_pw", "trims", "imu_level", "idle_s",
        "low_bat_mv", "syslog_ip", "syslog_port", "yaw_hold"}
REMOVABLE = {"app", "sim", "port", "trims", "imu_level", "idle_s", "low_bat_mv", "syslog_ip", "syslog_port",
             "yaw_hold"}


def load(path):
    """The lines to send, checked the way the board checks them (lib/hexa_settings)."""
    import make_nvs
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
            if key in vals:
                sys.exit(f"{path}:{n}: {key} given twice")
            vals[key] = val.strip()
    if not vals:
        sys.exit("no settings")
    for key, val in vals.items():
        if not val and key not in REMOVABLE:
            sys.exit(f"{key} cannot be removed over Wi-Fi")
    if vals.get("link") == "serial":
        sys.exit("link=serial would cut the Wi-Fi link: set it over USB (make_nvs.py)")
    if vals.get("agent"):
        try:
            ipaddress.IPv4Address(vals["agent"])
        except ValueError:
            sys.exit("agent must be an IPv4 address")
    if "psk" in vals and not 8 <= len(vals["psk"]) <= 63:
        sys.exit("psk must be 8-63 characters")
    # The value rules are make_nvs.py's: check what is set there (it exits with the reason).
    set_vals = {k: v for k, v in vals.items() if v}
    if set_vals.get("link") == "wifi":  # completeness is the board's check (with its NVS)
        set_vals.pop("link")
    if set_vals:
        old = os.umask(0o077)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                probe = os.path.join(tmp, "probe")
                with open(probe, "w") as f:
                    f.write("".join(f"{k}={v}\n" for k, v in set_vals.items()))
                make_nvs.load(probe)
        finally:
            os.umask(old)
    return vals


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit(__doc__)
    host, settings, pw_file = sys.argv[1:4]
    vals = load(settings)
    blob = (MAGIC + "".join(f"{k}={v}\n" for k, v in vals.items())).encode() + b"\0"
    if len(blob) > MAX:
        sys.exit(f"settings too long ({len(blob)} bytes, at most {MAX})")
    old = os.umask(0o077)
    try:
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "settings.bin")
            with open(path, "wb") as f:
                f.write(blob)
            rc = ota_upload.upload(host, path, pw_file, sys.argv[4] if len(sys.argv) == 5 else None, settings=True)
    finally:
        os.umask(old)
    print(f"settings {'sent' if rc == 0 else 'FAILED'} (rc {rc}): {', '.join(vals)}; the board reboots to apply them")
    sys.exit(rc)


if __name__ == "__main__":
    main()
