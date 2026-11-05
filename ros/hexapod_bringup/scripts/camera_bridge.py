#!/usr/bin/env python3
"""The miniHexa's camera on ROS: the camera board's MJPEG stream (camera/, hexacam) as compressed images.

The board sends JPEG frames over HTTP (multipart/x-mixed-replace, each part with Content-Length); this
node publishes each one as it is, no decoding:

  camera/image_raw/compressed   sensor_msgs/CompressedImage, format "jpeg" (image_transport's compressed)
  camera/camera_info            sensor_msgs/CameraInfo, the same stamp; from camera_info_file, else a
                                nominal pinhole from hfov_deg (uncalibrated: say so in the header's frame)

Stamps are the host's receive time less latency_s (the board has no ROS clock). A lost stream is
retried every reconnect_s; the rate, bandwidth and reconnects are logged every 10 s.

Parameters:
  url              http://10.72.1.21:81/stream
  frame_id         camera_optical_frame
  camera_info_file a camera_calibration YAML (empty: nominal)
  hfov_deg         66.0, the nominal horizontal field of view for the nominal intrinsics
  latency_s        0.0
  reconnect_s      2.0
"""
import math
import signal
import threading
import time
import urllib.request

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, CompressedImage


def jpeg_size(data):
    """(width, height) from a JPEG's start-of-frame marker, or None."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        seg = int.from_bytes(data[i + 2:i + 4], "big")
        if marker in (0xC0, 0xC1, 0xC2):
            return int.from_bytes(data[i + 7:i + 9], "big"), int.from_bytes(data[i + 5:i + 7], "big")
        i += 2 + seg
    return None


def nominal_info(width, height, hfov_deg):
    """A pinhole with the image centre and the focal length the field of view gives, no distortion."""
    m = CameraInfo()
    m.width, m.height = width, height
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    cx, cy = (width - 1) / 2, (height - 1) / 2
    m.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
    m.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.distortion_model = "plumb_bob"
    m.d = [0.0] * 5
    return m


def info_from_yaml(path):
    import yaml
    with open(path) as f:
        c = yaml.safe_load(f)
    m = CameraInfo()
    m.width, m.height = int(c["image_width"]), int(c["image_height"])
    m.distortion_model = c.get("distortion_model", "plumb_bob")
    m.d = [float(v) for v in c["distortion_coefficients"]["data"]]
    m.k = [float(v) for v in c["camera_matrix"]["data"]]
    m.r = [float(v) for v in c["rectification_matrix"]["data"]]
    m.p = [float(v) for v in c["projection_matrix"]["data"]]
    return m


def frames(stream):
    """JPEG payloads from a multipart stream whose parts carry Content-Length."""
    while True:
        length = None
        line = stream.readline()
        if not line:
            return
        while line.strip() == b"" or line.startswith(b"--"):  # the boundary and the blank line before it
            line = stream.readline()
            if not line:
                return
        while line.strip():  # the part's headers
            k, _, v = line.decode("latin-1").partition(":")
            if k.strip().lower() == "content-length":
                length = int(v.strip())
            line = stream.readline()
            if not line:
                return
        if length is None:
            raise ValueError("a part without Content-Length")
        data = stream.read(length)
        if len(data) < length:
            return
        yield data


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge")
        self.url = self.declare_parameter("url", "http://10.72.1.21:81/stream").value
        self.frame_id = self.declare_parameter("frame_id", "camera_optical_frame").value
        info_file = self.declare_parameter("camera_info_file", "").value
        self.hfov = float(self.declare_parameter("hfov_deg", 66.0).value)
        self.latency = float(self.declare_parameter("latency_s", 0.0).value)
        self.reconnect = float(self.declare_parameter("reconnect_s", 2.0).value)
        self.info = info_from_yaml(info_file) if info_file else None
        self.calibrated = bool(info_file)
        self.img_pub = self.create_publisher(CompressedImage, "camera/image_raw/compressed", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "camera/camera_info", qos_profile_sensor_data)
        self.n = self.bytes = self.reconnects = 0
        self.running = True
        self.create_timer(10.0, self.report)
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def report(self):
        self.get_logger().info(f"{self.n / 10.0:.1f} fps  {self.bytes * 8 / 10000.0:.0f} kbit/s  reconnects {self.reconnects}")
        self.n = self.bytes = 0

    def publish(self, data):
        stamp = (self.get_clock().now() - Duration(seconds=self.latency)).to_msg()
        img = CompressedImage()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.format = "jpeg"
        img.data = data
        self.img_pub.publish(img)
        size = jpeg_size(data)
        if self.info is None or (not self.calibrated and size and (self.info.width, self.info.height) != size):
            if not size:
                return
            self.info = nominal_info(size[0], size[1], self.hfov)
            self.get_logger().info(f"image {size[0]}x{size[1]}: nominal intrinsics from hfov {self.hfov:.0f} deg (uncalibrated)")
        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.info_pub.publish(self.info)
        self.n += 1
        self.bytes += len(data)

    def read_loop(self):
        while self.running and rclpy.ok():
            try:
                with urllib.request.urlopen(self.url, timeout=5) as r:
                    self.get_logger().info(f"stream {self.url}: connected")
                    for data in frames(r):
                        if not self.running:
                            return
                        self.publish(data)
            except Exception as e:  # the board rebooting, Wi-Fi gone, a timeout: try again
                if self.running:
                    self.get_logger().warning(f"stream {self.url}: {e.__class__.__name__}: {e}")
            if self.running:
                self.reconnects += 1
                time.sleep(self.reconnect)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CameraBridge()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
