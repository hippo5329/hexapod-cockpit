"""camera_bridge: an MJPEG stream (the camera board's form: chunked multipart, Content-Length per part)
becomes camera/image_raw/compressed with the JPEG bytes as they came, and camera/camera_info with the
stamp and the JPEG's size; a dropped stream is reconnected."""
import http.server
import os
import socketserver
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

NODE = os.environ["HEXA_CAMERA_BRIDGE"]


def fake_jpeg(w, h, tag):
    """SOI, a start-of-frame with the size, a little payload, EOI: enough for the bridge, which never decodes."""
    sof = b"\xff\xc0\x00\x11\x08" + h.to_bytes(2, "big") + w.to_bytes(2, "big") + b"\x03\x01\x22\x00\x02\x11\x01\x03\x11\x01"
    return b"\xff\xd8" + sof + b"\xff\xda" + tag + b"\xff\xd9"


class Stream(http.server.BaseHTTPRequestHandler):
    frames_per_connection = 30

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace;boundary=hexacamframe")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()
        try:
            for i in range(self.frames_per_connection):
                jpg = fake_jpeg(320, 240, b"frame%03d" % i)
                part = (b"\r\n--hexacamframe\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\nX-Timestamp: %d\r\n\r\n"
                        % (len(jpg), i * 100000)) + jpg
                self.wfile.write(b"%x\r\n" % len(part) + part + b"\r\n")
                self.wfile.flush()
                time.sleep(0.05)
            self.wfile.write(b"0\r\n\r\n")  # the board rebooting: the stream ends
        except BrokenPipeError:
            pass

    def log_message(self, *a):
        pass


def test_stream_becomes_compressed_images_and_info():
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", 0), Stream)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{srv.server_address[1]}/stream"
    proc = subprocess.Popen([sys.executable, NODE, "--ros-args", "-p", f"url:={url}", "-p", "reconnect_s:=0.5"])
    rclpy.init()
    node = rclpy.create_node("test_camera_bridge")
    imgs, infos = [], []
    node.create_subscription(CompressedImage, "camera/image_raw/compressed", imgs.append, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "camera/camera_info", infos.append, qos_profile_sensor_data)
    try:
        end = time.monotonic() + 20
        # 30 frames a connection: more than 40 means it reconnected after the stream ended.
        while time.monotonic() < end and len(imgs) < 45:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(imgs) >= 45, f"{len(imgs)} images: no reconnect after the stream ended?"
        tags = [bytes(m.data)[2 + 19 + 2:-2] for m in imgs]
        assert all(t.startswith(b"frame") for t in tags)
        assert bytes(imgs[0].data) == fake_jpeg(320, 240, tags[0]), "the JPEG bytes as they came"
        assert imgs[0].format == "jpeg" and imgs[0].header.frame_id == "camera_optical_frame"
        assert infos and infos[0].width == 320 and infos[0].height == 240
        stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in imgs}
        assert any((i.header.stamp.sec, i.header.stamp.nanosec) in stamps for i in infos), "info shares the image's stamp"
        f = infos[0].k[0]
        assert abs(f - 160 / 0.6494075932) < 0.5, f"nominal focal length for 66 deg over 320 px: {f}"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        proc.wait(timeout=10)
        srv.shutdown()
