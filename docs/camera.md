# Camera

The miniHexa's camera is a separate board: an ESP32-S3-CAM (ESP32-S3-WROOM N16R8: 16 MB flash, 8 MB octal
PSRAM; OV3660 sensor) on its own 5 V supply, joining the robot's Wi-Fi. It only captures and sends JPEG
frames; everything else happens on the host, where `camera_bridge` turns them into ROS images.

## Firmware (`camera/`)

    pio run -d camera                                   build (never `pio run -t upload`)
    camera/tools/make_cam_nvs.py <settings> nvs.bin     the board's settings as an NVS image

Flash with esptool over the board's native USB: `0x0 bootloader.bin`, `0x8000 partitions.bin`, `0x9000 nvs.bin`,
`0xe000 boot_app0.bin`, `0x10000 firmware.bin`, then `--after watchdog_reset`. Its RTS reset leaves the chip in
download mode, and opening the port with DTR/RTS asserted can do the same. Later updates go over Wi-Fi with
`firmware/tools/ota_upload.py` and the OTA password.

Settings (NVS namespace `hexacam`; the file carries the Wi-Fi key and the OTA password: keep it private):

| key | value |
|---|---|
| `ssid`, `psk` | the robot's network (2.4 GHz) |
| `ota_pw` | OTA password (port 3232); none: no OTA |
| `name` | hostname, default `hexacam` |
| `frame` | `qqvga`, `qvga` (default), `vga`, `svga`, `xga`, `hd` |
| `quality` | JPEG quality 10-40, lower is better (default 12) |
| `fps` | the stream's rate cap, 1-30 (default 10) |
| `vflip`, `hmirror` | `1`: flip the image for the mount |
| `syslog_ip`, `syslog_port` | the syslog sink (the robot firmware's module; port default 5140) |

HTTP:

| | |
|---|---|
| `:81/stream` | MJPEG (`multipart/x-mixed-replace`), each part with `Content-Length` and `X-Timestamp` (capture time, µs since the board's boot), paced to `fps`; one client at a time |
| `:80/capture` | one JPEG |
| `:80/status` | JSON: sensor, settings, the stream's measured rate and bandwidth, clients, RSSI, memory |
| `:80/control?vflip=0\|1&hmirror=0\|1` | flip the image for the mount: applied at once and kept in NVS (no USB needed); replies with the status |

The stream has its own server: a stream holds its handler for as long as the client reads. Modem sleep is
off (it adds 100+ ms to a frame). The log (syslog and the USB console): `boot`, the networks heard and every
Wi-Fi disconnect's reason, `wifi: up`, `stream` clients, a `health` line every 30 s.

Measured on the board: VGA at 24-28 fps from the sensor; on the robot's network QVGA quality 12 at 10 fps is
~5.5-6 kB a frame, ~0.5 Mbit/s.

## ROS (`hexapod_bringup`)

    ros2 launch hexapod_bringup camera.launch.py url:=http://<camera ip>:81/stream [camera_info_file:=<yaml>]

`camera_bridge` publishes each JPEG as it came (no decoding) on `camera/image_raw/compressed`
(`sensor_msgs/CompressedImage`, format `jpeg`, image_transport's compressed transport) and
`camera/camera_info` with the same stamp: from a camera_calibration YAML, or until there is one a nominal
pinhole from `hfov_deg` (58.4: fx 286 px at 320 px, measured from the checkerboard at 0.5 m and 1 m, the same within 1 %) and the JPEG's size. Stamps are the host's receive time less `latency_s`. A lost
stream is retried every `reconnect_s` (2 s); the rate, bandwidth and reconnects are logged every 10 s. For a
raw image: `ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=camera/image_raw/compressed -r out:=camera/image_raw`.

Calibration: print `camera/tools/make_checker_pdf.py out.pdf` at 100 % (check the 100 mm bar), glue it flat,
then either `camera/tools/calib_collect.py http://<camera ip>/capture <out dir>` (no display needed; it
logs what each kept view adds, move the board until the ranges are covered) or, with a display, republish
raw (above) and `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args -r image:=/camera/image_raw -r camera:=/camera`
(the measured square size in m). The YAML is `camera_info_file`; the launch default is
`config/hexacam1_qvga.yaml` (this robot, 320x240: fx 287.9 fy 285.8, cx 160.7 cy 131.7, k1 -0.111,
rms 0.37 px over 32 views). A calibration holds for its frame size only: at another size the bridge
warns and falls back to the nominal pinhole.

## Vision (`camera_vision`)

`ros2 run hexapod_bringup camera_vision --ros-args -p model_path:=<nanodet.onnx> -p depth_model_path:=<depth.onnx>`
reads the bridge's images, `camera_info` and `imu/data` and publishes `camera/detections`
(`vision_msgs/Detection2DArray`), `camera/vision` (JSON: objects with position and distance, and the free
floor ahead, left and right) and `camera/vision/image/compressed` (boxes, distances, obstacles tinted red).
The models are not in the repo:

- objects: NanoDet-Plus-m 416 from opencv_zoo (Apache-2.0),
  `https://huggingface.co/opencv/object_detection_nanodet/resolve/main/object_detection_nanodet_2022nov.onnx`
  (3.8 MB; COCO's 80 classes; OpenCV's dnn, ~30 ms on 4 cores)
- depth: Depth Anything V2 Small (Apache-2.0; the Base and Large models are not),
  `https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/main/onnx/model.onnx` (99 MB;
  onnxruntime, `pip install onnxruntime`; ~55 ms at 266x266 on 4 threads)

How far: the depth model's relative inverse depth is fitted to the floor, whose depth the geometry gives
(calibration, camera height, IMU tilt plus `mount_pitch_deg`); a point more than 3.5 cm above the floor
is an obstacle. Measured (2026-11-06): the checkerboard at 0.5 and 1.0 m within 5 %; walking, the
obstacle ahead agreed with the sonar within 2-5 cm under 0.7 m (0.513/0.508, 0.474/0.450, 0.507/0.505,
0.669/0.691 m). Beyond ~1.5 m it reads short (a closet 2.6 m away by sonar: 1.5-2.1 m): there 1 deg of
tilt is 0.5 m. The colour of the floor alone cannot separate it from white doors (Lab within 4), hence
the depth. The class names are COCO's: white closet doors come out as "refrigerator", furniture as
"chair", "bed" or "oven"; the obstacles do not depend on them.
