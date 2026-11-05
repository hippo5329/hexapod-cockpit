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
pinhole from `hfov_deg` (66) and the JPEG's size. Stamps are the host's receive time less `latency_s`. A lost
stream is retried every `reconnect_s` (2 s); the rate, bandwidth and reconnects are logged every 10 s. For a
raw image: `ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=camera/image_raw/compressed -r out:=camera/image_raw`.
