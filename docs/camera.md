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
| `:80/control?vflip=0\|1&hmirror=0\|1&fps=1-30&frame=<size>` | flip the image for the mount and pace the stream (applied at once), set the frame size (reboots: the buffers are sized at init); all kept in NVS (no USB needed); replies with the status. On the robot `fps=5`: the vision node uses 5, and the servos' load cuts the link to ~130-270 kbit/s (2026-11-06) |
| `:80/control?wbuf=1\|0` | where the Wi-Fi driver's and lwIP's buffers go: `1` (default) internal RAM, `0` PSRAM (the Arduino core's own choice). Kept in NVS (`wbuf`); the board reboots, because the buffers are taken when the driver starts. `/status` reports `wbuf`, how many requests went to each (`wbuf_internal`, `wbuf_psram`) and `heap_min`, the lowest free internal heap since boot |
| `POST :80/wifi` (form: `ssid`, `psk`, optional `syslog_ip`) | join another network without USB: kept and rebooted on trial; if the board has not joined within 60 s of the boot, the old ssid, psk and syslog_ip come back and it reboots. Send the body from a 0600 file (`curl --data-urlencode ssid@... --data-urlencode psk@...`), never argv |

The stream has its own server: a stream holds its handler for as long as the client reads. Modem sleep is
off (it adds 100+ ms to a frame). The log (syslog; `Serial` is UART0 on GPIO 43/44, not the USB port, which stays
for esptool): `boot`, the networks heard and every
Wi-Fi disconnect's reason, `wifi: up`, `stream` clients, a `health` line every 30 s.

**Wi-Fi buffers in internal RAM** (`src/wifi_mem.cpp`): the core's prebuilt IDF has
`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP`, so the Wi-Fi driver and lwIP ask for PSRAM first, where their frames share
the bus with the camera's DMA. The link wraps `heap_caps_{malloc,calloc,realloc}_prefer` (`-Wl,--wrap`): with `wbuf=1`
a request that prefers PSRAM tries internal RAM first and falls back to PSRAM only when internal RAM is out. Both
this and the USB console's removal were suspects in the link stalls on the robot (2026-11-06); the build is
**not yet run on a board** (OTA to hexacam1 failed on its bad link), so neither is measured.

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
tilt is 0.5 m. So `camera/obstacle_scan` reports only up to 1.0 m (`scan_range`) and feeds the local
costmap alone: at 1.2-1.5 m a ~2 deg tilt error lifted the floor past 4.5 cm, and every heading of a
look-around left an arc of phantom wall that fenced the robot in the global costmap (2026-11-08). The colour of the floor alone cannot separate it from white doors (Lab within 4), hence
the depth. The class names are COCO's: white closet doors come out as "refrigerator", furniture as
"chair", "bed" or "oven"; the obstacles do not depend on them.

### On a GPU

With a CUDA GPU, the detector and the depth can be larger models through PyTorch and transformers:

`ros2 run hexapod_bringup camera_vision --ros-args -p detector:=owlv2 -p depth_model_path:=hf:depth-anything/Depth-Anything-V2-Large-hf`

| | CPU (default) | GPU (RTX 5090, 320x240 frames) |
|---|---|---|
| objects | NanoDet-Plus, COCO's 80 classes, 37 ms | OWLv2 base, open vocabulary (`labels`, default `HOUSEHOLD`), 32 ms |
| depth | Depth Anything V2 Small at 266 px | V2 Large at 518 px, 38 ms |
| frame | 122 ms (3.8 Hz) | 72 ms (the camera's 5 fps) |

**Labels.** The open-vocabulary detector labels from a list of phrases. It names what COCO has no class for:
a slipper (score 0.22), folded towels, doors, cabinets and shelves. NanoDet found nothing in the same frames. Its
scores run lower than NanoDet's (keep 0.20 and up), and its mistakes are its own: a checkerboard came out as
"refrigerator" and a shelf as "bathtub".

**Depth.** On the two checkerboard frames, with the same floor fit, the boards (0.501 and 0.998 m away) read:

| model | 0.501 m board | 0.998 m board |
|---|---|---|
| Small at 266 (ONNX, CPU) | 0.595 | 1.862 |
| Small at 518 | 0.506 | 1.192 |
| Base | 0.522 | 1.182 |
| Large | 0.518 | 1.109 |

Grounding DINO (`detector:=grounding_dino`) takes 46 ms: its deformable attention runs in plain PyTorch, so it is
fed the frame at 480 px rather than 800, all its phrases in one prompt, at fp16. OWLv2 large takes 161 ms and found
the same labels.

Licences: OWLv2, Grounding DINO and Depth Anything V2 Small are Apache-2.0. **V2 Base and Large are
CC-BY-NC-4.0, for non-commercial use only**, so the default stays Small.

## Object survey while exploring (`object_survey`)

`ros2 run hexapod_bringup object_survey --ros-args -p note_prefix:=<path> [-p rooms_file:=<rooms.json> -p home_pose:=x,y,yaw_deg]`

The camera sees about 58 degrees, straight ahead, and while exploring it mostly sees what the frontiers point at.
So every 1.5 m of travel, and once after the start, the survey looks around:

1. It pauses the explorer: `explore/resume` false cancels its goal.
2. It turns a full circle in eight 45-degree steps with Nav2's Spin behaviour, which checks the costmap for room to turn.
3. At each heading it stands still for 2 s while `camera_vision` labels the frame.
4. It resumes the explorer, facing the way it was going.

Labels seen while walking count too. Each sighting within 2 m goes on the map through TF at the frame's stamp
(people and pets are skipped: they move). It merges by place: with the nearest object it has been named as within
0.5 m (0.9 m for furniture), else with the nearest object of any name within 0.35 m. An open-vocabulary detector
names one thing differently from frame to frame (a white bin: bathtub, basket, trash can), so the object's label
is the name with the most score summed over its sightings; the others are listed as "also seen as". An object is
confirmed after:
- sightings in two surveys, or
- sightings at two headings of the surveys with `confirm_score` (0.7; OWLv2 0.5) or more.

Walking sightings (every frame, often blurred) place and name an object but never confirm it alone: counting
frames confirmed 1104 "objects" in one 40-minute run, the label vote and the survey rule 101.

After every survey and at the end, the survey rewrites its note:
- `<note_prefix>.md`: the confirmed objects with their room and map position, then the objects room by room, then
  each survey.
- `<note_prefix>.json`: every object, confirmed or not, and the surveys.
- `<note_prefix>.png`: the map with the rooms, the objects and the path walked.
- `<note_prefix>_img/`: the labelled frame of each object's best sighting, and one frame of every survey heading
  (`survey<N>_h<k>.jpg`), to judge the detector's names by.
- `objects/markers`: for RViz.

Rooms are polygons in the **home frame**: home's position is the origin and its heading is +x. A map made fresh
at home each run keeps the rooms where they are. `home_pose` places home on the run's map. A TERM stops the
survey: it cancels a turn under way and writes the note. The labels are the detector's, with its mistakes (above): OWLv2 names a white wall or door that fills the frame
"bathtub" or "refrigerator" (`camera_vision` refuses those: a box touching three edges of the frame is a surface, and
a name whose real size at the measured distance is impossible, a 2.5 m "refrigerator" or a 0.3 m "bathtub", is dropped;
`refused` in `camera/vision` lists them with the reason); with OWLv2 pass `min_score:=0.2 confirm_score:=0.5`.
