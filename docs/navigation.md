# LiDAR, SLAM, Nav2 and exploration

The miniHexa maps and explores with linorobot2-cockpit's stack: its LD19 driver, slam_toolbox, Nav2 and
explore_lite, run from the cockpit's launchers with this robot's config,
[`ros/hexapod_bringup/config/minihexa_cockpit.yaml`](../ros/hexapod_bringup/config/minihexa_cockpit.yaml).
The Standard Kit has no LiDAR: an LD19 is added on top, and the camera board forwards its data over Wi-Fi.

```
LD19 ──TX──> camera board GPIO 14 ──UDP :8889──> ldlidar (udp_server) ──/scan──> slam_toolbox ──/map──┐
robot board ── micro-ROS (UDP :8888) ──> /odom/gait, /imu/data ──> EKF ──/odom──> Nav2 <── explore_lite ┘
                                         /sonar ────────────────────────────────> collision monitor
```

## The LiDAR

**Wiring.** The LD19's TX goes to the camera board's GPIO 14 (marked G14), its 5 V and GND to the camera
board's supply. The LiDAR's PWM line can go to a free GPIO (`lidar_pwm`), but it is optional: left floating
or low, the LD19 runs its own speed control at 10 Hz.

**Forwarder** (`camera/src/lidar_fwd.*`, ported from linorobot2-cockpit's firmware). It reads the LiDAR on UART2
and sends the raw bytes, more than 128 at a time and not aligned to packets, as UDP datagrams to the robot
computer. It runs in a task of its own, started once the network is up. While the radio is down or an OTA runs,
the bytes are dropped. Settings in the camera's NVS (`/control?...`, kept, the board reboots):

| key | value |
|---|---|
| `lidar_rx` | the GPIO wired to the LiDAR's TX (14 on this robot); `-1` (default): off |
| `lidar_ip` | the robot computer, where the ldlidar node listens; none: off |
| `lidar_port` | its UDP port, default 8889 (cockpit's `lidar.udp_port`) |
| `lidar_baud` | default 230400 (LD19) |
| `lidar_pwm`, `lidar_duty`, `lidar_pwm_hz` | the motor's PWM pin, duty in % or `auto` (default: the pin released), 20-50 kHz (default 30 kHz). `/control?lidar_motor=0\|1` stops or runs the motor until the next boot |

`/status` reports the bytes, packets and receive errors, and the scan rate read back from the packets (each
LD19 packet carries the motor's speed). On the robot's LD19, no PWM duty or frequency (0-100 %, 20-50 kHz, from
power-on or later) changed its 9.9 Hz: to turn it off, switch its 5 V instead.

**The camera's clock matters.** With some sensor clocks this board's Wi-Fi uplink leaves only at the access
point's beacons (pings in a ~100 ms sawtooth, 30-58 % lost), and LiDAR datagrams are dropped in the transmit
queue. 12 MHz was clean on every install tried; it is the default (`/control?xclk=5-24`, kept).

**The driver** runs on the robot computer: cockpit's `ldlidar_stl_ros2` in `udp_server` mode, with the
parameters cockpit's bringup gives it:

```
ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args -r __node:=ld19 \
  -p product_name:=LDLiDAR_LD19 -p topic_name:=scan -p frame_id:=laser_link \
  -p comm_mode:=udp_server -p server_ip:=0.0.0.0 -p server_port:=8889 \
  -p laser_scan_dir:=true -p bins:=456 -p pub_rate:=50.0 -p enable_angle_crop_func:=false
```

`pub_rate` 50, not the driver's default 10: it polls for a finished revolution at that rate, and at 10 Hz a
10 Hz LiDAR loses a scan whenever the two clocks are in phase.

**Mount.** `laser_link` in the URDF: 0.005 m behind the body centre, 0.145 m above the body's underside, yaw 0
(the LD19's 0° ahead), as measured on the robot and confirmed by SLAM.

**Measured** (2026-11-07): 21 KB/s, 0 receive errors; `/scan` 9.6-9.9 Hz; cockpit's `lidar_check` passes with
no returns from the robot itself.

## SLAM

```
ros2 launch hexapod_bringup view_robot.launch.py rviz:=false      # URDF, EKF: odom -> base_footprint
ros2 launch <cockpit>/launchers/slam.launch.py config_file:=<path>/minihexa_cockpit.yaml
```

beside the robot's micro-ROS agent (`micro_ros_agent udp4 -p 8888`) and the LiDAR driver. The `slam` block
is cockpit's reference block with the hexapod's frame and pace:

| | | why |
|---|---|---|
| `base_frame` | `base_footprint` | the top of this robot's tree (`odom` → `base_footprint` → `base_link`) |
| `minimum_travel_distance`, `minimum_travel_heading` | 0.1 m, 0.15 rad | it walks slowly |
| `max_laser_range` | 8 m | the LD19 indoors |
| `map_update_interval` | 2 s | |

The map is made fresh at the start of each run, where the robot stands: that spot is the map's origin.

## Nav2

```
ros2 launch <cockpit>/launchers/nav2.launch.py config_file:=<path>/minihexa_cockpit.yaml
```

The `nav2` block is cockpit's reference block with what a walker changes:

- **Frames** `base_footprint`; **controller and smoother at 10 Hz**: a 100 Hz `/cmd_vel` gave the gait late ticks.
- **MPPI with the Omni model** (vx, vy, wz): the hexapod walks in any direction, so it can back off and side-step
  a wall instead of turning on the spot. A rotation shim with pure pursuit, a pair made for differential drive,
  stood at walls: the collision monitor scaled the turn to nothing and reversing was off.
- **Limits** 0.20 m/s each way and 1.2 rad/s: the gait's top speed is 214 mm/s at its 0.4 s cycle and its top turn
  1.6 rad/s; 1.2 keeps it off its clamp. Slower requests stretch the gait's cycle ([firmware.md](firmware.md#gait)).
- **Scaled MPPI**: 2.5 s horizon (25 steps of 0.1 s), 600 rollouts (1000 ran the controller at 7-8 Hz of its 10),
  noise 0.05 m/s, temperature 0.05, gamma 0.00125. With the reference's weights the control cost held every
  command near zero (0.011-0.020 m/s); with 4 s and the goal critics from 0.5 m, a full-speed rollout overshot
  12 cm goals. `ay_min` is set to −0.2 like `ax_min`: the default −3.0 averaged side-steps to 0.011 m/s.
- **Head forward**: `PathAngleCritic` and `PreferForwardCritic` make facing the path the cheap way, because the
  camera looks ahead; sideways and backwards stay for corrections and backing off.
- **Robot radius** 0.17 m (the frame is 240 × 234 mm), inflation 0.45 m with `cost_scaling_factor` 10: at 3, the
  middle of a 0.8 m doorway was half lethal and MPPI crawled through it at ~0.01 m/s.
- **Progress** 0.10 m in 30 s; goal tolerance 0.15 m, yaw 0.25 rad.
- **Recovery: back off, never spin.** The behaviour tree
  ([`minihexa_nav_to_pose_bt.xml`](../ros/hexapod_bringup/config/minihexa_nav_to_pose_bt.xml)) is Nav2's
  `navigate_to_pose_w_replanning_and_recovery` with Spin replaced by a slow BackUp (0.15 m at 0.06 m/s) and a
  2 s wait. Boxed in (a doorway, chair legs, a slipper seen late), it backs off the way it came; a spin on the
  spot swept the legs into what had stopped it.

### Costmaps

| layer | local | global | source |
|---|---|---|---|
| LiDAR | voxel layer, to 2.5 m | obstacle layer, to 2.5 m | `/scan` |
| camera | `camera/obstacle_scan`, 0.25-1.0 m | — | `camera_vision`: what stands over 4.5 cm off the floor ahead, under the LiDAR's plane ([camera.md](camera.md#vision-camera_vision)) |
| sonar | — | — | the collision monitor only (below) |

The camera has a layer of its own: in the LiDAR's layer, the LiDAR's rays over a low obstacle would clear it.
It stays out of the global costmap: monocular depth is good near and wrong far, and after a look-around its
far-floor errors fenced off every frontier. Nothing nearer than 0.25 m: the camera sees no floor there, and a
mark nearer than the raytrace minimum is never cleared, so one left on the robot's outline refused every turn.

### Collision monitor

- **FootprintApproach** (LiDAR only): slows the robot 1.2 s before its footprint would touch a scan point.
- **The sonar slows and stops forward motion only**: within 0.40 m of the body centre it slows to 30 %, within
  0.25 m it stops, both only while moving forward (vx > 0.02). The sonar sits 0.08 m forward, inside the 0.17 m
  footprint, so an echo nearer than 9 cm (a chair leg under the LiDAR's plane) lies inside the footprint. In the
  approach polygon that made the collision time 0 for every command, backing up included, and the robot froze
  against the leg. Backing up, turning on the spot and side-steps now select a zone the sonar never marks.

The sonar is in no costmap: its beam is wider than its message says (~30°, not 15°), and the range layer
marked every echo as an arc across the cone that was never cleared, until turns on the spot stacked them into rings.

## Exploration

```
ros2 launch <cockpit>/launchers/explore.launch.py config_file:=<path>/minihexa_cockpit.yaml
```

explore_lite sends Nav2 to the best frontier of the global costmap, again and again, until none is left, then
back to the start. The `explore` block sets:

- `min_frontier_size` 0.3 m: explore_lite's own 0.5 found no frontiers in a furnished room after a partial map;
  cockpit's 0.2 sent the robot into a 0.2 m corner, where the collision monitor crept it forward at 0.003 m/s
  and explore_lite counted the creep as progress.
- `progress_timeout` 45 s.

cockpit's pipeline watcher can end a run at a battery floor (`--explore-stop-volts`) or on a stall
(`--explore-stall-s`) and walk the robot home. For the miniHexa's 2S pack the bench stops at 7.2 V loaded.

**The object survey** (`object_survey`, [camera.md](camera.md#object-survey-while-exploring-object_survey))
pauses the explorer every 2 m of travel for a look around with the camera and writes a note of what it saw
where. A due survey waits for open space by the LiDAR (nearest return 0.45 m or more, half of eight sectors
0.9 m or more); it is skipped after 2 m without it, or within 1.2 m of an earlier one.

### Measured

| run | time | map | surveys | notes |
|---|---|---|---|---|
| patrol, 4 loops of 4 waypoints (2026-11-07) | 125-133 s a loop | — | — | 16/16 legs, 0.13-0.15 m off each waypoint; map drift against the room 7-17 mm, ≤ 3.7° |
| explore, 0.4 s gait at 0.2 m/s (2026-11-08) | 38 min | 49,136 known cells (about 123 m²) | 21, 14.2 min of the run | stopped at the battery floor; home within 2.0 cm and 1.4° |
| explore with the survey's open-space rule (2026-11-09) | 33 min | 53,857 known cells (about 135 m²) | 7 (8 skipped), ~5 min | 63 m walked; stopped against a chair leg under the LiDAR's plane, which led to the sonar zones above; home within 3.3 cm and 1.7° |

Home is where each run started, the map's origin.
