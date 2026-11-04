# Firmware

One PlatformIO project in `firmware/`, one binary per ROS 2 distro (`minihexa` for Jazzy, `minihexa_lyrical`
for Lyrical, where `/cmd_vel` is a `TwistStamped`). Build and flash are separate steps; never
`pio run -t upload`.

```
pio run  -d firmware -e minihexa          # build
pio test -d firmware -e native            # IK, gait, servo and sensor tests on the host
```

## Layout

| path | what |
|---|---|
| `lib/hexa_kin` | body and leg geometry, leg IK and FK. Plain C++ |
| `lib/hexa_gait` | tripod gait: body twist in, foot positions out. Plain C++ |
| `lib/hexa_servo` | joint angle ↔ pulse, per-joint calibration, the simulated servo. Plain C++ |
| `lib/hexa_world` | the robot model: gait, IK and pulses for both modes; in sim mode the simulated servos and the true pose from the feet. Plain C++ |
| `lib/hexa_sensors` | the simulated IMU (QMI8658A) and ultrasonic. Plain C++ |
| `include/board_minihexa.h` | the controller board's pins. **Not identified yet** |
| `src/main.cpp` | applications, micro-ROS, console |
| `test/` | Unity tests, `native` env |

The libraries have no Arduino dependency, so the firmware, the tests, the host simulator and the ROS 2 node
(see [simulation.md](simulation.md)) all run the same model.

## Applications

The NVS key `app` picks what boots:

- **base**: micro-ROS over USB serial at 921600 baud. Subscribes `/cmd_vel`; publishes the topics below. The
  gait runs at 50 Hz with or without an agent. A `/cmd_vel` stream that stops for 0.5 s counts as zero: the
  robot finishes its step and stands.
- **console**: line commands on the same port: `info` (chip, revision, flash size, MAC, last reset, mode), `i2c` (bus scan),
  `app base|console`, `sim on|off`, `reboot`.

Press BOOT within two seconds after a reset to reach the console whatever `app` says. (BOOT held *through* the
reset starts the ROM bootloader instead, as on every ESP32.)

A loop stuck for 5 s trips the task watchdog: the board reboots and reconnects instead of freezing the gait
mid-stride. `info` says when that was the last reset.

## Sim and real

The NVS key `sim` (default on) chooses simulated servos. **Real mode is refused** until
`board_minihexa.h` sets `HEXA_BOARD_VERIFIED`. The board's servo outputs aren't known yet: 18 servos are more
than a classic ESP32's 16 LEDC channels. Until then, a wrong pin can't drive a joint into its stop. On a bare
ESP32 the base application walks simulated servos and publishes their angles, plus the simulated IMU, ultrasonic
and ground truth. In real mode `/joint_states` will carry the *commanded* angles, because a PWM servo reports
nothing back, and no sensor is published until its bus and pins are read off the board.

## Topics

| topic | type | notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (Jazzy), `TwistStamped` (Lyrical) | `linear.x`, `linear.y`, `angular.z`; clamped to the stride limit |
| `/joint_states` | `sensor_msgs/JointState` | 18 joints `rf_coxa` … `lf_tibia`; sim: simulated angle; real: commanded |
| `/odom/gait` | `nav_msgs/Odometry` | integrated commanded body twist (dead reckoning, no slip); `odom` → `base_footprint` (as a TF by `hexapod_bringup`'s `gait_odom_tf`) |
| `/imu/data` | `sensor_msgs/Imu` | sim mode: the simulated QMI8658A, 50 Hz, no orientation yet ([simulation.md](simulation.md)) |
| `/sonar` | `sensor_msgs/Range` | sim mode: the simulated ultrasonic, 10 Hz |
| `/sim/ground_truth` | `nav_msgs/Odometry` | sim mode: the true pose, 5 Hz |

Every published topic is best effort. Real IMU, ultrasonic and battery come once their buses and pins are read
off the board.

### Serial link budget

At 921600 baud the link carries about 90 KB/s. The topics above are about 50 KB/s of messages (`/joint_states`
452 B and `/odom/gait` 732 B at 25 Hz, `/imu/data` 332 B at 50 Hz, the rest small or slow). With ground truth
at 25 Hz it was 65 KB/s, and rates fell to ~88 % as the link saturated. Keep new topics inside the budget, or move
the robot to Wi-Fi.

The executor polls with a zero timeout. With 1 ms, the micro-ROS client's framed read sometimes never returned
(it takes each read's elapsed time off its timeout in whole milliseconds) and the loop hung for good: about
half the sessions at full load, on both distros. A core dump showed the loop inside that read.

## Conventions

Body frame x forward, y left, z up. Legs `rf rm rr lr lm lf` (clockwise from the front right); tripod A = rf, rr,
lm. Joints per leg: coxa (yaw), femur (pitch, 0 = horizontal, + lifts the knee), tibia (relative to the femur,
0 = straight, − folds the foot down). Joint index = leg × 3 + joint.

The leg lengths and hip positions in `hexa_kin.cpp` are **placeholders** until measured (open question 6 in
[hardware-minihexa.md](hardware-minihexa.md)).
