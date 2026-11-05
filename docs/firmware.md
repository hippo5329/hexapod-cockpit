# Firmware

One PlatformIO project in `firmware/`, one binary per ROS 2 distro (`minihexa` for Jazzy, `minihexa_lyrical`
for Lyrical, where `/cmd_vel` is a `TwistStamped`). Build and flash are separate steps; never
`pio run -t upload`.

```
pio run  -d firmware -e minihexa          # build
pio test -d firmware -e native            # IK, gait, servo, sensor and AHRS tests on the host
```

## Layout

| path | what |
|---|---|
| `lib/hexa_kin` | body and leg geometry, leg IK and FK. Plain C++ |
| `lib/hexa_gait` | tripod gait: body twist in, foot positions out. Plain C++ |
| `lib/hexa_servo` | joint angle ↔ pulse, per-joint calibration, the simulated servo. Plain C++ |
| `lib/hexa_world` | the robot model: gait, IK and pulses for both modes; in sim mode the simulated servos and the true pose from the feet. Plain C++ |
| `lib/hexa_sensors` | the simulated IMU (QMI8658A) and ultrasonic. Plain C++ |
| `lib/hexa_ahrs` | the AHRS: orientation from the IMU's gyro and accelerometer (below). Plain C++ |
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

## AHRS

The board fuses the IMU itself (`lib/hexa_ahrs`), so `/imu/data` carries an orientation and no filter node is
needed on the host. It is Madgwick's gradient-descent filter for a 6-axis IMU, written from his 2010 report:
the gyro rate is integrated, and each sample takes a step of fixed size β down the gradient of the error between
the gravity the estimate predicts and the accelerometer's direction. It is the filter `imu_filter_madgwick` runs
for an IMU without a magnetometer; driven with the same samples and seeded alike, the two agree to 2.4e-5 rad
once the reference's fast inverse square root is replaced by an exact one (with it, the reference sits up to
0.3° away standing still).

- **Gain** β = 0.1, `imu_filter_madgwick`'s default: a 10° tilt step settles within 0.5° in 0.84 s. Unmeasured
  on the robot.
- **Seed**: the first sample sets roll and pitch from gravity, with yaw 0, so the first message is already
  level. Until then `orientation_covariance[0]` is −1 (REP 145: no orientation).
- **Roll and pitch** are absolute, levelled against gravity. They are the *sensor's*: a board mounted off level
  reads its own tilt (the simulated one, roll 1.0° and pitch −1.5°, shows up as such). Declared variance
  (0.2°)², from the filter's error in simulation.
- **Yaw** is the gyro's integral from 0 at boot. The QMI8658A has no magnetometer, so nothing anchors it and it
  drifts with what is left of the gyro's bias. Its declared variance is 10⁶: fuse the yaw *rate*, never this yaw
  as absolute.
- **Gyro bias** (`GyroBias`): a standing walker does not turn, and the gait knows when it stands (every foot down
  on its neutral position, nothing commanded). After 0.5 s standing the board averages the gyro (time constant
  1 s) into a bias estimate and takes it out of every sample, before the AHRS and before `/imu/data`; walking,
  the estimate is held. A reading over 0.05 rad/s is motion, not bias, and is not learned. In simulation, 120 s
  standing drift the yaw 0.0045 rad at worst instead of 0.055. A robot turned slowly by hand while it stands
  would be learned as bias: nothing on board can tell the two apart.
- **Robustness**: a gap over 0.5 s (a stalled loop) holds the estimate rather than integrating it as motion; a
  zero or non-finite accelerometer falls back to the gyro; a non-finite gyro is skipped.
- **Gravity stays in** `linear_acceleration` (REP 145): a consumer that wants motion removes it.

It runs on every IMU sample, connected to an agent or not, at the 50 Hz frame (dt = 20 ms). The ROS 2 node runs
the same code. A real QMI8658A will need dt from the sample timestamps.

## Topics

| topic | type | notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (Jazzy), `TwistStamped` (Lyrical) | `linear.x`, `linear.y`, `angular.z`; clamped to the stride limit |
| `/joint_states` | `sensor_msgs/JointState` | 18 joints `rf_coxa` … `lf_tibia`; sim: simulated angle; real: commanded |
| `/odom/gait` | `nav_msgs/Odometry` | integrated commanded body twist (dead reckoning, no slip), `odom` → `base_footprint`; covariances below |
| `/imu/data` | `sensor_msgs/Imu` | sim mode: the simulated QMI8658A, 50 Hz ([simulation.md](simulation.md)), gyro bias taken out, with the AHRS's orientation (above) |
| `/sonar` | `sensor_msgs/Range` | sim mode: the simulated ultrasonic, 10 Hz |
| `/sim/ground_truth` | `nav_msgs/Odometry` | sim mode: the true pose, 5 Hz |

`/odom/gait` declares (`gait_odom_variance`, one table for the firmware and the ROS 2 node):

| | x, y | yaw | z, roll, pitch |
|---|---|---|---|
| pose | 0.01 | 0.05 | 10⁶ |
| twist, walking | 0.004 | 0.02 | 10⁶ |
| twist, standing | 1e-8 | 1e-8 | 10⁶ |

Walking, the feet slip and servos lag, so the twist is loose and an EKF's heading follows the gyro (2e-6). Standing,
the body is still and the gait's zero is exact, declared tighter than the gyro: the EKF holds the heading on it
(a zero-velocity update). Its pose is the integral of the same twist; the EKF does not fuse it.

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
