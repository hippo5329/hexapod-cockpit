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
| `lib/hexa_servo` | joint angle ↔ pulse, per-joint calibration, servo ids, the servo controller's frame, the simulated servo. Plain C++ |
| `lib/hexa_world` | the robot model: gait, IK and pulses for both modes; in sim mode the simulated servos and the true pose from the feet. Plain C++ |
| `lib/hexa_sensors` | the simulated IMU (QMI8658A) and ultrasonic. Plain C++ |
| `lib/hexa_ahrs` | the AHRS: orientation from the IMU's gyro and accelerometer (below). Plain C++ |
| `include/board_minihexa.h` | the controller board's pins and buses (verified on a robot) |
| `src/main.cpp` | applications, micro-ROS, console |
| `src/board_io.*` | the board's buses: the servo controller (UART), battery monitor, ultrasonic and IMU (I2C) |
| `src/qmi8658.*` | the QMI8658 driver, ported from linorobot2-cockpit |
| `src/link.*` | the micro-ROS link (USB serial or Wi-Fi UDP) and OTA updates |
| `tools/` | `make_nvs.py` (robot settings → NVS image), `ota_upload.py` |
| `test/` | Unity tests, `native` env |

The libraries have no Arduino dependency, so the firmware, the tests, the host simulator and the ROS 2 node
(see [simulation.md](simulation.md)) all run the same model.

## Applications

The NVS key `app` picks what boots:

- **base**: micro-ROS over USB serial at 921600 baud, or over Wi-Fi (below). Subscribes `/cmd_vel` and
  `/gait_params`; publishes the topics below. The gait runs at 50 Hz with or without an agent. A `/cmd_vel`
  stream that stops for 0.5 s counts as zero: the robot finishes its steps and stands.
- **console**: line commands on the USB serial port. On a Wi-Fi link the port is free, so the base application
  takes the same commands there too. `info` (chip, revision, flash size, MAC, last reset, mode, IMU, sonar,
  gait, link), `link`, `i2c` (bus scan), `bat`, `sonar`, `imu`, `trims`, `app base|console`, `sim on|off`,
  `reboot`. Bench commands, sim mode only (the robot on a stand): `servo <id> <us> [ms]` moves one servo;
  `foot <leg> <dx> <dy> <dz> [ms]` moves one leg's foot by millimetres from its standing position, body frame,
  through the gait's IK and the servo calibration.

Press BOOT within two seconds after a reset to reach the console whatever `app` says. (BOOT held *through* the
reset starts the ROM bootloader instead, as on every ESP32.)

A loop stuck for 5 s trips the task watchdog: the board reboots and reconnects instead of freezing the gait
mid-stride. `info` says when that was the last reset.

## Robot settings

Everything that belongs to one robot lives in the NVS namespace `hexa`, written as an NVS image with the
firmware (`tools/make_nvs.py <settings file> <out.bin>`, flashed at 0x9000; OTA updates leave it in place):

| key | value |
|---|---|
| `app` | `base` (default) or `console` |
| `sim` | `1` (default): simulated servos and sensors; `0`: the real robot |
| `link` | `serial` (default) or `wifi` |
| `ssid`, `psk` | the Wi-Fi network (2.4 GHz) and its key |
| `agent`, `port` | the micro-ROS agent's IPv4 address and UDP port (default 8888) |
| `ota_pw` | the OTA password; none: no OTA |
| `trims` | 18 comma-separated servo trims in µs, by servo id: the vendor's "deviation" values |
| `imu_level` | `roll,pitch` in degrees (each within ±10): how the IMU sits on its board, read with the body level (a level on the top plate) from the console's `imu`. Every IMU sample is turned into the body frame with it; none: the board is taken as level |

The settings file carries the Wi-Fi key and the OTA password: keep it private (mode 0600) and out of any
public repository. Read the trims off the robot (the vendor firmware keeps them in its NVS) **before** our
firmware replaces it.

## Wi-Fi and OTA

With `link=wifi` the board joins the network (modem sleep off) and talks to a micro-ROS agent over UDP
(`micro_ros_agent udp4 -p 8888`), one datagram per message. A Wi-Fi link that cannot work (no network or no
valid agent address) falls back to serial. The USB port then prints a status line on every change and every
30 s: network, address, signal, channel, power-save mode, agent, datagrams sent, refused and received, the OTA
state, the running partition and the first 16 hex digits of the running ELF's SHA-256 (`sha256sum
firmware.elf` matches it).

With `ota_pw` set, the board takes ArduinoOTA updates on TCP 3232 (no mDNS): `tools/ota_upload.py <board ip>
<firmware.bin> <password file>`. The gait is told to stand when an update starts. The partition table
(`min_spiffs.csv`) has two 1.9 MB app slots; each update goes to the other slot, so a failed one leaves the
running app in place. A wrong password is refused.

The link's quality is the access point's: a busy shared 2.4 GHz network gave pings of 30-50 ms with 250 ms
peaks and up to half the sends refused (the driver's queue full), and the topics fell short of their rates; a
dedicated access point gave 4.5 ms with 14 ms peaks and every check at its rate.

## Sim and real

The NVS key `sim` (default on) chooses the simulated robot. On a bare ESP32 the base application walks simulated
servos and publishes their angles, plus the simulated IMU, ultrasonic and ground truth.

In real mode (`sim=0`, allowed because `board_minihexa.h` sets `HEXA_BOARD_VERIFIED`) the gait drives the 18
servos: every tick one frame to the servo controller with all 18 pulses (the first after boot over 1 s, from
wherever the legs hang to the stand). `/joint_states` carries the *commanded* angles, because a PWM servo reports
nothing back. The sensors are the real parts, each published only when it answers at boot: the QMI8658 as
`/imu/data`, the ultrasonic as `/sonar`, the battery monitor as `/battery` (that one in both modes). **Nothing
simulated is published in real mode.**

## Servos

The servos sit behind a servo controller on the board, which takes frames on a UART (230400 baud): header
`55 55`, function `01`, length, command `01`, count, move time (ms), then per servo its id and pulse (µs), and a
checksum (`servo_bus_frame`). Pulses run 500-2500 µs over 280° (409 µs/rad). The calibration (`default_cal`) is
the vendor's convention: at 1500 µs every joint sits at its zero (coxa and femur 0, tibia −90° from the femur);
a longer pulse turns every coxa counter-clockwise seen from above, lowers a right femur and raises a left one
(the left femur and tibia servos are mounted mirrored); the femur is held to 857-2143 µs (±90°). The vendor
numbers the legs 1-3 right rear, middle, front and 4-6 left front, middle, rear; servo id = (leg − 1) × 3 +
joint + 1 (`servo_id`). Each servo's trim is added to every pulse. The standing pose gives the vendor's
standing pulses (1500 / 1409 / 1357 µs right, 1500 / 1591 / 1643 left) within 1 µs.

## Gait

A tripod gait (`lib/hexa_gait`). Defaults (`GaitParams`), tuned on a robot on glazed tile: a 1.5 s cycle, each
foot down 70 % of it (both tripods down for a fifth of the cycle at each change), 30 mm lift, 60 mm longest
stride; top speed 57 mm/s. Measured by the gyro and a camera against the earlier 1.0 s / 0.5 / 25 mm / 40 mm:
turns reach 65-68 % of the command instead of 38-41 %, straight lines 67 % instead of 53 % (hard feet slip on
tile; `/odom/gait` integrates the command, so it reads long by that much). A slower cycle takes fewer, longer
steps per metre, so fewer touchdowns skid; more overlap helps a little; a higher lift did not.

`/gait_params` (`std_msgs/Float32MultiArray`: period s, duty, lift m, longest stride m) changes them at run time.
They are taken when the gait next stands and only within bounds (period 0.5-3 s, duty 0.5-0.8, lift 10-40 mm,
stride 10-60 mm); `info` shows the ones in use.

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
| `/gait_params` | `std_msgs/Float32MultiArray` | subscribed: the gait's parameters (above) |
| `/imu/data` | `sensor_msgs/Imu` | 50 Hz, gyro bias taken out, with the AHRS's orientation (above). Sim: the simulated QMI8658A ([simulation.md](simulation.md)); real: the QMI8658 (±8 g, ±1024 dps, 224 Hz, on-chip low-pass ~30 Hz), in the body frame (the robot's `imu_level`) |
| `/sonar` | `sensor_msgs/Range` | 10 Hz. Sim: the simulated ultrasonic; real: the I2C module. Nothing within reach reads `max_range`, never +INF (`nav2_collision_monitor` treats an out-of-span reading as an invalid source and stops) |
| `/battery` | `sensor_msgs/BatteryState` | 1 Hz, the pack's voltage from the board's monitor (both modes; only when it answers) |
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

Every published topic is best effort.

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

The leg lengths, hip positions and mount directions in `hexa_kin.cpp` are the vendor's design figures
([hardware-minihexa.md](hardware-minihexa.md)): coxa 28.5 mm, femur 52 mm, tibia 72 mm (foot plate included);
hips at (±56.5, ±31.5) mm and (0, ±43.4) mm; corner legs pointing out at 45°, middle legs straight out.
