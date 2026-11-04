# Firmware

One PlatformIO project in `firmware/`, one binary per ROS 2 distro (`minihexa` for Jazzy, `minihexa_lyrical`
for Lyrical, where `/cmd_vel` is a `TwistStamped`). Build and flash are separate steps; never
`pio run -t upload`.

```
pio run  -d firmware -e minihexa          # build
pio test -d firmware -e native            # IK, gait and servo tests on the host
```

## Layout

| path | what |
|---|---|
| `lib/hexa_kin` | body and leg geometry, leg IK and FK. Plain C++ |
| `lib/hexa_gait` | tripod gait: body twist in, foot positions out. Plain C++ |
| `lib/hexa_servo` | joint angle ↔ pulse, per-joint calibration, the simulated servo. Plain C++ |
| `include/board_minihexa.h` | the controller board's pins. **Not identified yet** |
| `src/main.cpp` | applications, micro-ROS, console |
| `test/` | Unity tests, `native` env |

The three libraries have no Arduino dependency, so the firmware, the tests and the host simulator
(`sim/`, see [simulation.md](simulation.md)) all run the same gait and kinematics code.

## Applications

The NVS key `app` picks what boots:

- **base**: micro-ROS over USB serial at 921600 baud. Subscribes `/cmd_vel`; publishes `/joint_states` and
  `/odom/gait` at 25 Hz. The gait runs at 50 Hz with or without an agent. A `/cmd_vel` stream that stops for
  0.5 s counts as zero: the robot finishes its step and stands.
- **console**: line commands on the same port: `info` (chip, revision, flash size, MAC, mode), `i2c` (bus scan),
  `app base|console`, `sim on|off`, `reboot`.

Press BOOT within two seconds after a reset to reach the console whatever `app` says. (BOOT held *through* the
reset starts the ROM bootloader instead, as on every ESP32.)

## Sim and real

The NVS key `sim` (default on) chooses simulated servos. **Real mode is refused** until
`board_minihexa.h` sets `HEXA_BOARD_VERIFIED`. The board's servo outputs aren't known yet: 18 servos are more
than a classic ESP32's 16 LEDC channels. Until then, a wrong pin can't drive a joint into its stop. On a bare
ESP32 the base application walks simulated servos and publishes their angles. In real mode `/joint_states` will
carry the *commanded* angles, because a PWM servo reports nothing back.

## Topics

| topic | type | notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (Jazzy), `TwistStamped` (Lyrical) | `linear.x`, `linear.y`, `angular.z`; clamped to the stride limit |
| `/joint_states` | `sensor_msgs/JointState` | 18 joints `rf_coxa` … `lf_tibia`; sim: simulated angle; real: commanded |
| `/odom/gait` | `nav_msgs/Odometry` | integrated commanded body twist (dead reckoning, no slip); `odom` → `base_footprint` (as a TF by `hexapod_bringup`'s `gait_odom_tf`) |

Not yet: IMU (QMI8658A), ultrasonic range, battery. They come once their buses and pins are read off the board.

## Conventions

Body frame x forward, y left, z up. Legs `rf rm rr lr lm lf` (clockwise from the front right); tripod A = rf, rr,
lm. Joints per leg: coxa (yaw), femur (pitch, 0 = horizontal, + lifts the knee), tibia (relative to the femur,
0 = straight, − folds the foot down). Joint index = leg × 3 + joint.

The leg lengths and hip positions in `hexa_kin.cpp` are **placeholders** until measured (open question 6 in
[hardware-minihexa.md](hardware-minihexa.md)).
