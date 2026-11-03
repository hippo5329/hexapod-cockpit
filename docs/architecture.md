# Architecture (plan)

What carries over from linorobot2-cockpit, and what a walker changes.

## Layers

```
browser ── supervisor (FastAPI :8000) ── rosbridge :9090
                 │
robot computer:  micro-ROS agent ── hexapod_bringup ── EKF ── SLAM / Nav2
                 │ (Wi-Fi UDP, or USB serial)
controller:      ESP32 firmware: micro-ROS node · gait engine · IK · 18 servo outputs · IMU · ultrasonic · battery
```

The miniHexa has no robot computer on board, so the first setup runs the agent and stack on a PC over Wi-Fi
(like a robot without a Pi). A small SBC on the back is a later option.

## Firmware (one binary, tools chosen by `app`, as in the cockpit)

- **base**: micro-ROS node. Subscribes `/cmd_vel` (Twist on Jazzy, TwistStamped on Lyrical) and `/body_pose`
  (height, roll, pitch, yaw of the body over the feet). Publishes `/imu/data_raw`, `/range`, `/battery`,
  `/joint_states` (commanded angles: the servos give no feedback), `/odom/gait` (body motion from completed steps).
- **Gait engine on the controller**: tripod first, then ripple and wave. Body velocity → step length and
  direction per leg → foot trajectories → leg IK → servo pulses at 50 Hz. A lost `/cmd_vel` stream (timeout) ends
  the current step and stands, never freezes mid-stride.
- **Tools**: `servo_trim` (centre and direction per joint, saved to NVS), `test_servos` (one joint at a time),
  `test_imu`, `test_sensors`, `i2c_detect`, `adc_calibrate` (battery divider).
- Build and flash stay separate: PlatformIO builds, the cockpit flasher writes.

## Odometry

No encoders: `/odom/gait` is dead reckoning from commanded steps and will slip. The EKF fuses it with the IMU's
gyro for yaw. The QMI8658A has no magnetometer, so yaw has no absolute anchor; long runs need SLAM's scan
matching, and the Standard Kit has no LiDAR. Nav2 is therefore phase 3 and needs a range sensor added.

## Phases

1. **Bring-up**: identify the board (questions in `hardware-minihexa.md`), save the vendor trims, flash our
   firmware, stand, move each joint, read IMU and range on their topics.
2. **Walk**: tripod gait from `/cmd_vel`, teleop from the web UI, gait odometry measured against the floor.
3. **Map**: add a LiDAR (an LD19 is the cockpit's default) or the camera, then SLAM and Nav2 with a legged
   footprint and slow limits.
4. **Simulation**: a simulated controller that walks with the same gait code and the real robot's slip.

## Reused from linorobot2-cockpit

Supervisor and web UI shell, flasher, per-robot config files (`<robot>_config.yaml`, config data outside the
repo), Docker images per distro, release and gate process. Wheel kinematics, motor drivers, encoders and PID are
not used.
