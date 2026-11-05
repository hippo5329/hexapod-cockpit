# hexapod-cockpit

One-click from a bare hexapod controller to ROS 2: unified micro-ROS firmware, a gait engine, a web supervisor,
SLAM and Nav2 for six-legged walkers. Sister project of
[linorobot2-cockpit](https://github.com/hippo5329/linorobot2-cockpit), which does the same for wheeled robots.

**Status: early.** The first target robot is the Hiwonder **miniHexa** Standard Kit (18 DOF, ESP32), described in
[`docs/hardware-minihexa.md`](docs/hardware-minihexa.md). The plan is in [`docs/architecture.md`](docs/architecture.md).

What runs today:
- **Firmware** ([`docs/firmware.md`](docs/firmware.md)): tripod gait, leg IK, an AHRS and micro-ROS over USB
  serial or Wi-Fi, with OTA updates (`/cmd_vel` in; `/joint_states`, `/odom/gait`, `/imu/data` with
  orientation, `/sonar` and `/battery` out). On the robot it drives the 18 servos and reads the real IMU,
  ultrasonic and battery; on a bare ESP32 it walks a simulated robot with simulated sensors.
- **Host simulator** ([`docs/simulation.md`](docs/simulation.md)): the same gait code walking simulated servos, with
  true pose vs gait odometry. Run it with `make -C sim run`.
- **Robot model** (`ros/hexapod_description`): URDF and RViz view of the miniHexa, tested against the firmware's
  kinematics. Run it with `ros2 launch hexapod_description display.launch.py`.
- **Walking view and EKF** (`ros/hexapod_bringup`): robot_localization's EKF fuses the gait's twist with the
  gyro's yaw rate into `/odom` and the `odom` → `base_footprint` TF, so RViz shows the robot walking across the
  floor. Run it with `ros2 launch hexapod_bringup view_robot.launch.py`
  beside a micro-ROS agent.
- **Simulator node** (`ros/hexapod_sim`): the host simulator in place of the firmware, with ground truth and
  the simulated IMU and ultrasonic in a walled room. Run it with `ros2 launch hexapod_sim sim.launch.py`: no
  board needed.

## Goals

- Stock miniHexa in, ROS 2 robot out: flash once from the browser supervisor, then drive by `/cmd_vel`.
- Gait and inverse kinematics on the controller, so walking survives a dropped link.
- Honest odometry: a legged robot has no wheel encoders; the firmware reports commanded body motion and the IMU,
  and the stack says which one it trusts.
- Simulation mode that walks like the real robot, for testing without hardware.
- Both ROS 2 distros the cockpit supports (Jazzy and Lyrical).

## Licence

Apache-2.0, see [`LICENSE`](LICENSE).
