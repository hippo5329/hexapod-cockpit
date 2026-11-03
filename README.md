# hexapod-cockpit

One-click from a bare hexapod controller to ROS 2: unified micro-ROS firmware, a gait engine, a web supervisor,
SLAM and Nav2 for six-legged walkers. Sister project of
[linorobot2-cockpit](https://github.com/hippo5329/linorobot2-cockpit), which does the same for wheeled robots.

**Status: planning.** Nothing here runs yet. The first target robot is the Hiwonder **miniHexa** Standard Kit
(18 DOF, ESP32), described in [`docs/hardware-minihexa.md`](docs/hardware-minihexa.md). The plan is in
[`docs/architecture.md`](docs/architecture.md).

## Goals

- Stock miniHexa in, ROS 2 robot out: flash once from the browser supervisor, then drive by `/cmd_vel`.
- Gait and inverse kinematics on the controller, so walking survives a dropped link.
- Honest odometry: a legged robot has no wheel encoders; the firmware reports commanded body motion and the IMU,
  and the stack says which one it trusts.
- Simulation mode that walks like the real robot, for testing without hardware.
- Both ROS 2 distros the cockpit supports (Jazzy and Lyrical).

## Licence

Apache-2.0, see [`LICENSE`](LICENSE).
