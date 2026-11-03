# Simulation

Two ways to run the miniHexa without the robot. Both use the firmware's own gait, IK and servo code
(`firmware/lib`), so what walks in simulation is what will walk on the robot.

## Host simulator (`sim/`)

```
make -C sim run                          # every scenario, exit 0 on PASS
sim/hexa_sim --slip 0.10 --csv walk.csv  # another slip, a trace per tick
```

Each 20 ms tick follows the same path as the firmware:
1. The gait gives foot targets.
2. IK turns them into joint angles.
3. The calibration turns the angles into pulses.
4. Each simulated servo turns toward its pulse.

The simulator then works out where the robot really went:
- Forward kinematics of the servos' **actual** angles gives the real foot positions.
- A foot on the ground does not move in the world. The body's true motion is therefore the rigid transform that
  best explains how the grounded feet moved in the body frame (2-D least squares).
- A slip fraction then removes part of that motion.

It also integrates the commanded body motion, exactly as the firmware's `/odom/gait` does, and reports how far
that is from the truth.

The scenarios are: forward, strafe, turn on the spot, arc, and a command beyond the stride limit (which gets
clamped).

A scenario passes when:
- no foot target was out of reach;
- no pulse was clamped;
- the robot stood within two gait cycles of the stop;
- the true travel was 75–105 % of the command.

With no slip, the true motion matches the command within about 1 mm and 1° over 0.3 m and 137°. That shows the
IK, FK and gait are consistent with each other.

### Model figures

| figure | value | basis |
|---|---|---|
| servo rate | 10.5 rad/s | listed HPD-0127 0.05 s/60° at 7.4 V, halved for load. **Unmeasured** |
| servo deadband | 3 µs | a typical hobby micro servo. **Unmeasured** |
| pulse resolution | 1 µs | firmware rounds to whole µs |
| foot slip | 5 % of stance motion (`--slip`) | assumed for hard feet on a smooth floor. **Unmeasured** |
| geometry | coxa 30, femur 50, tibia 75 mm; hips on a 120 × 110 mm layout | **placeholders** sized to the 240 × 234 mm listing |

Each figure gets replaced by a bench measurement once the robot is here. The sim is only as good as these numbers.

## Firmware sim mode

On a bare ESP32 (or the miniHexa before its pins are identified) the `base` application drives simulated servos
and publishes their angles on `/joint_states`. A micro-ROS agent on the serial port and a `/cmd_vel` publisher
are enough to watch the gait in RViz (`display.launch.py source:=robot`, below).

## Robot model and RViz (`ros/hexapod_description`)

A ROS 2 package with the miniHexa URDF (xacro), an RViz config and a display launch. Build it in a colcon workspace,
then:

```
ros2 launch hexapod_description display.launch.py                # the gait's standing pose
ros2 launch hexapod_description display.launch.py source:=gui    # a slider per joint
ros2 launch hexapod_description display.launch.py source:=robot  # /joint_states from the firmware
```

- `stand` holds the gait's neutral stance (`config/stand_pose.yaml`).
- `gui` needs `joint_state_publisher_gui`, which not every image has.
- `robot` takes `/joint_states` from the firmware's `base` application, through a micro-ROS agent: sim mode on a
  bare ESP32, or the robot itself. A hand-published `/joint_states` needs a real stamp (`header: {stamp: now}`):
  `robot_state_publisher` publishes nothing for stamp zero.

![miniHexa standing pose in RViz](images/minihexa_rviz.png)

The fixed frame is `base_footprint`, the floor under the standing body. `base_link` is at hip height.

The URDF is tied to the firmware by tests (`colcon test`):
- **Joint names:** the URDF's are the firmware's 18.
- **Kinematics:** the foot positions agree with the firmware's FK within 0.01 mm, at 300 random poses inside the
  servo range.
- **Limits:** the joint limits are the servo range.
- **Standing pose:** it puts every foot on the gait's neutral stance.

Change a length in `hexa_kin.cpp` without changing it in `minihexa.urdf.xacro` (or the reverse), and the tests fail.

## Not yet

- A ROS 2 node that wraps the host simulator (publishing `/joint_states`, `/odom/gait` and the true pose as
  `/sim/ground_truth`).
- IMU and range simulation.
- Uneven ground, and body roll and pitch from leg compliance.
