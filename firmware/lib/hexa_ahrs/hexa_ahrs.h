// Attitude estimate on the board: Madgwick's gradient-descent filter, 6-axis (the
// QMI8658A has no magnetometer). The firmware runs it on every IMU sample, simulated or
// real, and /imu/data carries the result; the ROS 2 node runs the same code.
//
// Written from Madgwick's report (S. Madgwick, "An efficient orientation filter for
// inertial and inertial/magnetic sensor arrays", 2010): the gyro rate integrated, and a
// step of fixed size beta down the gradient of the error between the gravity the estimate
// predicts and the accelerometer's direction. This is the filter imu_filter_madgwick runs
// for an IMU without a magnetometer; the lab checks the two agree.
//
// What it can and cannot know:
//   roll, pitch   levelled against gravity: absolute, to the filter's accuracy. They are
//                 the SENSOR's (imu_link's): a board mounted off level reads that tilt.
//   yaw           the gyro's integral from zero at the first sample, with nothing to
//                 anchor it: it drifts with the gyro bias. A consumer fuses the yaw RATE,
//                 never this yaw as absolute (the declared yaw variance says so).
//
// Orientation is the ROS one: the rotation from the sensor frame to the world (z up),
// quaternion w, x, y, z.
#pragma once

namespace hexa {

struct Quat {
    float w = 1, x = 0, y = 0, z = 0;
};

struct AhrsParams {
    // Gradient step, rad/s: imu_filter_madgwick's default. Madgwick sizes it as
    // sqrt(3/4) x the gyro's error; larger corrects tilt faster but lets each footfall's
    // jolt pull the estimate further. Unmeasured on the robot.
    float beta = 0.1f;
    // A gap longer than this is a stalled loop, not motion: the estimate is held.
    float max_dt = 0.5f;
    // What the message declares: roll and pitch (0.2 deg)^2, the filter's measured error
    // in simulation (rms 0.15-0.16 deg standing, turning and walking; test_ahrs), mostly
    // the accelerometer's bias and noise. Yaw has no reference at all.
    float roll_pitch_var = 1.22e-5f;
    float yaw_var = 1e6f;
};

class Ahrs {
public:
    void init(const AhrsParams& p = AhrsParams());
    // One IMU sample: gyro rad/s, accelerometer m/s^2 (any scale: only its direction is
    // used), dt since the previous sample. The first sample with a usable accelerometer
    // seeds roll and pitch from gravity (yaw 0), so the first estimate is already level.
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    bool seeded() const { return seeded_; }
    const Quat& q() const { return q_; }
    float roll() const;
    float pitch() const;
    float yaw() const;
    const AhrsParams& params() const { return p_; }

private:
    bool seed(float ax, float ay, float az);

    AhrsParams p_;
    Quat q_;
    bool seeded_ = false;
};

}  // namespace hexa
