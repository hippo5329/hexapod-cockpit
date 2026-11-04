// Simulated sensors for the miniHexa, fed by the robot model's true motion (hexa_world):
// the QMI8658A IMU (accelerometer and gyro, no magnetometer) and the ultrasonic range
// finder. Plain C++: the firmware's sim mode, the host simulator and the ROS 2 node use
// the same models.
//
// A simulated sensor behaves like a typical real one and declares the noise it actually
// makes: the white noise is uniform on +/-peak, so its variance is uniform_var(peak), and
// that is the covariance the messages carry (never a second constant). Bias, drift,
// scale error and mount tilt are on top, as on a real part; the fusion downstream exists
// to fight them, so a perfect sensor would make it look better than it is.
#pragma once

#include <cstdint>

#include "hexa_world.h"

namespace hexa {

// xorshift32: deterministic, cheap on the ESP32, the same sequence on every target.
class Rng {
public:
    explicit Rng(uint32_t seed = 5329) : s_(seed ? seed : 1) {}
    uint32_t next();
    float uniform(float peak);  // uniform on +/-peak
private:
    uint32_t s_;
};

constexpr float uniform_var(float peak) { return peak * peak / 3.0f; }

struct ImuParams {
    // White noise sized to the QMI8658's datasheet variances (linorobot2-cockpit's per-chip
    // table): accel 8e-5 (m/s^2)^2, gyro 2e-6 (rad/s)^2; peak = sqrt(3 var).
    float accel_noise = 0.0154919f;
    float gyro_noise = 0.0024495f;
    // Bias starts calibrated (zero, as after the boot-time gyro calibration), then
    // wanders as a random walk held inside an envelope. Generic MEMS figures: unmeasured.
    float gyro_drift = 0.0015f;  // rad/s per sqrt(s)
    float gyro_bias_limit = 0.004f;
    float accel_drift = 0.01f;   // m/s^2 per sqrt(s)
    float accel_bias_limit = 0.03f;
    float scale_error = 0.01f;   // every axis reads 1 % high
    float accel_tau_s = 0.06f;   // accelerometer band limit
    // The board is never mounted level: the tilt leaks gravity into ax and ay. The URDF's
    // imu_link is level, so the stack meets this as an unmodelled error, as on the robot.
    float mount_roll = 0.0174533f;    // 1.0 deg
    float mount_pitch = -0.0261799f;  // -1.5 deg
    float gravity = 9.80665f;
};

struct ImuSample {
    float ax, ay, az;  // specific force, m/s^2, sensor frame (+g on z when level and still)
    float gx, gy, gz;  // rad/s, sensor frame
};

class SimImu {
public:
    void init(const ImuParams& p = ImuParams(), uint32_t seed = 5329);
    // One sample per model step, from the body's true twist over that step
    // (SimRobot::truth_twist()). The body is level: the model has no roll or pitch.
    ImuSample step(float dt, const BodyTwist& truth);
    const ImuParams& params() const { return p_; }

private:
    ImuParams p_;
    Rng rng_;
    float m_[3][3];  // body -> sensor
    float vx_ = 0, vy_ = 0, ax_ = 0, ay_ = 0;
    bool first_ = true;
    float gyro_bias_[3] = {0, 0, 0};
    float accel_bias_[3] = {0, 0, 0};
};

// A vertical wall: a segment in the world (odom) frame, which starts at the robot's
// start pose.
struct Wall {
    float x1, y1, x2, y2;
};

constexpr int kRoomWalls = 4;
// The default world: a closed 2 m x 2 m room centred on the start pose.
void default_room(Wall out[kRoomWalls], float half_size = 1.0f);

struct RangeParams {
    // The sensor in base_footprint (the URDF's sonar_link): the front of the body, looking
    // forward. Placeholder until the robot is measured.
    float x = 0.080f, y = 0.0f, yaw = 0.0f;
    // HC-SR04-class figures, unmeasured: 15 deg cone, 2 cm .. 4 m.
    float fov = 0.2618f;
    float min_range = 0.02f;
    float max_range = 4.0f;
    // A ray meeting a wall further than this from its normal reflects away (specular):
    // no echo from that ray.
    float max_incidence = 0.5236f;  // 30 deg
    float noise = 0.003f;           // +/- peak, m, plus...
    float noise_frac = 0.005f;      // ... this fraction of the range
    int rays = 9;                   // across the cone
};

class SimRange {
public:
    void init(const RangeParams& p = RangeParams(), uint32_t seed = 977);
    // Nearest echo inside the cone (m); +inf when nothing answers within max_range,
    // -inf when closer than min_range (REP 117).
    float measure(const Pose2& body, const Wall* walls, int n);
    const RangeParams& params() const { return p_; }

private:
    RangeParams p_;
    Rng rng_;
};

}  // namespace hexa
