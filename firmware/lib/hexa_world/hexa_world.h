// The robot model: the gait, IK and servo mapping driving simulated servos on a flat
// floor, with the body's true pose worked out from the feet. One model for the
// firmware (both modes), the host simulator (sim/hexa_sim) and the ROS 2 node
// (ros/hexapod_sim). In real mode the firmware sends pulse_us() to the servos and
// reports commanded(); the simulated servos and the world are not published.
//
// Each step the gait gives foot targets, IK turns them into joint angles, the
// calibration into pulses, and each SimServo turns toward its pulse at a bounded
// rate. Forward kinematics of the servos' ACTUAL angles gives where the feet really
// are; the feet on the ground do not move in the world, so the body's true motion is
// the rigid transform that best explains how those feet moved in the body frame
// (2-D least squares). A slip fraction loses part of that motion, as a real foot on a
// smooth floor does. Gait odometry integrates what the gait commanded, as the
// firmware's /odom/gait does.
#pragma once

#include "hexa_gait.h"
#include "hexa_kin.h"
#include "hexa_servo.h"

namespace hexa {

struct Pose2 {
    double x = 0, y = 0, yaw = 0;
    void apply_body(double dx, double dy, double dyaw);
};

// Body motion that maps foot positions `now` onto `before` (feet fixed in the world),
// from the legs with use[i]. False with fewer than two.
bool rigid_2d(const Vec3* before, const Vec3* now, const bool* use, double* dx, double* dy, double* dyaw);

// The variances gait odometry declares (nav_msgs/Odometry diagonals: x, y, z, roll, pitch,
// yaw), one table for the firmware and the ROS 2 node. Dead reckoning from commanded steps:
// the pose is loose and its yaw the loosest (no magnetometer). The twist is loose while the
// gait walks (slip, servo lag). While it stands the body is still and the gait's zero is
// exact, so the twist is declared tighter than any gyro: an EKF fusing both then holds the
// heading on it instead of integrating the gyro's bias (a zero-velocity update). A robot
// pushed while it stands breaks that promise; nothing on board can see it.
struct OdomVariance {
    double pose[6];
    double twist[6];
};
OdomVariance gait_odom_variance(bool standing);

struct StepCounts {
    int unreachable = 0;  // legs whose foot target IK could not reach (they keep their angles)
    int clamped = 0;      // joint pulses clamped to the servo range
};

// Rest: the idle mode. The servo controller has no "servos off" (its protocol only sets
// pulses, and it holds the last ones), so the robot saves power by lying down: with the gait
// standing, the body lowers onto its belly (12-15 mm under the hip axis, measured) and the
// feet come up off the floor, so the servos hold only the legs. Feet 10 mm under the hip
// axis, 75 mm out: every femur pulse, trims included, stays >= 38 us inside its clamp.
struct RestParams {
    float reach = 0.075f;  // hip-to-foot, horizontal
    float z = -0.010f;     // foot height in the body frame (stand_z -0.060)
    float time_s = 3.0f;   // down, and up again
};

class SimRobot {
public:
    // Standing on the gait's neutral stance, servos already there. False if that
    // stance is unreachable (a geometry fault).
    bool init(double slip = 0.05, const GaitParams& gp = GaitParams(), const SimServoParams& sp = SimServoParams());

    // Per-servo trims (us, indexed by servo id - 1: the vendor's "deviation"), added to
    // every pulse from now on. The simulated servos read pulses through the same
    // calibration, so in simulation a trim moves nothing.
    void set_trims(const int16_t* by_servo_id);

    // A command that moves wakes a resting robot: it stands up first (the gait gets zero
    // until it is up), then walks.
    void command(const BodyTwist& cmd, float now_s);
    // Lie down (taken once the gait stands) or stand up again.
    void rest(bool on) { rest_want_ = on; }
    // 0 standing .. 1 lying down.
    float rest_level() const { return rest_s_; }
    bool resting() const { return rest_s_ >= 1.0f; }
    // The gait's parameters, taken only while it stands (TripodGait::set_params).
    bool set_gait_params(const GaitParams& p) {
        if (!gait_.set_params(p)) return false;
        gp_ = p;
        return true;
    }
    StepCounts step(float dt, float now_s);

    float joint_angle(int k) const { return servo_[k].angle(); }  // the simulated servo's
    float commanded(int k) const { return commanded_[k]; }        // what IK asked for
    int pulse_us(int k) const { return pulse_us_[k]; }            // what goes to the servo
    const Pose2& truth() const { return truth_; }
    // The true body twist over the last step (body frame, after slip).
    const BodyTwist& truth_twist() const { return truth_twist_; }
    const Pose2& odom() const { return odom_; }
    BodyTwist odom_twist() const { return gait_.applied(); }
    const TripodGait& gait() const { return gait_; }
    const GaitParams& gait_params() const { return gp_; }
    bool foot_down(int leg) const { return contact_[leg]; }

    // Both poses back to the origin (the robot stays where its feet are).
    void reset_poses() { truth_ = Pose2(); odom_ = Pose2(); }

private:
    void actual_feet(Vec3* out, bool* contact) const;

    const Geometry* g_ = nullptr;
    double slip_ = 0.05;
    GaitParams gp_;
    SimServoParams sp_;
    TripodGait gait_;
    ServoCal cal_[kJoints];
    SimServo servo_[kJoints];
    float commanded_[kJoints];
    int pulse_us_[kJoints];
    Vec3 feet_[kLegs];
    bool contact_[kLegs];
    Pose2 truth_, odom_;
    BodyTwist truth_twist_{0, 0, 0};
    RestParams rp_;
    bool rest_want_ = false;
    float rest_s_ = 0;
};

}  // namespace hexa
