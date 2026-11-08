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
// feet come down beside it, 75 mm out from each hip and 15 mm under the hip axis: on the floor
// with the belly, so the robot lies on its belly and six feet (steadier than on the belly
// alone with the feet held up, user 2026-11-08), the servos still holding only the legs.
// Body pose: the body shifted (x forward, y left, z up, m) and turned (roll, pitch, yaw, rad: yaw
// about z, then pitch about y, then roll about x) over feet that stay where the gait puts them,
// standing or walking. Every foot target goes through the inverse before the IK. Asked-for poses
// are clamped to PoseLimits and reached at its rates (no jerk on the servos), and only as far as
// every foot stays reachable with no pulse clamped: each axis alone reaches its limit standing,
// all six at once about 70 % of them, so a combined pose stops where the legs end. While walking
// the target is scaled to `walking` (every corner reachable at top speed). Lying down (idle or the
// final shutdown) eases the pose out first and back in after.
struct BodyPose {
    float x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
};
struct PoseLimits {
    float xy = 0.025f, z = 0.020f;  // m
    float angle = 0.2618f;          // rad (15 deg), each of roll, pitch, yaw
    float speed = 0.04f;            // m/s
    float turn = 0.5f;              // rad/s
    float walking = 0.2f;           // the share of the pose kept while the gait walks
};
BodyPose clamp_pose(const BodyPose& p, const PoseLimits& l = PoseLimits());
// The foot target f (stance frame: where the neutral body's feet are) seen from the posed body.
Vec3 to_posed_body(const BodyPose& p, const Vec3& f);
// And back: a point in the posed body's frame, in the stance frame.
Vec3 from_posed_body(const BodyPose& p, const Vec3& b);

// Tilt feedback: on the robot the body reaches ~70 % of a commanded roll or pitch (the legs and servos
// give under the load). Standing, the integral of (commanded - measured) roll and pitch, the
// measured being the IMU's (gravity-referenced), is added to the pose's roll and pitch
// (SimRobot::set_tilt_trim), capped; inside the dead band it holds still, so a settled pose stops
// moving. So a pose's roll and pitch are relative to gravity: a neutral pose is a level body, on a
// slope or straddling a step. Off (and reset) while walking or lying down. Gain 0: off.
struct TiltHoldParams {
    float gain = 1.5f;        // 1/s: trim change per second per rad of error
    float max_trim = 0.14f;   // rad (8 deg), each axis
    float deadband = 0.005f;  // rad (0.3 deg)
};

class TiltHold {
public:
    explicit TiltHold(const TiltHoldParams& p = TiltHoldParams()) : p_(p) {}
    void set_gain(float g) { p_.gain = g; }
    const TiltHoldParams& params() const { return p_; }
    // One tick; returns whether the trims changed.
    bool update(float cmd_roll, float cmd_pitch, float meas_roll, float meas_pitch, bool active, float dt) {
        if (!active || p_.gain <= 0) {
            const bool was = roll_ != 0 || pitch_ != 0;
            roll_ = pitch_ = 0;
            return was;
        }
        const float er = cmd_roll - meas_roll, ep = cmd_pitch - meas_pitch;
        const float r0 = roll_, p0 = pitch_;
        if (er > p_.deadband || er < -p_.deadband) roll_ = cap(roll_ + p_.gain * er * dt);
        if (ep > p_.deadband || ep < -p_.deadband) pitch_ = cap(pitch_ + p_.gain * ep * dt);
        return roll_ != r0 || pitch_ != p0;
    }
    float roll() const { return roll_; }
    float pitch() const { return pitch_; }

private:
    float cap(float v) const { return v > p_.max_trim ? p_.max_trim : v < -p_.max_trim ? -p_.max_trim : v; }
    TiltHoldParams p_;
    float roll_ = 0, pitch_ = 0;
};

struct RestParams {
    float reach = 0.075f;  // hip-to-foot, horizontal
    float z = -0.015f;     // foot height in the body frame (stand_z -0.060): the floor under the belly
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
    // YawHold's trim (TripodGait::set_yaw_trim).
    void set_yaw_trim(float wz) { gait_.set_yaw_trim(wz); }
    // The body pose to go to (clamped; zero: neutral). pose(): where it is now, as applied (the
    // rest level eases it out); pose_settled(): at the target and not lying down or between.
    void set_pose(const BodyPose& p) { pose_target_ = clamp_pose(p); }
    // TiltHold's trims, added to the target's roll and pitch (within the limits, and the reach gate).
    void set_tilt_trim(float roll, float pitch) {
        tilt_roll_ = roll;
        tilt_pitch_ = pitch;
    }
    const BodyPose& pose_target() const { return pose_target_; }
    const BodyPose& pose() const { return pose_applied_; }
    // The pose did not move this tick (the gyro's bias may be learned).
    bool pose_settled() const { return !pose_moved_; }
    // The pose has reached its target, trims included (not still on its way at the limits' rates):
    // TiltHold integrates only then, so it does not wind up on the ramp.
    bool pose_at_target() const { return pose_at_target_; }
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
    // Static stability: how far the centre of mass is inside the support polygon (the convex hull
    // of the feet on the ground), metres; negative: outside (the robot would tip), -1 with fewer
    // than three feet down. The centre of mass is the body's origin plus set_com()'s offset
    // (body frame; the real one is unmeasured: the battery and the camera move it).
    float margin() const { return margin_; }
    // The balance probe: one foot moved off its stance spot (stance frame, m: |dx|, |dy| <= 0.02,
    // 0 <= dz <= 0.03) while the gait stands and the robot is up; it moves there at 20 mm/s and back
    // the same way. Lifted, the robot stands on the others: a body pose then shows where the centre
    // of mass tips it. False (nothing changed) when out of range, walking or lying; leg -1: back down.
    // A command that moves, or the gait stepping, puts it back.
    bool set_foot_offset(int leg, const Vec3& d);
    int foot_offset_leg() const { return off_leg_; }
    // A foot is off its spot or on its way back (the tilt hold waits: it would level the tip away).
    bool foot_offset_active() const { return off_leg_ >= 0; }
    void set_com(float x, float y) { com_x_ = x; com_y_ = y; }

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
    BodyPose pose_target_, pose_, pose_applied_;
    float tilt_roll_ = 0, tilt_pitch_ = 0;
    bool pose_moved_ = false, pose_at_target_ = true;
    bool feet_fit(const BodyPose& applied, float rest) const;
    Vec3 foot_target(int leg, float rest) const;
    bool rest_want_ = false;
    float margin_ = 0, com_x_ = 0, com_y_ = 0;
    int off_leg_ = -1;
    Vec3 off_{0, 0, 0}, off_target_{0, 0, 0};
    float rest_s_ = 0;
};

}  // namespace hexa
