// The host simulator's world: the firmware's gait, IK and servo mapping driving
// simulated servos on a flat floor, with the body's true pose worked out from the
// feet. Shared by hexa_sim (scenarios) and the ROS 2 node (ros/hexapod_sim).
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

struct StepCounts {
    int unreachable = 0;  // legs whose foot target IK could not reach (they keep their angles)
    int clamped = 0;      // joint pulses clamped to the servo range
};

class SimRobot {
public:
    // Standing on the gait's neutral stance, servos already there. False if that
    // stance is unreachable (a geometry fault).
    bool init(double slip = 0.05, const GaitParams& gp = GaitParams(), const SimServoParams& sp = SimServoParams());

    void command(const BodyTwist& cmd, float now_s) { gait_.command(cmd, now_s); }
    StepCounts step(float dt, float now_s);

    float joint_angle(int k) const { return servo_[k].angle(); }
    const Pose2& truth() const { return truth_; }
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
    Vec3 feet_[kLegs];
    bool contact_[kLegs];
    Pose2 truth_, odom_;
};

}  // namespace hexa
