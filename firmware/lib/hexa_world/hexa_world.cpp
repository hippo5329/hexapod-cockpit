#include "hexa_world.h"

#include <cmath>

namespace hexa {

void Pose2::apply_body(double dx, double dy, double dyaw) {
    const double c = std::cos(yaw), s = std::sin(yaw);
    x += c * dx - s * dy;
    y += s * dx + c * dy;
    yaw = std::remainder(yaw + dyaw, 2 * M_PI);
}

OdomVariance gait_odom_variance(bool standing) {
    OdomVariance v = {{0.01, 0.01, 1e6, 1e6, 1e6, 0.05}, {0.004, 0.004, 1e6, 1e6, 1e6, 0.02}};
    if (standing) v.twist[0] = v.twist[1] = v.twist[5] = 1e-8;  // gyro white noise: 2e-6 (rad/s)^2
    return v;
}

bool rigid_2d(const Vec3* before, const Vec3* now, const bool* use, double* dx, double* dy, double* dyaw) {
    int n = 0;
    double ax = 0, ay = 0, bx = 0, by = 0;
    for (int i = 0; i < kLegs; ++i)
        if (use[i]) {
            ax += before[i].x; ay += before[i].y; bx += now[i].x; by += now[i].y; ++n;
        }
    if (n < 2) return false;
    ax /= n; ay /= n; bx /= n; by /= n;
    double sxx = 0, sxy = 0;
    for (int i = 0; i < kLegs; ++i)
        if (use[i]) {
            const double px = now[i].x - bx, py = now[i].y - by;
            const double qx = before[i].x - ax, qy = before[i].y - ay;
            sxx += px * qx + py * qy;
            sxy += px * qy - py * qx;
        }
    const double th = std::atan2(sxy, sxx);
    const double c = std::cos(th), s = std::sin(th);
    *dyaw = th;
    *dx = ax - (c * bx - s * by);
    *dy = ay - (s * bx + c * by);
    return true;
}

bool SimRobot::init(double slip, const GaitParams& gp, const SimServoParams& sp) {
    g_ = &minihexa_geometry();
    slip_ = slip;
    gp_ = gp;
    sp_ = sp;
    gait_.init(g_, gp_);
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        if (!leg_ik(*g_, leg, gait_.foot(leg), &a)) return false;
        const float ang[3] = {a.coxa, a.femur, a.tibia};
        for (int j = 0; j < 3; ++j) {
            const int k = leg * 3 + j;
            cal_[k] = default_cal(k);  // TODO: trims from NVS (servo_trim tool)
            servo_[k].reset(cal_[k], ang[j]);
            commanded_[k] = ang[j];
            pulse_us_[k] = angle_to_us(cal_[k], ang[j]);
        }
    }
    actual_feet(feet_, contact_);
    reset_poses();
    return true;
}

void SimRobot::actual_feet(Vec3* out, bool* contact) const {
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a{servo_[leg * 3].angle(), servo_[leg * 3 + 1].angle(), servo_[leg * 3 + 2].angle()};
        out[leg] = leg_fk(*g_, leg, a);
        contact[leg] = out[leg].z <= g_->stand_z + 0.003f;
    }
}

StepCounts SimRobot::step(float dt, float now_s) {
    StepCounts n;
    gait_.step(dt, now_s);
    const BodyTwist ap = gait_.applied();
    odom_.apply_body(ap.vx * dt, ap.vy * dt, ap.wz * dt);

    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        if (!leg_ik(*g_, leg, gait_.foot(leg), &a)) { ++n.unreachable; continue; }
        const float ang[3] = {a.coxa, a.femur, a.tibia};
        for (int j = 0; j < 3; ++j) {
            const int k = leg * 3 + j;
            bool cl = false;
            commanded_[k] = ang[j];
            pulse_us_[k] = angle_to_us(cal_[k], ang[j], &cl);
            if (cl) ++n.clamped;
            servo_[k].command_us(pulse_us_[k]);
        }
    }
    for (SimServo& s : servo_) s.step(dt, sp_);

    Vec3 feet[kLegs];
    bool contact[kLegs], both[kLegs];
    actual_feet(feet, contact);
    for (int i = 0; i < kLegs; ++i) both[i] = contact[i] && contact_[i];
    double dx = 0, dy = 0, dyaw = 0;
    if (rigid_2d(feet_, feet, both, &dx, &dy, &dyaw)) {
        dx *= 1 - slip_; dy *= 1 - slip_; dyaw *= 1 - slip_;
        truth_.apply_body(dx, dy, dyaw);
    } else {
        dx = dy = dyaw = 0;
    }
    truth_twist_ = {static_cast<float>(dx / dt), static_cast<float>(dy / dt), static_cast<float>(dyaw / dt)};
    for (int i = 0; i < kLegs; ++i) { feet_[i] = feet[i]; contact_[i] = contact[i]; }
    return n;
}

}  // namespace hexa
