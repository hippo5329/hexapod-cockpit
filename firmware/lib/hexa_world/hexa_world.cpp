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
            cal_[k] = default_cal(k);
            servo_[k].reset(cal_[k], ang[j]);
            commanded_[k] = ang[j];
            pulse_us_[k] = angle_to_us(cal_[k], ang[j]);
        }
    }
    actual_feet(feet_, contact_);
    reset_poses();
    return true;
}

void SimRobot::set_trims(const int16_t* by_servo_id) {
    for (int k = 0; k < kJoints; ++k) {
        cal_[k].trim_us = by_servo_id[servo_id(k) - 1];
        servo_[k].reset(cal_[k], servo_[k].angle());
        pulse_us_[k] = angle_to_us(cal_[k], commanded_[k]);
    }
}

void SimRobot::actual_feet(Vec3* out, bool* contact) const {
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a{servo_[leg * 3].angle(), servo_[leg * 3 + 1].angle(), servo_[leg * 3 + 2].angle()};
        out[leg] = leg_fk(*g_, leg, a);
        contact[leg] = out[leg].z <= g_->stand_z + 0.003f;
    }
}

void SimRobot::command(const BodyTwist& cmd, float now_s) {
    if (cmd.vx != 0 || cmd.vy != 0 || cmd.wz != 0) rest_want_ = false;
    gait_.command(rest_s_ > 0 ? BodyTwist{0, 0, 0} : cmd, now_s);
}

StepCounts SimRobot::step(float dt, float now_s) {
    StepCounts n;
    gait_.step(dt, now_s);
    if (rest_want_ && gait_.standing()) rest_s_ = std::fmin(1.0f, rest_s_ + dt / rp_.time_s);
    else if (!rest_want_) rest_s_ = std::fmax(0.0f, rest_s_ - dt / rp_.time_s);
    const float r = rest_s_ * rest_s_ * (3 - 2 * rest_s_);  // smoothstep: no jerk at either end
    const BodyTwist ap = gait_.applied();
    odom_.apply_body(ap.vx * dt, ap.vy * dt, ap.wz * dt);

    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        Vec3 f = gait_.foot(leg);
        if (r > 0) {
            // Resting only from a stance: the feet are on their neutral spots.
            const Hip& h = g_->hip[leg];
            const float k = rp_.reach / g_->neutral_reach;
            const Vec3 down{h.x + (f.x - h.x) * k, h.y + (f.y - h.y) * k, rp_.z};
            f = {f.x + (down.x - f.x) * r, f.y + (down.y - f.y) * r, f.z + (down.z - f.z) * r};
        }
        if (!leg_ik(*g_, leg, f, &a)) { ++n.unreachable; continue; }
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
