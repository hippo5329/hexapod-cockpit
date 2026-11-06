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

BodyPose clamp_pose(const BodyPose& p, const PoseLimits& l) {
    auto c = [](float v, float m) { return v > m ? m : v < -m ? -m : v; };
    return {c(p.x, l.xy), c(p.y, l.xy), c(p.z, l.z), c(p.roll, l.angle), c(p.pitch, l.angle), c(p.yaw, l.angle)};
}

namespace {
// R = Rz(yaw) Ry(pitch) Rx(roll), applied to v (or its transpose, inverse = true).
Vec3 rotate(const BodyPose& p, const Vec3& v, bool inverse) {
    const float cr = std::cos(p.roll), sr = std::sin(p.roll), cp = std::cos(p.pitch), sp = std::sin(p.pitch),
                cy = std::cos(p.yaw), sy = std::sin(p.yaw);
    const float r[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                           {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                           {-sp, cp * sr, cp * cr}};
    const float in[3] = {v.x, v.y, v.z};
    float o[3];
    for (int i = 0; i < 3; ++i)
        o[i] = inverse ? r[0][i] * in[0] + r[1][i] * in[1] + r[2][i] * in[2]
                       : r[i][0] * in[0] + r[i][1] * in[1] + r[i][2] * in[2];
    return {o[0], o[1], o[2]};
}

float approach(float v, float to, float step) { return v < to ? std::fmin(to, v + step) : std::fmax(to, v - step); }

// Signed distance of (px, py) inside the convex hull of the contact feet (x, y): the least over the
// hull's edges; -1 with fewer than three feet down.
float support_margin(const Vec3* feet, const bool* contact, float px, float py) {
    Vec3 p[kLegs];
    int n = 0;
    for (int i = 0; i < kLegs; ++i)
        if (contact[i]) p[n++] = feet[i];
    if (n < 3) return -1;
    // Monotone chain, counter-clockwise.
    for (int i = 1; i < n; ++i)
        for (int j = i; j > 0 && (p[j].x < p[j - 1].x || (p[j].x == p[j - 1].x && p[j].y < p[j - 1].y)); --j) {
            const Vec3 t = p[j]; p[j] = p[j - 1]; p[j - 1] = t;
        }
    auto cross = [](const Vec3& o, const Vec3& a, const Vec3& b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
    Vec3 h[2 * kLegs];
    int k = 0;
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
        h[k++] = p[i];
    }
    for (int i = n - 2, lo = k + 1; i >= 0; --i) {
        while (k >= lo && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
        h[k++] = p[i];
    }
    --k;  // the last point is the first
    if (k < 3) return -1;  // the feet in a line
    float m = 1e9f;
    for (int i = 0; i < k; ++i) {
        const Vec3& a = h[i];
        const Vec3& b = h[(i + 1) % k];
        const float ex = b.x - a.x, ey = b.y - a.y, len = std::sqrt(ex * ex + ey * ey);
        m = std::fmin(m, (ex * (py - a.y) - ey * (px - a.x)) / len);  // left of a->b is inside
    }
    return m;
}
}  // namespace

Vec3 to_posed_body(const BodyPose& p, const Vec3& f) { return rotate(p, {f.x - p.x, f.y - p.y, f.z - p.z}, true); }

Vec3 from_posed_body(const BodyPose& p, const Vec3& b) {
    const Vec3 v = rotate(p, b, false);
    return {v.x + p.x, v.y + p.y, v.z + p.z};
}

// Where leg's foot goes in the stance frame this tick: the gait's, drawn in for lying down.
Vec3 SimRobot::foot_target(int leg, float r) const {
    Vec3 f = gait_.foot(leg);
    if (r > 0) {
        // Resting only from a stance: the feet are on their neutral spots.
        const Hip& h = g_->hip[leg];
        const float k = rp_.reach / g_->neutral_reach;
        const Vec3 down{h.x + (f.x - h.x) * k, h.y + (f.y - h.y) * k, rp_.z};
        f = {f.x + (down.x - f.x) * r, f.y + (down.y - f.y) * r, f.z + (down.z - f.z) * r};
    }
    return f;
}

// Every foot reachable, with no pulse clamped, through this pose.
bool SimRobot::feet_fit(const BodyPose& applied, float r) const {
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        if (!leg_ik(*g_, leg, to_posed_body(applied, foot_target(leg, r)), &a)) return false;
        const float ang[3] = {a.coxa, a.femur, a.tibia};
        for (int j = 0; j < 3; ++j) {
            bool cl = false;
            angle_to_us(cal_[leg * 3 + j], ang[j], &cl);
            if (cl) return false;
        }
    }
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
        out[leg] = from_posed_body(pose_applied_, leg_fk(*g_, leg, a));
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
    // The pose moves toward its target (a share of it while walking) at the limits' rates, only as
    // far as the feet fit, and is eased out while lying down (the rest pose is a neutral body's).
    const PoseLimits lim;
    const float ds = lim.speed * dt, da = lim.turn * dt, w = gait_.standing() ? 1.0f : lim.walking;
    BodyPose t = pose_target_;
    t.roll += tilt_roll_;
    t.pitch += tilt_pitch_;
    t = clamp_pose(t, lim);
    const BodyPose next = {approach(pose_.x, t.x * w, ds),        approach(pose_.y, t.y * w, ds),
                           approach(pose_.z, t.z * w, ds),        approach(pose_.roll, t.roll * w, da),
                           approach(pose_.pitch, t.pitch * w, da), approach(pose_.yaw, t.yaw * w, da)};
    const float keep = 1 - r;
    auto scaled = [keep](const BodyPose& p) {
        return BodyPose{p.x * keep, p.y * keep, p.z * keep, p.roll * keep, p.pitch * keep, p.yaw * keep};
    };
    const BodyPose before = pose_applied_;
    // Toward the target only if the feet fit there; moving back toward neutral (smaller) always.
    auto norm = [](const BodyPose& p) {
        return std::fabs(p.x) + std::fabs(p.y) + std::fabs(p.z) + std::fabs(p.roll) + std::fabs(p.pitch) + std::fabs(p.yaw);
    };
    if (norm(next) <= norm(pose_) || feet_fit(scaled(next), r)) pose_ = next;
    // Within one tick's step of the target (the trims move it a little every tick).
    pose_at_target_ = r == 0 && std::fabs(pose_.roll - t.roll * w) <= da && std::fabs(pose_.pitch - t.pitch * w) <= da &&
                      std::fabs(pose_.yaw - t.yaw * w) <= da && std::fabs(pose_.x - t.x * w) <= ds &&
                      std::fabs(pose_.y - t.y * w) <= ds && std::fabs(pose_.z - t.z * w) <= ds;
    pose_applied_ = scaled(pose_);
    pose_moved_ = before.x != pose_applied_.x || before.y != pose_applied_.y || before.z != pose_applied_.z ||
                  before.roll != pose_applied_.roll || before.pitch != pose_applied_.pitch || before.yaw != pose_applied_.yaw;

    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        const Vec3 f = to_posed_body(pose_applied_, foot_target(leg, r));
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
    actual_feet(feet, contact);  // in the stance frame (the pose undone)
    for (int i = 0; i < kLegs; ++i) both[i] = contact[i] && contact_[i];
    // The centre of mass in the stance frame: the posed body's origin plus the offset.
    {
        const Vec3 com = from_posed_body(pose_applied_, {com_x_, com_y_, 0});
        margin_ = support_margin(feet, contact, com.x, com.y);
    }
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
