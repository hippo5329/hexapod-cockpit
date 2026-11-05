#include "hexa_gait.h"

#include <cmath>

namespace hexa {

namespace {

constexpr float kSettleEps = 0.001f;   // 1 mm
constexpr float kZeroCmd = 1e-4f;

// Tripod A (rf, rr, lm) starts in stance, tripod B half a cycle later.
constexpr float kOffset[kLegs] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};

float frac(float v) { return v - std::floor(v); }

// Stance velocity of a foot at p: -(v + w x p).
void stance_vel(const BodyTwist& t, const Vec3& p, float* vx, float* vy) {
    *vx = -(t.vx - t.wz * p.y);
    *vy = -(t.vy + t.wz * p.x);
}

}  // namespace

void TripodGait::init(const Geometry* g, const GaitParams& p) {
    g_ = g;
    p_ = p;
    cmd_ = {0, 0, 0};
    cmd_time_ = -1e9f;
    phase_ = 0.0f;
    standing_ = true;
    for (int i = 0; i < kLegs; ++i) {
        foot_[i] = neutral_foot(*g_, i);
        swing_from_[i] = foot_[i];
        landing_[i] = foot_[i];
        was_stance_[i] = in_stance(i);
    }
}

BodyTwist TripodGait::clamp(const BodyTwist& c) const {
    const float t_stance = p_.duty * p_.period_s;
    float worst = 0.0f;
    for (int i = 0; i < kLegs; ++i) {
        float vx, vy;
        stance_vel(c, neutral_foot(*g_, i), &vx, &vy);
        worst = std::fmax(worst, std::sqrt(vx * vx + vy * vy) * t_stance);
    }
    if (worst <= p_.max_stride) return c;
    const float k = p_.max_stride / worst;
    return {c.vx * k, c.vy * k, c.wz * k};
}

void TripodGait::command(const BodyTwist& cmd, float now_s) {
    cmd_ = clamp(cmd);
    cmd_time_ = now_s;
}

float TripodGait::leg_phase(int leg) const { return frac(phase_ + kOffset[leg]); }

bool TripodGait::params_valid(const GaitParams& p) {
    // Duty above 0.5 overlaps the tripods' stances (both down at each change); at or below
    // 0.5 there is an instant with three feet or fewer. The rest bound what the legs reach.
    return p.period_s >= 0.5f && p.period_s <= 3.0f && p.duty >= 0.5f && p.duty <= 0.8f && p.lift >= 0.01f &&
           p.lift <= 0.04f && p.max_stride >= 0.01f && p.max_stride <= 0.06f && p.cmd_timeout_s > 0.0f;
}

bool TripodGait::set_params(const GaitParams& p) {
    if (!standing_ || !params_valid(p)) return false;
    p_ = p;
    return true;
}

bool TripodGait::in_stance(int leg) const { return leg_phase(leg) < p_.duty; }

bool TripodGait::settled() const {
    for (int i = 0; i < kLegs; ++i) {
        const Vec3 n = neutral_foot(*g_, i);
        if (std::fabs(foot_[i].x - n.x) > kSettleEps || std::fabs(foot_[i].y - n.y) > kSettleEps ||
            std::fabs(foot_[i].z - n.z) > kSettleEps)
            return false;
    }
    return true;
}

BodyTwist TripodGait::applied() const {
    if (standing_) return {0, 0, 0};
    return cmd_;
}

void TripodGait::step(float dt, float now_s) {
    if (now_s - cmd_time_ > p_.cmd_timeout_s) cmd_ = {0, 0, 0};
    const bool moving = std::fabs(cmd_.vx) > kZeroCmd || std::fabs(cmd_.vy) > kZeroCmd ||
                        std::fabs(cmd_.wz) > kZeroCmd;
    if (!moving && settled()) {
        standing_ = true;
        idle_s_ = 0;
        return;
    }
    const float next = frac(phase_ + dt / p_.period_s);
    if (!moving) {
        // At a duty of 0.5 one tripod lifts on the very tick the other lands, so the check above
        // never sees all six feet down at once and the gait would step in place forever (it did,
        // on the robot, at 0.8 s / 0.5). Look at the tick ahead instead: the feet landing on it
        // and the feet about to lift (still down) all on neutral is a stand.
        bool down = true;
        for (int i = 0; i < kLegs && down; ++i) {
            const bool stance_next = frac(next + kOffset[i]) < p_.duty;
            if (!stance_next && !was_stance_[i]) down = false;  // mid-swing
            const Vec3& p = stance_next && !was_stance_[i] ? landing_[i] : foot_[i];
            const Vec3 n = neutral_foot(*g_, i);
            down = down && std::fabs(p.x - n.x) <= kSettleEps && std::fabs(p.y - n.y) <= kSettleEps &&
                   (was_stance_[i] || stance_next);
        }
        idle_s_ += dt;
        // The backstop: still stepping two cycles after the command went to zero (no case left
        // known): put every foot down on neutral and stand. forced_stands() counts it.
        if (down || idle_s_ > 2 * p_.period_s) {
            if (!down) ++forced_stands_;
            for (int i = 0; i < kLegs; ++i) {
                foot_[i] = neutral_foot(*g_, i);
                landing_[i] = swing_from_[i] = foot_[i];
            }
            phase_ = next;
            for (int i = 0; i < kLegs; ++i) was_stance_[i] = true;
            standing_ = true;
            idle_s_ = 0;
            return;
        }
    } else {
        idle_s_ = 0;
    }
    standing_ = false;
    phase_ = next;

    const float t_stance = p_.duty * p_.period_s;
    for (int i = 0; i < kLegs; ++i) {
        const bool stance = in_stance(i);
        if (stance) {
            if (!was_stance_[i]) {
                // Touch-down: the last swing tick stops short of s = 1, so snap onto the
                // landing point instead of leaving the foot a few mm off it.
                foot_[i] = landing_[i];
            }
            float vx, vy;
            stance_vel(cmd_, foot_[i], &vx, &vy);
            foot_[i].x += vx * dt;
            foot_[i].y += vy * dt;
            foot_[i].z = g_->stand_z;
        } else {
            if (was_stance_[i]) swing_from_[i] = foot_[i];  // lift-off
            const float s = (leg_phase(i) - p_.duty) / (1.0f - p_.duty);  // 0..1
            // Land ahead of neutral by half a stance of travel, so the stance is centred.
            const Vec3 n = neutral_foot(*g_, i);
            float vx, vy;
            stance_vel(cmd_, n, &vx, &vy);
            const float lx = n.x - vx * 0.5f * t_stance;
            const float ly = n.y - vy * 0.5f * t_stance;
            landing_[i] = {lx, ly, g_->stand_z};
            const float e = 0.5f - 0.5f * std::cos(static_cast<float>(M_PI) * s);  // ease in/out
            foot_[i].x = swing_from_[i].x + (lx - swing_from_[i].x) * e;
            foot_[i].y = swing_from_[i].y + (ly - swing_from_[i].y) * e;
            foot_[i].z = g_->stand_z + p_.lift * std::sin(static_cast<float>(M_PI) * s);
        }
        was_stance_[i] = stance;
    }
}

}  // namespace hexa
