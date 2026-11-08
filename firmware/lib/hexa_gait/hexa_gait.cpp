#include "hexa_gait.h"

#include <cmath>
#include <initializer_list>

namespace hexa {

namespace {

constexpr float kSettleEps = 0.001f;   // 1 mm
constexpr float kZeroCmd = 1e-4f;

// Tripod A (rf, rr, lm) starts in stance, tripod B half a cycle later.
constexpr float kOffset[kLegs] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
constexpr int kCorners[4] = {0, 2, 3, 5};  // rf, rr, lr, lf: the quadruped's legs

float frac(float v) { return v - std::floor(v); }

// A half-plane n.p >= c (n unit).
struct Half {
    float nx, ny, c;
};

bool inside(const Half* h, int n, float x, float y) {
    for (int i = 0; i < n; ++i)
        if (h[i].nx * x + h[i].ny * y < h[i].c - 1e-6f) return false;
    return true;
}

// The point of the half-planes' intersection nearest the origin (the least sway); false if empty.
bool nearest_inside(const Half* h, int n, float* px, float* py) {
    if (inside(h, n, 0, 0)) {
        *px = *py = 0;
        return true;
    }
    float best = 1e9f;
    auto take = [&](float x, float y) {
        if (inside(h, n, x, y) && x * x + y * y < best) {
            best = x * x + y * y;
            *px = x;
            *py = y;
        }
    };
    for (int i = 0; i < n; ++i) take(h[i].nx * h[i].c, h[i].ny * h[i].c);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            const float det = h[i].nx * h[j].ny - h[i].ny * h[j].nx;
            if (std::fabs(det) < 1e-6f) continue;
            take((h[i].c * h[j].ny - h[i].ny * h[j].c) / det, (h[i].nx * h[j].c - h[i].c * h[j].nx) / det);
        }
    return best < 1e9f;
}

// The triangle of three feet as three half-planes, each edge moved in by m.
void triangle(const Vec3& a, const Vec3& b, const Vec3& c, float m, Half* out) {
    const Vec3* v[3] = {&a, &b, &c};
    for (int e = 0; e < 3; ++e) {
        const Vec3& p = *v[e];
        const Vec3& q = *v[(e + 1) % 3];
        const Vec3& r = *v[(e + 2) % 3];
        float nx = -(q.y - p.y), ny = q.x - p.x;
        const float len = std::sqrt(nx * nx + ny * ny);
        nx /= len;
        ny /= len;
        if (nx * (r.x - p.x) + ny * (r.y - p.y) < 0) {
            nx = -nx;
            ny = -ny;
        }
        out[e] = {nx, ny, nx * p.x + ny * p.y + m};
    }
}

// Stance velocity of a foot at p: -(v + w x p).
void stance_vel(const BodyTwist& t, const Vec3& p, float* vx, float* vy) {
    *vx = -(t.vx - t.wz * p.y);
    *vy = -(t.vy + t.wz * p.x);
}

}  // namespace

GaitParams quad_gait_params() {
    GaitParams p;
    p.quad = true;
    p.period_s = 2.5f;
    p.max_period_s = 4.0f;
    p.duty = 0.88f;
    p.max_stride = 0.045f;
    p.margin = 0.020f;
    p.sway_speed = 0.12f;
    return p;
}

// The high walk (sim/tools/high_walk_sim.cpp, 2026-11-08: over cables and pipes, user): the body 20 mm up
// (belly 65 mm off the floor, was 45) and the swing 50 mm high, so a cable or a pipe up to about 45 mm
// passes under the feet and the body; the stride 45 mm (65 less the height), a 1.0 s cycle for the
// longer swing. Top speed 64 mm/s; every servo 260 us or more inside its range.
GaitParams high_gait_params() {
    GaitParams p;
    p.period_s = 1.0f;
    p.height = 0.020f;
    p.lift = 0.050f;
    p.max_stride = 0.045f;
    return p;
}

// The climb pose: the highest body the legs still walk with (40 mm up: belly 85 mm off the floor) and
// the highest swing under it (80 mm), for stepping onto something: a foot reaches 80 mm above the
// floor. The stride is 25 mm (the legs nearly straight), a slow 2.0 s cycle, longer on the ground
// (duty 0.75); top speed 17 mm/s. The gait does not feel the step: a foot landing on it is a foot
// landing high, which tilts the body until it steps off again.
GaitParams climb_gait_params() {
    GaitParams p;
    p.period_s = 2.0f;
    p.max_period_s = 4.0f;
    p.duty = 0.75f;
    p.height = 0.040f;
    p.lift = 0.080f;
    p.max_stride = 0.025f;
    return p;
}

// The tripod from phase 0; the quadruped from the gap after rf's step (0.5 + 2 (1 - duty)).
float TripodGait::start_phase() const { return p_.quad ? frac(2.5f - 2 * p_.duty) : 0.0f; }

float TripodGait::offset(int leg) const {
    if (!p_.quad) return kOffset[leg];
    // Lift-off (phase = duty) of lr at 0, lf right after it, rr at half a cycle, rf right after.
    const float d = p_.duty;
    switch (leg) {
        case 3: return frac(d);
        case 5: return frac(2 * d - 1);
        case 2: return frac(d - 0.5f);
        case 0: return frac(2 * d - 1.5f);
        default: return 0;
    }
}

Vec3 TripodGait::neutral(int leg) const {
    const Vec3 n = neutral_foot(*g_, leg);
    return {n.x, n.y, z_};
}

Vec3 TripodGait::home(int leg) const {
    if (walks(leg)) return neutral(leg);
    const Hip& h = g_->hip[leg];
    return {h.x + p_.mid_reach * std::cos(h.mount_yaw), h.y + p_.mid_reach * std::sin(h.mount_yaw),
            z_ + p_.mid_lift};
}

bool TripodGait::mids_home() const {
    for (int leg : {1, 4}) {
        const Vec3 h = home(leg);
        if (std::fabs(foot_[leg].x - h.x) > kSettleEps || std::fabs(foot_[leg].y - h.y) > kSettleEps ||
            std::fabs(foot_[leg].z - h.z) > kSettleEps)
            return false;
    }
    return true;
}

Vec3 TripodGait::foot(int leg) const {
    if (!walks(leg)) return foot_[leg];
    return {foot_[leg].x - sway_x_, foot_[leg].y - sway_y_, foot_[leg].z};
}

// The sway's target: the body's centre `margin` inside the triangle of the feet down while the
// swinging leg is up, and (if both can hold) the triangle for the leg that lifts next; the least
// sway that does. Then toward it at sway_speed.
void TripodGait::sway(float dt) {
    if (!p_.quad) return;
    int swinging = -1, next = -1;
    float soonest = 2;
    for (int leg : kCorners) {
        if (!in_stance(leg)) {
            swinging = leg;
            continue;
        }
        const float to_lift = p_.duty - leg_phase(leg);  // stance: phase < duty
        if (to_lift < soonest) {
            soonest = to_lift;
            next = leg;
        }
    }
    // The feet relative to the centre of mass at zero sway: the sway is then where it must be.
    auto tri = [&](int up, Half* out) {
        Vec3 v[3];
        int n = 0;
        for (int leg : kCorners)
            if (leg != up) v[n++] = {foot_[leg].x - p_.com_x, foot_[leg].y - p_.com_y, foot_[leg].z};
        triangle(v[0], v[1], v[2], p_.margin, out);
    };
    Half h[6];
    int n = 0;
    if (swinging >= 0) {
        tri(swinging, h);
        n = 3;
    }
    float tx = 0, ty = 0;
    if (next >= 0) {
        tri(next, h + n);
        if (!nearest_inside(h, n + 3, &tx, &ty) && !(n && nearest_inside(h, n, &tx, &ty))) tx = ty = 0;
    } else if (n && !nearest_inside(h, n, &tx, &ty)) {
        tx = ty = 0;
    }
    const float dx = tx - sway_x_, dy = ty - sway_y_, d = std::sqrt(dx * dx + dy * dy), step = p_.sway_speed * dt;
    if (d <= step) {
        sway_x_ = tx;
        sway_y_ = ty;
    } else {
        sway_x_ += dx * step / d;
        sway_y_ += dy * step / d;
    }
}

void TripodGait::init(const Geometry* g, const GaitParams& p) {
    g_ = g;
    p_ = p;
    raw_ = cmd_ = {0, 0, 0};
    z_ = g->stand_z - p.height;
    period_ = p.period_s;
    cmd_time_ = -1e9f;
    phase_ = start_phase();
    standing_ = true;
    sway_x_ = sway_y_ = 0;
    for (int i = 0; i < kLegs; ++i) {
        foot_[i] = home(i);
        swing_from_[i] = foot_[i];
        landing_[i] = foot_[i];
        was_stance_[i] = in_stance(i);
    }
}

float TripodGait::worst_foot_speed(const BodyTwist& c) const {
    float worst = 0.0f;
    for (int i = 0; i < kLegs; ++i) {
        if (!walks(i)) continue;
        float vx, vy;
        stance_vel(c, neutral(i), &vx, &vy);
        worst = std::fmax(worst, std::sqrt(vx * vx + vy * vy));
    }
    return worst;
}

BodyTwist TripodGait::clamp(const BodyTwist& c) const {
    const float worst = worst_foot_speed(c) * p_.duty * period_;
    if (worst <= p_.max_stride) return c;
    const float k = p_.max_stride / worst;
    return {c.vx * k, c.vy * k, c.wz * k};
}

// The share of c (0..1) that keeps every stance foot within a stride of its neutral spot at its
// lift-off. A command that changes mid-stance moves a foot on from where the last one left it: a
// reversal carried a foot landed half a stride ahead a full stride further, 1.5 strides out, past
// the reach of a raised body (sim/tools/high_walk_sim.cpp). A stride out is what a start from
// standing takes (its first stance begins on neutral), so a steady walk is never slowed. Only the
// end is checked: the path there is a straight line, and the disc is convex.
float TripodGait::reach_scale(const BodyTwist& c) const {
    const float r2 = p_.max_stride * p_.max_stride * 1.0001f;
    float k = 1.0f;
    for (int i = 0; i < kLegs; ++i) {
        if (!in_stance(i)) continue;
        const float tau = (p_.duty - leg_phase(i)) * period_;  // to lift-off
        float vx, vy;
        stance_vel(c, foot_[i], &vx, &vy);
        const Vec3 n = neutral(i);
        const float dx = foot_[i].x - n.x, dy = foot_[i].y - n.y, wx = vx * tau, wy = vy * tau;
        const float a = wx * wx + wy * wy, b = 2 * (dx * wx + dy * wy), cc = dx * dx + dy * dy - r2;
        if (a < 1e-12f) continue;
        if (cc > 0) {  // already out (it should not be): only inward
            if (b >= 0) k = 0;
            continue;
        }
        k = std::fmin(k, (-b + std::sqrt(b * b - 4 * a * cc)) / (2 * a));  // the root past zero
    }
    return std::fmax(0.0f, k);
}

void TripodGait::command(const BodyTwist& cmd, float now_s) {
    raw_ = worst_foot_speed(cmd) < p_.min_foot_speed ? BodyTwist{0, 0, 0} : cmd;
    if (standing_) {
        // From a stand the cycle starts at the one this command wants (adaptive cadence, step()).
        const float v = worst_foot_speed(raw_);
        if (v > 0)
            period_ = std::fmin(p_.max_period_s, std::fmax(p_.period_s, 0.95f * p_.max_stride / (p_.duty * v)));
    }
    cmd_ = clamp(raw_);
    odom_ = cmd_;
    cmd_time_ = now_s;
}

float TripodGait::leg_phase(int leg) const { return frac(phase_ + offset(leg)); }

bool TripodGait::params_valid(const GaitParams& p) {
    // Duty above 0.5 overlaps the tripods' stances (both down at each change); at or below
    // 0.5 there is an instant with three feet or fewer. The quadruped needs more than 0.75: its
    // sway across waits for the four-feet gap, 0.5 - 2 (1 - duty) of the cycle. The rest bound
    // what the legs reach: a raised body (height) leaves less leg for the stride (65 mm less the
    // height) and room for a higher swing (40 mm plus the height, the peak under the belly), every
    // corner of these walked clean in sim/tools/high_walk_sim.cpp (top speed each way, then 120 s of
    // commands changing every 0.2-1 s: a stance foot carries on from where the last one left it).
    const bool duty_ok = p.quad ? p.duty >= 0.76f && p.duty <= 0.9f : p.duty >= 0.5f && p.duty <= 0.8f;
    const bool quad_ok = !p.quad || (p.mid_lift >= 0.01f && p.mid_lift <= 0.045f && p.mid_reach >= 0.04f &&
                                     p.mid_reach <= 0.0701f && p.margin >= 0.0f && p.margin <= 0.03f &&
                                     p.sway_speed >= 0.02f && p.sway_speed <= 0.2f && p.mid_speed >= 0.01f &&
                                     p.mid_speed <= 0.1f && std::fabs(p.com_x) <= 0.03f &&
                                     std::fabs(p.com_y) <= 0.03f);
    return duty_ok && quad_ok && p.period_s >= 0.5f && p.period_s <= 3.0f && p.lift >= 0.01f &&
           p.height >= 0.0f && p.height <= 0.04f && p.lift <= std::fmin(0.08f, 0.04f + p.height + 1e-6f) &&
           p.max_stride >= 0.01f && p.max_stride <= std::fmin(0.06f, 0.065f - p.height + 1e-6f) && p.cmd_timeout_s > 0.0f &&
           p.max_period_s >= p.period_s && p.max_period_s <= 4.0f && p.min_foot_speed >= 0.0f &&
           p.min_foot_speed <= 0.01f;
}

bool TripodGait::set_params(const GaitParams& p) {
    if (!standing_ || !params_valid(p)) return false;
    p_ = p;
    period_ = p.period_s;
    phase_ = start_phase();
    for (int i = 0; i < kLegs; ++i) was_stance_[i] = in_stance(i);
    return true;
}

bool TripodGait::in_stance(int leg) const { return walks(leg) && leg_phase(leg) < p_.duty; }

bool TripodGait::settled() const {
    if (std::fabs(sway_x_) > kSettleEps || std::fabs(sway_y_) > kSettleEps || !mids_home()) return false;
    for (int i = 0; i < kLegs; ++i) {
        if (!walks(i)) continue;
        const Vec3 n = neutral(i);
        if (std::fabs(foot_[i].x - n.x) > kSettleEps || std::fabs(foot_[i].y - n.y) > kSettleEps ||
            std::fabs(foot_[i].z - n.z) > kSettleEps)
            return false;
    }
    return true;
}

BodyTwist TripodGait::applied() const {
    if (standing_) return {0, 0, 0};
    return odom_;
}

void TripodGait::step(float dt, float now_s) {
    if (now_s - cmd_time_ > p_.cmd_timeout_s) raw_ = {0, 0, 0};
    const float want_z = g_->stand_z - p_.height;
    if (standing_ && (z_ != want_z || !mids_home())) {
        // A new height while standing: every walking foot (all down, on neutral) straight up or down
        // at kRiseSpeed, the body the other way.
        const float dz = want_z - z_, st = kRiseSpeed * dt;
        z_ = std::fabs(dz) <= st ? want_z : z_ + (dz > 0 ? st : -st);
        for (int i = 0; i < kLegs; ++i)
            if (walks(i)) foot_[i].z = z_;
        // A new mode while standing: the middle feet go up (quad) or down (tripod), in a
        // straight line at mid_speed (onto the moving floor); walking waits for them.
        for (int leg : {1, 4}) {
            const Vec3 h = home(leg);
            const float dx = h.x - foot_[leg].x, dy = h.y - foot_[leg].y, dz = h.z - foot_[leg].z;
            const float d = std::sqrt(dx * dx + dy * dy + dz * dz), k = std::fmin(1.0f, p_.mid_speed * dt / d);
            foot_[leg] = d <= kSettleEps ? h : Vec3{foot_[leg].x + dx * k, foot_[leg].y + dy * k, foot_[leg].z + dz * k};
        }
        return;
    }
    // Adaptive cadence: the cycle that keeps the stride at 95 % of max_stride for this command,
    // within period_s .. max_period_s, reached over about a cycle (a sudden change of cycle would
    // jump the swing feet's landing points). Faster than the cycle allows is clamped meanwhile.
    // The cycle is sized for what the feet will do: the command plus the yaw trim.
    BodyTwist eff = raw_;
    if (worst_foot_speed(raw_) > 0) eff.wz += trim_wz_;  // the trim only while there is a command to walk
    const float v = worst_foot_speed(eff);
    if (v > 0) {
        const float want = std::fmin(p_.max_period_s, std::fmax(p_.period_s, 0.95f * p_.max_stride / (p_.duty * v)));
        period_ += (want - period_) * std::fmin(1.0f, dt / period_);
    }
    cmd_ = clamp(eff);
    const float reach = standing_ ? 1.0f : reach_scale(cmd_);
    cmd_ = {cmd_.vx * reach, cmd_.vy * reach, cmd_.wz * reach};
    odom_ = {cmd_.vx, cmd_.vy, clamp(raw_).wz * reach};
    const bool moving = std::fabs(cmd_.vx) > kZeroCmd || std::fabs(cmd_.vy) > kZeroCmd ||
                        std::fabs(cmd_.wz) > kZeroCmd;
    if (!moving && settled()) {
        if (!standing_) {
            for (int i = 0; i < kLegs; ++i) foot_[i] = home(i);  // within 1 mm: onto it
            sway_x_ = sway_y_ = 0;
        }
        standing_ = true;
        idle_s_ = 0;
        return;
    }
    const float next = frac(phase_ + dt / period_);
    if (!moving && p_.quad) {
        // The quadruped stops in a four-feet gap, every corner foot down on neutral: the body
        // sways back to the centre there before it stands (stepping on would need the sway).
        bool down = true;
        for (int leg : kCorners) {
            const Vec3 n = neutral(leg);
            down = down && was_stance_[leg] && std::fabs(foot_[leg].x - n.x) <= kSettleEps &&
                   std::fabs(foot_[leg].y - n.y) <= kSettleEps;
        }
        idle_s_ += dt;
        if (down || idle_s_ > 2 * period_) {
            if (!down) {  // the backstop, as below
                ++forced_stands_;
                for (int i = 0; i < kLegs; ++i) foot_[i] = landing_[i] = swing_from_[i] = home(i);
                for (int i = 0; i < kLegs; ++i) was_stance_[i] = walks(i);
                sway_x_ = sway_y_ = 0;
            }
            for (int leg : kCorners) foot_[leg].z = z_;
            const float d = std::sqrt(sway_x_ * sway_x_ + sway_y_ * sway_y_), st = p_.sway_speed * dt;
            if (d <= st) {
                sway_x_ = sway_y_ = 0;
                for (int i = 0; i < kLegs; ++i) foot_[i] = home(i);
                standing_ = true;
                idle_s_ = 0;
            } else {
                sway_x_ -= sway_x_ * st / d;
                sway_y_ -= sway_y_ * st / d;
                standing_ = false;
            }
            return;
        }
    } else if (!moving) {
        // At a duty of 0.5 one tripod lifts on the very tick the other lands, so the check above
        // never sees all six feet down at once and the gait would step in place forever (it did,
        // on the robot, at 0.8 s / 0.5). Look at the tick ahead instead: the feet landing on it
        // and the feet about to lift (still down) all on neutral is a stand.
        bool down = true;
        for (int i = 0; i < kLegs && down; ++i) {
            const bool stance_next = frac(next + kOffset[i]) < p_.duty;
            if (!stance_next && !was_stance_[i]) down = false;  // mid-swing
            const Vec3& p = stance_next && !was_stance_[i] ? landing_[i] : foot_[i];
            const Vec3 n = neutral(i);
            down = down && std::fabs(p.x - n.x) <= kSettleEps && std::fabs(p.y - n.y) <= kSettleEps &&
                   (was_stance_[i] || stance_next);
        }
        idle_s_ += dt;
        // The backstop: still stepping two cycles after the command went to zero (no case left
        // known): put every foot down on neutral and stand. forced_stands() counts it.
        if (down || idle_s_ > 2 * period_) {
            if (!down) ++forced_stands_;
            for (int i = 0; i < kLegs; ++i) {
                foot_[i] = neutral(i);
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
    // The quadruped starts walking at the start of a four-feet gap: the sway before the first lift.
    phase_ = standing_ && p_.quad ? frac(start_phase() + dt / period_) : next;
    standing_ = false;

    const float t_stance = p_.duty * period_;
    for (int i = 0; i < kLegs; ++i) {
        if (!walks(i)) continue;  // a quad's middle foot stays up
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
            foot_[i].z = z_;
        } else {
            if (was_stance_[i]) swing_from_[i] = foot_[i];  // lift-off
            const float s = (leg_phase(i) - p_.duty) / (1.0f - p_.duty);  // 0..1
            // Land ahead of neutral by half a stance of travel, so the stance is centred.
            const Vec3 n = neutral(i);
            float vx, vy;
            stance_vel(cmd_, n, &vx, &vy);
            const float lx = n.x - vx * 0.5f * t_stance;
            const float ly = n.y - vy * 0.5f * t_stance;
            landing_[i] = {lx, ly, z_};
            const float e = 0.5f - 0.5f * std::cos(static_cast<float>(M_PI) * s);  // ease in/out
            foot_[i].x = swing_from_[i].x + (lx - swing_from_[i].x) * e;
            foot_[i].y = swing_from_[i].y + (ly - swing_from_[i].y) * e;
            foot_[i].z = z_ + p_.lift * std::sin(static_cast<float>(M_PI) * s);
        }
        was_stance_[i] = stance;
    }
    sway(dt);
}

}  // namespace hexa
