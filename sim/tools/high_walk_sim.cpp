// How high the miniHexa can stand and step: the reach behind the high walk and the climb pose
// (hexa_gait.cpp high_gait_params, climb_gait_params). The firmware's world model (gait, IK, servo
// calibration) at the 50 Hz tick.
//
// 1. Walk sweep: for each body height (GaitParams::height) and swing lift, rise, then walk at top
//    speed forward, sideways, diagonally, turning and turning while walking; count the foot targets
//    out of reach or clamped to a servo's range, and the smallest margin left to any servo's range.
// 2. Climb reach: at each body height, how high above the floor the front foot reaches, at its
//    neutral spot and up to 30 mm further out, with every servo inside its range by a margin.
//   g++ -std=c++17 -O2 -Ifirmware/lib/hexa_kin -Ifirmware/lib/hexa_gait -Ifirmware/lib/hexa_servo \
//       -Ifirmware/lib/hexa_world -Ifirmware/lib/hexa_sensors sim/tools/high_walk_sim.cpp \
//       firmware/lib/hexa_world/hexa_world.cpp firmware/lib/hexa_sensors/hexa_sensors.cpp \
//       firmware/lib/hexa_kin/hexa_kin.cpp firmware/lib/hexa_gait/hexa_gait.cpp \
//       firmware/lib/hexa_servo/hexa_servo.cpp -o /tmp/high_walk_sim
//   /tmp/high_walk_sim [margin_us]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

#include "hexa_gait.h"
#include "hexa_kin.h"
#include "hexa_servo.h"
#include "hexa_world.h"

using namespace hexa;

namespace {

// The smallest distance (us) of any joint's commanded pulse to its servo's range.
int pulse_margin(const SimRobot& r) {
    int m = 100000;
    for (int k = 0; k < kJoints; ++k) {
        const ServoCal c = default_cal(k);
        const float us = c.centre_us + c.dir * (r.commanded(k) - c.zero_rad) * kUsPerRad;
        m = std::min(m, static_cast<int>(std::fmin(us - c.min_us, c.max_us - us)));
    }
    return m;
}

struct WalkResult {
    int bad = 0;        // ticks with a foot out of reach or a pulse clamped
    int margin = 1e5;   // us
    float rise_s = 0;   // standing up to the height
};

WalkResult walk(const GaitParams& p) {
    WalkResult out;
    const BodyTwist asks[] = {{0.2f, 0, 0},     {-0.2f, 0, 0},    {0, 0.2f, 0},     {0, -0.2f, 0},
                              {0.14f, 0.14f, 0}, {0, 0, 1.5f},    {0.1f, 0, 0.8f},  {0.07f, 0.07f, -0.8f}};
    for (const BodyTwist& ask : asks) {
        SimRobot r;
        GaitParams p0;
        r.init(0.05, p0);
        const float dt = 0.02f;
        float t = 0;
        for (int i = 0; i < 10; ++i) r.step(dt, t += dt);
        r.set_gait_params(p);
        // Rise (no command), then walk 4 top-speed cycles of the slowest kind, then stop.
        while (t < 30 && (r.gait().floor_z() != minihexa_geometry().stand_z - p.height)) {
            const StepCounts n = r.step(dt, t += dt);
            out.bad += n.unreachable || n.clamped;
            out.margin = std::min(out.margin, pulse_margin(r));
            out.rise_s = std::fmax(out.rise_s, t);
        }
        const float t0 = t;
        while (t < t0 + 4 * p.max_period_s + 2) {
            r.command(t < t0 + 4 * p.max_period_s ? ask : BodyTwist{0, 0, 0}, t);
            const StepCounts n = r.step(dt, t += dt);
            out.bad += n.unreachable || n.clamped;
            out.margin = std::min(out.margin, pulse_margin(r));
        }
    }
    // Then what Nav2 does: a new command every 0.2-1.0 s, any mix of the three at up to top speed
    // and past it, for 120 s from one stand (a stance foot carries on from where the last command
    // left it).
    {
        SimRobot r;
        r.init(0.05, p);
        unsigned seed = 12345;
        auto rnd = [&seed]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xffff) / 65535.0f; };
        const float dt = 0.02f;
        float t = 0, next = 0;
        BodyTwist ask{0, 0, 0};
        while (t < 120) {
            if (t >= next) {
                ask = {(rnd() * 2 - 1) * 0.15f, (rnd() * 2 - 1) * 0.15f, (rnd() * 2 - 1) * 1.2f};
                if (rnd() < 0.15f) ask = {0, 0, 0};
                next = t + 0.2f + 0.8f * rnd();
            }
            r.command(ask, t);
            const StepCounts n = r.step(dt, t += dt);
            out.bad += n.unreachable || n.clamped;
            out.margin = std::min(out.margin, pulse_margin(r));
        }
    }
    out.rise_s = std::fmax(0.0f, out.rise_s - 0.2f);
    return out;
}

// Highest foot z (body frame, mm steps) of leg `leg` reachable at the horizontal spot (x, y) with
// every joint inside its range by `margin_us`; NAN if none.
float top_reach(const Geometry& g, int leg, float x, float y, float floor_z, int margin_us, float top = 0.10f) {
    float best = NAN;
    for (float z = floor_z; z <= top + 1e-6f; z += 0.001f) {
        LegAngles a;
        if (!leg_ik(g, leg, {x, y, z}, &a)) continue;
        const float ang[3] = {a.coxa, a.femur, a.tibia};
        bool ok = true;
        for (int j = 0; j < 3 && ok; ++j) {
            const ServoCal c = default_cal(leg * 3 + j);
            const float us = c.centre_us + c.dir * (ang[j] - c.zero_rad) * kUsPerRad;
            ok = us - c.min_us >= margin_us && c.max_us - us >= margin_us;
        }
        if (ok) best = z;
    }
    return best;
}

}  // namespace

int main(int argc, char** argv) {
    const int margin_us = argc > 1 ? std::atoi(argv[1]) : 50;
    const Geometry& g = minihexa_geometry();
    std::printf("Walk sweep: tripod, period 0.8 s, duty 0.7, stride 60/45 mm; bad = ticks with a foot "
                "unreachable or clamped (8 asks), margin = least us to any servo's range\n");
    std::printf("height lift | stride 60: bad margin | stride 45: bad margin | belly mm (each run is checked against params_valid only below)\n");
    for (float h : {0.0f, 0.01f, 0.02f, 0.025f, 0.03f, 0.035f, 0.04f}) {
        for (float lift : {0.03f, 0.04f, 0.05f}) {
            GaitParams p;
            p.height = h;
            p.lift = lift;
            const WalkResult a = walk(p);
            p.max_stride = 0.045f;
            const WalkResult b = walk(p);
            // The belly 12-15 mm under the hip axis (hexa_world.h RestParams): the higher figure.
            std::printf("%4.0f mm %3.0f mm | %5d %5d us | %5d %5d us | %4.0f\n", h * 1000, lift * 1000, a.bad, a.margin,
                        b.bad, b.margin, (-(g.stand_z - h) - 0.015f) * 1000);
        }
    }
    std::printf("\nThe longest clean stride (no bad tick, 8 asks) per height and lift, period 1.0 s\n");
    for (float h : {0.015f, 0.02f, 0.025f, 0.03f, 0.035f, 0.04f, 0.045f}) {
        for (float lift : {0.04f, 0.05f, 0.06f}) {
            float best = 0;
            int margin = 0;
            for (float s = 0.02f; s <= 0.0601f; s += 0.005f) {
                GaitParams p;
                p.period_s = 1.0f;
                p.height = h;
                p.lift = lift;
                p.max_stride = s;
                if (!TripodGait::params_valid(p)) continue;
                const WalkResult w = walk(p);
                if (w.bad == 0 && w.margin >= margin_us) {
                    best = s;
                    margin = w.margin;
                }
            }
            std::printf("height %2.0f mm lift %2.0f mm: stride %2.0f mm (margin %3d us), belly %2.0f mm, top %3.0f mm/s\n",
                        h * 1000, lift * 1000, best * 1000, margin, (-(g.stand_z - h) - 0.015f) * 1000,
                        best / (0.7f * 1.0f) * 1000);
        }
    }
    std::printf("\nThe bounds' corners (params_valid: stride <= 65 mm - height, lift <= 40 mm + height, 80 max)\n");
    for (float h = 0; h <= 0.0401f; h += 0.005f) {
        for (float period : {0.8f, 2.0f}) {
            GaitParams p;
            p.period_s = period;
            p.max_period_s = std::fmax(3.0f, period);
            p.height = h;
            p.lift = std::fmin(0.08f, 0.04f + h);
            p.max_stride = std::fmin(0.06f, 0.065f - h);
            const bool valid = TripodGait::params_valid(p);
            const WalkResult w = walk(p);
            std::printf("height %2.0f lift %2.0f stride %2.0f period %.1f: %s bad %d margin %d us, rise %.1f s\n", h * 1000,
                        p.lift * 1000, p.max_stride * 1000, period, valid ? "valid" : "INVALID", w.bad, w.margin, w.rise_s);
        }
    }
    std::printf("\nThe presets\n");
    const struct { const char* name; GaitParams p; } presets[] = {
        {"tripod", GaitParams()}, {"high", high_gait_params()}, {"climb", climb_gait_params()}};
    for (const auto& s : presets) {
        const WalkResult w = walk(s.p);
        std::printf("%-6s %s height %2.0f lift %2.0f stride %2.0f period %.1f duty %.2f: bad %d margin %d us, rise %.1f s, "
                    "top %3.0f mm/s, belly %2.0f mm\n", s.name, TripodGait::params_valid(s.p) ? "valid" : "INVALID",
                    s.p.height * 1000, s.p.lift * 1000, s.p.max_stride * 1000, s.p.period_s, s.p.duty, w.bad, w.margin,
                    w.rise_s, s.p.max_stride / (s.p.duty * s.p.period_s) * 1000, (-(g.stand_z - s.p.height) - 0.015f) * 1000);
    }
    std::printf("\nClimb reach (front feet, %d us margin): the highest foot above the floor at its neutral "
                "spot, +15 and +30 mm out, up to the belly's line (a step any higher meets the body)\n", margin_us);
    for (float h = 0; h <= 0.0401f; h += 0.005f) {
        const float floor = g.stand_z - h;
        std::printf("height %2.0f mm (belly %2.0f mm):", h * 1000, (-floor - 0.015f) * 1000);
        for (float out : {0.0f, 0.015f, 0.03f}) {
            const Vec3 n = neutral_foot(g, 0);
            const Hip& hp = g.hip[0];
            const float k = (g.neutral_reach + out) / g.neutral_reach;
            const float z = top_reach(g, 0, hp.x + (n.x - hp.x) * k, hp.y + (n.y - hp.y) * k, floor, margin_us, -0.015f);
            std::printf("  +%2.0f: %5.1f mm", out * 1000, (z - floor) * 1000);
        }
        // Can it stand there (every foot on neutral, inside the ranges)?
        bool stand = true;
        for (int leg = 0; leg < kLegs && stand; ++leg) {
            const Vec3 n = neutral_foot(g, leg);
            stand = !std::isnan(top_reach(g, leg, n.x, n.y, floor, margin_us, floor));
        }
        std::printf("%s\n", stand ? "" : "  (cannot stand)");
    }
}
