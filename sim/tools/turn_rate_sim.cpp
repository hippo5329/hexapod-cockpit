// The tripod gait (firmware/lib/hexa_gait) at the firmware's 50 Hz tick, a pure turn command at 10 Hz for 20 s:
// what turn rate the gait's odometry gives (the stride clamp) and the cycle it settles on, for a range of asks.
//   g++ -std=c++17 -O2 -Ifirmware/lib/hexa_kin -Ifirmware/lib/hexa_gait sim/tools/turn_rate_sim.cpp \
//       firmware/lib/hexa_gait/hexa_gait.cpp firmware/lib/hexa_kin/hexa_kin.cpp -o /tmp/turn_rate_sim
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "hexa_gait.h"
#include "hexa_kin.h"

using namespace hexa;

int main() {
    const Geometry& g = minihexa_geometry();
    float rmax = 0;
    for (int i = 0; i < kLegs; ++i) {
        const Vec3 f = neutral_foot(g, i);
        rmax = std::fmax(rmax, std::sqrt(f.x * f.x + f.y * f.y));
    }
    const GaitParams p;
    std::printf("neutral foot radius (max) %.3f m; top foot speed %.3f m/s -> top turn ~%.2f rad/s\n", rmax,
                p.max_stride / (p.duty * p.period_s), p.max_stride / (p.duty * p.period_s) / rmax);
    for (float wz : {0.2f, 0.3f, 0.4f, 0.5f, 0.7f, 1.0f}) {
        TripodGait gait;
        gait.init(&g, p);
        const float dt = 0.02f, S = 20.0f;
        float t = 0, yaw = 0, next = 0;
        while (t < S) {
            if (t >= next) { gait.command({0, 0, wz}, t); next += 0.1f; }
            gait.step(dt, t);
            yaw += gait.applied().wz * dt;
            t += dt;
        }
        std::printf("ask %.2f rad/s: gait %.3f rad/s (%.0f %%), cycle %.2f s\n", wz, yaw / S, 100 * yaw / S / wz,
                    gait.period());
    }
}
