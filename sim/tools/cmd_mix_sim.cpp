// The tripod gait (firmware/lib/hexa_gait) at the firmware's 50 Hz tick, fed a velocity command at
// 10 Hz, alone and interleaved with a second publisher's zero twists at 20 Hz (a teleop node left
// running beside Nav2). Prints the gait odometry's speed and the cycle it settles on.
//   g++ -std=c++17 -O2 -Ifirmware/lib/hexa_kin -Ifirmware/lib/hexa_gait sim/tools/cmd_mix_sim.cpp \
//       firmware/lib/hexa_gait/hexa_gait.cpp firmware/lib/hexa_kin/hexa_kin.cpp -o /tmp/cmd_mix_sim
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "hexa_gait.h"
#include "hexa_kin.h"

using namespace hexa;

struct Run { float dist, period; };

static Run run(float vx, float vy, float secs, float zeros_hz, float zero_phase_s) {
    TripodGait g;
    g.init(&minihexa_geometry(), GaitParams());
    const float dt = 0.02f;
    float t = 0, x = 0, y = 0, next_cmd = 0, next_zero = zero_phase_s;
    while (t < secs) {
        if (t >= next_cmd) { g.command({vx, vy, 0}, t); next_cmd += 0.1f; }
        if (zeros_hz > 0 && t >= next_zero) { g.command({0, 0, 0}, t); next_zero += 1.0f / zeros_hz; }
        g.step(dt, t);
        const BodyTwist a = g.applied();
        x += a.vx * dt; y += a.vy * dt;
        t += dt;
    }
    return {std::sqrt(x * x + y * y), g.period()};
}

int main() {
    const float S = 20.0f;
    std::printf("command 10 Hz for %.0f s; asked distance = v x %.0f s\n", S, S);
    for (float v : {0.04f, -0.04f}) {
        for (int axis = 0; axis < 2; ++axis) {
            const float vx = axis ? 0 : v, vy = axis ? v : 0;
            const Run clean = run(vx, vy, S, 0, 0);
            std::printf("%s %+.2f m/s  alone: %.3f m (%.1f mm/s), cycle %.2f s", axis ? "vy" : "vx", v, clean.dist,
                        clean.dist / S * 1000, clean.period);
            for (float ph : {0.01f, 0.03f, 0.05f}) {
                const Run mix = run(vx, vy, S, 19.6f, ph);
                std::printf(" | +zeros 19.6 Hz (offset %2.0f ms): %.3f m (%.1f mm/s), cycle %.2f s", ph * 1000, mix.dist,
                            mix.dist / S * 1000, mix.period);
            }
            std::printf("\n");
        }
    }
}
