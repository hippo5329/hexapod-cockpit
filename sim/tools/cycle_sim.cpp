// How short the tripod's cycle can be: for each period, walk at top speed forward, sideways,
// diagonally and turning, with the firmware's world model (gait, IK, servo calibration, servos that
// turn at most SimServoParams::max_rate) at the 50 Hz tick, and report the fastest joint the gait
// asks for, how far the servos fall behind it, the feet out of reach or clamped, and the speed the
// body really makes (truth, after slip). The joints' demand grows as 1 / period; once it passes the
// servos' rate, the swing lands short and the stride is lost.
//   g++ -std=c++17 -O2 -Ifirmware/lib/hexa_kin -Ifirmware/lib/hexa_gait -Ifirmware/lib/hexa_servo \
//       -Ifirmware/lib/hexa_world -Ifirmware/lib/hexa_sensors sim/tools/cycle_sim.cpp \
//       firmware/lib/hexa_world/hexa_world.cpp firmware/lib/hexa_sensors/hexa_sensors.cpp \
//       firmware/lib/hexa_kin/hexa_kin.cpp firmware/lib/hexa_gait/hexa_gait.cpp \
//       firmware/lib/hexa_servo/hexa_servo.cpp -o /tmp/cycle_sim
//   /tmp/cycle_sim [max_rate rad/s]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

#include "hexa_gait.h"
#include "hexa_servo.h"
#include "hexa_world.h"

using namespace hexa;

namespace {

struct Run {
    float rate = 0;    // fastest commanded joint, rad/s
    float lag = 0;     // most a servo trailed its command, rad
    int bad = 0;       // ticks with a foot out of reach or a pulse clamped
    float speed = 0;   // the body's true speed (m/s, or rad/s turning) over the walk
};

Run walk(float period, const BodyTwist& ask, const SimServoParams& sp) {
    Run out;
    GaitParams p;
    p.period_s = period;
    p.max_period_s = std::fmax(3.0f, period);
    SimRobot r;
    r.init(0.05, p, sp);
    const float dt = 0.02f;
    float t = 0;
    for (int i = 0; i < 25; ++i) r.step(dt, t += dt);  // stand
    float prev[kJoints];
    for (int k = 0; k < kJoints; ++k) prev[k] = r.commanded(k);
    const int cycles = 8;
    const float t0 = t, t_meas = t0 + 2 * period;  // two cycles to get up to speed
    Pose2 start;
    while (t < t0 + cycles * period) {
        r.command(ask, t);
        const StepCounts n = r.step(dt, t += dt);
        out.bad += n.unreachable || n.clamped;
        for (int k = 0; k < kJoints; ++k) {
            out.rate = std::fmax(out.rate, std::fabs(r.commanded(k) - prev[k]) / dt);
            out.lag = std::fmax(out.lag, std::fabs(r.commanded(k) - r.joint_angle(k)));
            prev[k] = r.commanded(k);
        }
        if (t - dt < t_meas && t >= t_meas) start = r.truth();
    }
    const Pose2& end = r.truth();
    const float span = t - t_meas;
    out.speed = ask.wz != 0 && ask.vx == 0 && ask.vy == 0 ? std::fabs(end.yaw - start.yaw) / span
                                                          : std::hypot(end.x - start.x, end.y - start.y) / span;
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    SimServoParams sp;
    if (argc > 1) sp.max_rate = std::atof(argv[1]);
    std::printf("Servo rate %.1f rad/s; tripod, duty 0.7, lift 30 mm, stride 60 mm; asks past top speed (the gait "
                "clamps); speed = truth after slip over cycles 3-8\n", sp.max_rate);
    std::printf("period | forward: need rad/s, lag deg, bad, mm/s | sideways | diagonal | turn: rad/s\n");
    const BodyTwist asks[] = {{0.5f, 0, 0}, {0, 0.5f, 0}, {0.35f, 0.35f, 0}, {0, 0, 4.0f}};
    for (float period : {1.0f, 0.8f, 0.65f, 0.55f, 0.5f, 0.45f, 0.4f, 0.35f, 0.3f}) {
        std::printf("%.2f s", period);
        for (const BodyTwist& a : asks) {
            const Run w = walk(period, a, sp);
            const bool turn = a.wz != 0;
            std::printf(" | %5.1f %4.1f %3d %s%5.*f", w.rate, w.lag * 57.2958f, w.bad, turn ? "" : "",
                        turn ? 2 : 0, turn ? w.speed : w.speed * 1000);
        }
        std::printf("\n");
    }
}
