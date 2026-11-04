// Host simulator for the miniHexa: the firmware's own gait, IK and servo mapping
// (firmware/lib) driving simulated servos, on a flat floor.
//
// The world (servos, foot contact, true pose from the grounded feet, slip) is
// firmware/lib/hexa_world, the model the firmware and the ROS 2 node (ros/hexapod_sim)
// run too. Gait odometry integrates what the gait commanded, as the firmware's
// /odom/gait does, so the run reports how far that is from the truth.
//
//   hexa_sim [--slip 0.05] [--csv out.csv] [--quiet]
//
// Exit 0 when every scenario passed: no unreachable foot, no clamped pulse, the robot
// stands within two cycles of the stop, and the true travel is within 25 % of the
// command (servo lag and slip make it fall short; more than that is a gait fault).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "hexa_world.h"

using namespace hexa;

namespace {

struct Scenario {
    const char* name;
    BodyTwist cmd;
    float seconds;
};

}  // namespace

int main(int argc, char** argv) {
    double slip = 0.05;
    const char* csv_path = nullptr;
    bool quiet = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--slip") && i + 1 < argc) slip = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) csv_path = argv[++i];
        else if (!std::strcmp(argv[i], "--quiet")) quiet = true;
        else { std::fprintf(stderr, "usage: hexa_sim [--slip f] [--csv file] [--quiet]\n"); return 2; }
    }
    FILE* csv = csv_path ? std::fopen(csv_path, "w") : nullptr;
    if (csv) std::fprintf(csv, "t,scenario,cmd_vx,cmd_vy,cmd_wz,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw\n");

    SimRobot robot;
    if (!robot.init(slip)) { std::fprintf(stderr, "neutral stance unreachable\n"); return 1; }
    const GaitParams& gp = robot.gait_params();

    const Scenario plan[] = {
        {"forward", {0.05f, 0, 0}, 6},
        {"strafe_left", {0, 0.05f, 0}, 6},
        {"turn_ccw", {0, 0, 0.4f}, 6},
        {"arc", {0.04f, 0, -0.3f}, 6},
        {"too_fast", {0.5f, 0, 0}, 4},  // clamped by the stride limit
    };
    const float dt = 0.02f;  // the firmware's 50 Hz servo frame
    const float settle_limit = 2.0f * gp.period_s;
    float t = 0;
    int failures = 0;

    for (const Scenario& sc : plan) {
        robot.reset_poses();
        Pose2 expect;  // the clamped command integrated over the driving time
        int unreachable = 0, clamped = 0;
        float stood_after = -1;
        const float total = sc.seconds + settle_limit + 1.0f;
        BodyTwist target{0, 0, 0};
        for (float st = 0; st < total; st += dt, t += dt) {
            const bool driving = st < sc.seconds;
            // A 50 Hz command stream, then one explicit zero (a teleop released), as the
            // command timeout is a link-loss guard and has its own unit test.
            if (driving) robot.command(sc.cmd, t);
            else if (st < sc.seconds + dt) robot.command({0, 0, 0}, t);
            const StepCounts n = robot.step(dt, t);
            unreachable += n.unreachable;
            clamped += n.clamped;
            if (driving) {
                target = robot.gait().target();
                expect.apply_body(target.vx * dt, target.vy * dt, target.wz * dt);
            }
            const Pose2& truth = robot.truth();
            const Pose2& odom = robot.odom();

            if (!driving && stood_after < 0 && robot.gait().standing()) stood_after = st - sc.seconds;
            if (csv)
                std::fprintf(csv, "%.2f,%s,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, sc.name,
                             driving ? target.vx : 0, driving ? target.vy : 0, driving ? target.wz : 0,
                             truth.x, truth.y, truth.yaw, odom.x, odom.y, odom.yaw);
        }
        const Pose2& truth = robot.truth();
        const Pose2& odom = robot.odom();
        // Travel = how much of the commanded motion the robot really made: distance for a
        // translation, heading for a pure turn (unwrapped by the commanded total).
        const double exp_lin = std::hypot(expect.x, expect.y);
        const double ew = target.wz * sc.seconds;
        const double true_yaw = ew + std::remainder(truth.yaw - ew, 2 * M_PI);
        const double ratio = exp_lin > 1e-3 ? std::hypot(truth.x, truth.y) / exp_lin
                                            : (std::fabs(ew) > 1e-3 ? true_yaw / ew : 1);
        const double odom_err = std::hypot(odom.x - truth.x, odom.y - truth.y);
        const bool ok = unreachable == 0 && clamped == 0 && stood_after >= 0 && stood_after <= settle_limit &&
                        ratio > 0.75 && ratio < 1.05;
        if (!ok) ++failures;
        if (!quiet || !ok)
            std::printf("%-12s %s  cmd(%.3f %.3f %.2f) true(%.3f m, %.3f m, %5.1f deg) odom(%.3f, %.3f, %5.1f deg)"
                        "  travel %.0f%%  odom-true %.3f m  stood %.2f s  unreach %d  clamp %d\n",
                        sc.name, ok ? "PASS" : "FAIL", target.vx, target.vy, target.wz, truth.x, truth.y,
                        truth.yaw * 180 / M_PI, odom.x, odom.y, odom.yaw * 180 / M_PI, ratio * 100, odom_err,
                        stood_after, unreachable, clamped);
    }
    if (csv) std::fclose(csv);
    std::printf("%s: %d of %zu scenarios failed (slip %.2f)\n", failures ? "FAIL" : "PASS", failures,
                sizeof(plan) / sizeof(plan[0]), slip);
    return failures ? 1 : 0;
}
