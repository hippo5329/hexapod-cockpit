// Host simulator for the miniHexa: the firmware's own gait, IK and servo mapping
// (firmware/lib) driving simulated servos, on a flat floor.
//
// The world (servos, foot contact, true pose from the grounded feet, slip) is
// firmware/lib/hexa_world, the model the firmware and the ROS 2 node (ros/hexapod_sim)
// run too. Gait odometry integrates what the gait commanded, as the firmware's
// /odom/gait does, so the run reports how far that is from the truth.
//
//   hexa_sim [--slip 0.05] [--csv out.csv] [--quiet] [--quad] [--com x_mm,y_mm] [--set key=value]...
//
// --set overrides a gait parameter (period, duty, stride, lift, height, margin, sway, mid_lift, mid_reach,
// max_period, com_x, com_y (what the gait believes; --com is the truth); metres and seconds), after
// --quad's defaults.
// --quad: the quadruped crawl (quad_gait_params(): middle legs up). --high, --climb: the high walk and the
// climb pose (high_gait_params(), climb_gait_params(): the body raised; height in --set). --com: the centre of mass
// that far off the body's origin (body frame), for the static stability check.
//
// Exit 0 when every scenario passed: no unreachable foot, no clamped pulse, the robot
// stands within two cycles of the stop, the true travel is within 25 % of the
// command (servo lag and slip make it fall short; more than that is a gait fault), and the
// centre of mass stays at least 5 mm inside the feet on the ground (static stability; the
// gait slows to a crawl, so a statically unstable moment is a fall).
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
    bool quiet = false, quad = false;
    const char* preset = "tripod";
    float com_x = 0, com_y = 0;
    const char* sets[16];
    int nsets = 0;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--slip") && i + 1 < argc) slip = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) csv_path = argv[++i];
        else if (!std::strcmp(argv[i], "--quiet")) quiet = true;
        else if (!std::strcmp(argv[i], "--quad")) quad = true;
        else if (!std::strcmp(argv[i], "--high") || !std::strcmp(argv[i], "--climb")) preset = argv[i] + 2;
        else if (!std::strcmp(argv[i], "--set") && i + 1 < argc && nsets < 16) sets[nsets++] = argv[++i];
        else if (!std::strcmp(argv[i], "--com") && i + 1 < argc && std::sscanf(argv[i + 1], "%f,%f", &com_x, &com_y) == 2) ++i;
        else { std::fprintf(stderr, "usage: hexa_sim [--slip f] [--csv file] [--quiet] [--quad|--high|--climb] [--set key=value] [--com x_mm,y_mm]\n"); return 2; }
    }
    FILE* csv = csv_path ? std::fopen(csv_path, "w") : nullptr;
    if (csv) std::fprintf(csv, "t,scenario,cmd_vx,cmd_vy,cmd_wz,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw,margin,sway_x,sway_y,feet_down\n");

    SimRobot robot;
    GaitParams gp0 = quad ? quad_gait_params()
                   : !std::strcmp(preset, "high") ? high_gait_params()
                   : !std::strcmp(preset, "climb") ? climb_gait_params() : GaitParams();
    for (int i = 0; i < nsets; ++i) {
        char key[16];
        float v;
        if (std::sscanf(sets[i], "%15[a-z_]=%f", key, &v) != 2) { std::fprintf(stderr, "--set %s: key=value\n", sets[i]); return 2; }
        float* f = !std::strcmp(key, "period") ? &gp0.period_s : !std::strcmp(key, "duty") ? &gp0.duty
                 : !std::strcmp(key, "stride") ? &gp0.max_stride : !std::strcmp(key, "lift") ? &gp0.lift
                 : !std::strcmp(key, "margin") ? &gp0.margin : !std::strcmp(key, "sway") ? &gp0.sway_speed
                 : !std::strcmp(key, "mid_lift") ? &gp0.mid_lift : !std::strcmp(key, "mid_reach") ? &gp0.mid_reach
                 : !std::strcmp(key, "max_period") ? &gp0.max_period_s : !std::strcmp(key, "com_x") ? &gp0.com_x
                 : !std::strcmp(key, "com_y") ? &gp0.com_y : !std::strcmp(key, "height") ? &gp0.height : nullptr;
        if (!f) { std::fprintf(stderr, "--set %s: no such parameter\n", key); return 2; }
        *f = v;
    }
    if (!TripodGait::params_valid(gp0)) { std::fprintf(stderr, "gait parameters out of range\n"); return 2; }
    if (!robot.init(slip, gp0)) { std::fprintf(stderr, "neutral stance unreachable\n"); return 1; }
    robot.set_com(com_x / 1000, com_y / 1000);
    constexpr float kMinMargin = 0.005f;
    const GaitParams& gp = robot.gait_params();

    const Scenario plan[] = {
        {"forward", {0.05f, 0, 0}, 6},
        {"strafe_left", {0, 0.05f, 0}, 6},
        {"turn_ccw", {0, 0, 0.4f}, 6},
        {"arc", {0.04f, 0, -0.3f}, 6},
        {"too_fast", {0.5f, 0, 0}, 4},  // clamped by the stride limit
    };
    const float dt = 0.02f;  // the firmware's 50 Hz servo frame
    const float settle_limit = 2.0f * gp.max_period_s;  // adaptive cadence: a slow walk stops on a slow cycle
    float t = 0;
    int failures = 0;

    for (const Scenario& sc : plan) {
        robot.reset_poses();
        Pose2 expect;  // the clamped command integrated over the driving time
        int unreachable = 0, clamped = 0;
        float min_margin = 1;
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
            min_margin = std::fmin(min_margin, robot.margin());
            if (driving) {
                target = robot.gait().target();
                expect.apply_body(target.vx * dt, target.vy * dt, target.wz * dt);
            }
            const Pose2& truth = robot.truth();
            const Pose2& odom = robot.odom();

            if (!driving && stood_after < 0 && robot.gait().standing()) stood_after = st - sc.seconds;
            if (csv)
            {
                int down = 0;
                for (int leg = 0; leg < kLegs; ++leg) down += robot.foot_down(leg);
                std::fprintf(csv, "%.2f,%s,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n", t, sc.name,
                             driving ? target.vx : 0, driving ? target.vy : 0, driving ? target.wz : 0,
                             truth.x, truth.y, truth.yaw, odom.x, odom.y, odom.yaw, robot.margin(),
                             robot.gait().sway_x(), robot.gait().sway_y(), down);
            }
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
                        ratio > 0.75 && ratio < 1.05 && min_margin >= kMinMargin;
        if (!ok) ++failures;
        if (!quiet || !ok)
            std::printf("%-12s %s  cmd(%.3f %.3f %.2f) true(%.3f m, %.3f m, %5.1f deg) odom(%.3f, %.3f, %5.1f deg)"
                        "  travel %.0f%%  odom-true %.3f m  stood %.2f s  unreach %d  clamp %d  margin %.1f mm\n",
                        sc.name, ok ? "PASS" : "FAIL", target.vx, target.vy, target.wz, truth.x, truth.y,
                        truth.yaw * 180 / M_PI, odom.x, odom.y, odom.yaw * 180 / M_PI, ratio * 100, odom_err,
                        stood_after, unreachable, clamped, min_margin * 1000);
    }
    const bool raised = gp0.height > 0;
    if (quad || raised) {
        // The switch on the robot: standing on the tripod's six, to the quadruped (middle feet up)
        // or a raised body (the feet down, the body up), a walk, and back to six on the standard
        // stand; stable all along, and walking again on six.
        SimRobot r;
        r.init(slip);
        r.set_com(com_x / 1000, com_y / 1000);
        float tt = 0, mm = 1, up_s = -1, down_s = -1;
        int un = 0, cl = 0;
        auto run = [&](float secs, const BodyTwist& c) {
            for (float st = 0; st < secs; st += dt, tt += dt) {
                r.command(c, tt);
                const StepCounts n = r.step(dt, tt);
                un += n.unreachable;
                cl += n.clamped;
                mm = std::fmin(mm, r.margin());
            }
        };
        const bool took_quad = r.set_gait_params(gp0);
        const float t_up = tt;
        const Geometry& g = minihexa_geometry();
        auto at_mode = [&](const GaitParams& p) {
            return r.gait().mids_home() && r.gait().floor_z() == g.stand_z - p.height;
        };
        for (int i = 0; i < 500 && !at_mode(gp0); ++i) run(dt, {0, 0, 0});
        up_s = tt - t_up;
        run(0.2f, {0, 0, 0});  // the servos catch up
        int down_n = 0;
        for (int leg = 0; leg < kLegs; ++leg) down_n += r.foot_down(leg);
        const bool mids_up = quad ? !r.foot_down(1) && !r.foot_down(4) : down_n == kLegs && std::hypot(r.truth().x, r.truth().y) < 1e-4;  // no step while rising
        run(6, {0.02f, 0, 0});
        run(2 * gp0.max_period_s + 1, {0, 0, 0});
        const bool took_tripod = r.gait().standing() && r.set_gait_params(GaitParams());
        const float t_down = tt;
        for (int i = 0; i < 500 && !at_mode(GaitParams()); ++i) run(dt, {0, 0, 0});
        down_s = tt - t_down;
        run(0.2f, {0, 0, 0});
        bool six = true;
        for (int leg = 0; leg < kLegs; ++leg) six = six && r.foot_down(leg);
        r.reset_poses();
        run(4, {0.04f, 0, 0});
        const bool walked = r.truth().x > 0.1;
        const bool ok = took_quad && mids_up && took_tripod && six && walked && un == 0 && cl == 0 && mm >= kMinMargin;
        if (!ok) ++failures;
        const char* to = quad ? "quad" : preset;
        std::printf("mode_switch  %s  tripod->%s %s (%s in %.2f s%s), walk, %s->tripod %s (down in %.2f s%s), "
                    "tripod walk %.3f m  unreach %d  clamp %d  margin %.1f mm\n",
                    ok ? "PASS" : "FAIL", to, took_quad ? "taken" : "REFUSED", quad ? "middle feet up" : "body up", up_s,
                    mids_up ? "" : ", NOT UP", to, took_tripod ? "taken" : "REFUSED", down_s, six ? "" : ", NOT DOWN",
                    r.truth().x, un, cl, mm * 1000);
    }
    if (csv) std::fclose(csv);
    std::printf("%s: %d of %zu scenarios failed (%s, slip %.2f, com %+.0f,%+.0f mm)\n", failures ? "FAIL" : "PASS",
                failures, sizeof(plan) / sizeof(plan[0]) + (quad || raised), quad ? "quadruped" : preset, slip, com_x, com_y);
    return failures ? 1 : 0;
}
