// Host simulator for the miniHexa: the firmware's own gait, IK and servo mapping
// (firmware/lib) driving simulated servos, on a flat floor.
//
// Each tick the gait gives foot targets, IK turns them into joint angles, the
// calibration into pulses, and each SimServo turns toward its pulse at a bounded
// rate. Forward kinematics of the servos' ACTUAL angles gives where the feet really
// are; the feet on the ground do not move in the world, so the body's true motion is
// the rigid transform that best explains how those feet moved in the body frame
// (2-D least squares). A slip fraction loses part of that motion, as a real foot on a
// smooth floor does. Gait odometry integrates what the gait commanded, as the
// firmware's /odom/gait will, so the run reports how far that is from the truth.
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

#include "hexa_gait.h"
#include "hexa_kin.h"
#include "hexa_servo.h"

using namespace hexa;

namespace {

struct Pose {
    double x = 0, y = 0, yaw = 0;
    void apply_body(double dx, double dy, double dyaw) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        x += c * dx - s * dy;
        y += s * dx + c * dy;
        yaw = std::remainder(yaw + dyaw, 2 * M_PI);
    }
};

struct Scenario {
    const char* name;
    BodyTwist cmd;
    float seconds;
};

// Body motion that maps foot positions `now` onto `before` (feet fixed in the world).
bool rigid_2d(const Vec3* before, const Vec3* now, const bool* use, double* dx, double* dy,
              double* dyaw) {
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

    const Geometry& g = minihexa_geometry();
    GaitParams gp;
    TripodGait gait;
    gait.init(&g, gp);
    SimServoParams sp;
    ServoCal cal[kJoints];
    SimServo servo[kJoints];
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        if (!leg_ik(g, leg, gait.foot(leg), &a)) { std::fprintf(stderr, "neutral stance unreachable\n"); return 1; }
        const float ang[3] = {a.coxa, a.femur, a.tibia};
        for (int j = 0; j < 3; ++j) {
            cal[leg * 3 + j] = default_cal(leg * 3 + j);
            servo[leg * 3 + j].reset(cal[leg * 3 + j], ang[j]);
        }
    }

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

    Vec3 feet_prev[kLegs];
    bool contact_prev[kLegs];
    auto actual_feet = [&](Vec3* out, bool* contact) {
        for (int leg = 0; leg < kLegs; ++leg) {
            LegAngles a{servo[leg * 3].angle(), servo[leg * 3 + 1].angle(), servo[leg * 3 + 2].angle()};
            out[leg] = leg_fk(g, leg, a);
            contact[leg] = out[leg].z <= g.stand_z + 0.003f;
        }
    };
    actual_feet(feet_prev, contact_prev);

    for (const Scenario& sc : plan) {
        Pose truth, odom, expect;  // expect: the clamped command integrated over the driving time
        int unreachable = 0, clamped = 0;
        float stood_after = -1;
        const float total = sc.seconds + settle_limit + 1.0f;
        BodyTwist target{0, 0, 0};
        for (float st = 0; st < total; st += dt, t += dt) {
            const bool driving = st < sc.seconds;
            // A 50 Hz command stream, then one explicit zero (a teleop released), as the
            // command timeout is a link-loss guard and has its own unit test.
            if (driving) gait.command(sc.cmd, t);
            else if (st < sc.seconds + dt) gait.command({0, 0, 0}, t);
            gait.step(dt, t);
            if (driving) {
                target = gait.target();
                expect.apply_body(target.vx * dt, target.vy * dt, target.wz * dt);
            }
            const BodyTwist ap = gait.applied();
            odom.apply_body(ap.vx * dt, ap.vy * dt, ap.wz * dt);

            for (int leg = 0; leg < kLegs; ++leg) {
                LegAngles a;
                if (!leg_ik(g, leg, gait.foot(leg), &a)) { ++unreachable; continue; }
                const float ang[3] = {a.coxa, a.femur, a.tibia};
                for (int j = 0; j < 3; ++j) {
                    bool cl = false;
                    const int us = angle_to_us(cal[leg * 3 + j], ang[j], &cl);
                    if (cl) ++clamped;
                    servo[leg * 3 + j].command_us(us);
                }
            }
            for (SimServo& s : servo) s.step(dt, sp);

            Vec3 feet[kLegs];
            bool contact[kLegs], both[kLegs];
            actual_feet(feet, contact);
            for (int i = 0; i < kLegs; ++i) both[i] = contact[i] && contact_prev[i];
            double dx, dy, dyaw;
            if (rigid_2d(feet_prev, feet, both, &dx, &dy, &dyaw))
                truth.apply_body(dx * (1 - slip), dy * (1 - slip), dyaw * (1 - slip));
            for (int i = 0; i < kLegs; ++i) { feet_prev[i] = feet[i]; contact_prev[i] = contact[i]; }

            if (!driving && stood_after < 0 && gait.standing()) stood_after = st - sc.seconds;
            if (csv)
                std::fprintf(csv, "%.2f,%s,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", t, sc.name,
                             driving ? target.vx : 0, driving ? target.vy : 0, driving ? target.wz : 0,
                             truth.x, truth.y, truth.yaw, odom.x, odom.y, odom.yaw);
        }
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
