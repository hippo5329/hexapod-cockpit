#include <unity.h>

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "hexa_gait.h"
#include "hexa_kin.h"
#include "hexa_servo.h"

using namespace hexa;

static const float kDt = 0.02f;
static TripodGait gait;
static GaitParams params;

void setUp() {
    params = GaitParams();
    gait.init(&minihexa_geometry(), params);
}
void tearDown() {}

// Walk with a fresh command every tick for `seconds`, starting at *t.
static void drive(const BodyTwist& cmd, float seconds, float* t) {
    for (float s = 0; s < seconds; s += kDt, *t += kDt) {
        gait.command(cmd, *t);
        gait.step(kDt, *t);
    }
}

static int feet_down() {
    int n = 0;
    for (int i = 0; i < kLegs; ++i)
        if (gait.foot(i).z <= minihexa_geometry().stand_z + 1e-6f) ++n;
    return n;
}

static void test_starts_standing() {
    TEST_ASSERT_TRUE(gait.standing());
    const BodyTwist a = gait.applied();
    TEST_ASSERT_EQUAL_FLOAT(0, a.vx);
    float t = 0;
    gait.step(kDt, t);
    TEST_ASSERT_TRUE(gait.standing());
}

// At the stride limit, in eight directions and both turns, every foot target stays
// reachable, every pulse stays in range, and a tripod is always on the ground.
static void test_feet_reachable_at_max_speed() {
    const Geometry& g = minihexa_geometry();
    const BodyTwist cmds[] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},  {0, -1, 0}, {1, 1, 0},
                              {1, -1, 0}, {0, 0, 3},  {0, 0, -3}, {1, 0, 2},  {-1, 1, -2}};
    for (const BodyTwist& c : cmds) {
        setUp();
        float t = 0;
        for (float s = 0; s < 5.0f; s += kDt, t += kDt) {
            gait.command(c, t);
            gait.step(kDt, t);
            TEST_ASSERT_TRUE(feet_down() >= 3);
            for (int leg = 0; leg < kLegs; ++leg) {
                LegAngles a;
                TEST_ASSERT_TRUE_MESSAGE(leg_ik(g, leg, gait.foot(leg), &a), "foot out of reach");
                const float ang[3] = {a.coxa, a.femur, a.tibia};
                for (int j = 0; j < 3; ++j) {
                    bool clamped = false;
                    angle_to_us(default_cal(leg * 3 + j), ang[j], &clamped);
                    TEST_ASSERT_FALSE_MESSAGE(clamped, "pulse clamped");
                }
            }
        }
    }
}

static void test_stride_clamped() {
    float t = 0;
    gait.command({5.0f, 0, 0}, t);
    const BodyTwist c = gait.target();
    const float t_stance = params.duty * params.period_s;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, params.max_stride, c.vx * t_stance);
    // A slow command passes unchanged.
    gait.command({0.01f, 0, 0}, t);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, gait.target().vx);
}

// Released: finishes its steps and stands on the neutral stance within two cycles.
static void test_stop_settles_to_neutral() {
    float t = 0;
    drive({0.05f, 0.02f, 0.3f}, 3.3f, &t);  // stop mid-stride
    TEST_ASSERT_FALSE(gait.standing());
    gait.command({0, 0, 0}, t);
    float waited = 0;
    while (!gait.standing() && waited < 2 * params.period_s) {
        gait.step(kDt, t);
        t += kDt;
        waited += kDt;
    }
    TEST_ASSERT_TRUE(gait.standing());
    for (int leg = 0; leg < kLegs; ++leg) {
        const Vec3 n = neutral_foot(minihexa_geometry(), leg);
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, n.x, gait.foot(leg).x);
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, n.y, gait.foot(leg).y);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, n.z, gait.foot(leg).z);
    }
}

// Every valid duty and period stands after a stop, within two cycles, on its own (the backstop
// never fires): at 0.5 one tripod lands on the tick the other lifts, and it once stepped forever.
static void test_stop_settles_for_every_duty_and_period() {
    for (float duty : {0.5f, 0.55f, 0.6f, 0.7f, 0.8f}) {
        for (float period : {0.5f, 0.8f, 1.0f, 1.5f, 2.0f, 3.0f}) {
            GaitParams p = params;
            p.period_s = period;
            p.duty = duty;
            TripodGait g;
            g.init(&minihexa_geometry(), p);
            float t = 0;
            for (float s = 0; s < 2.3f * period; s += kDt, t += kDt) {
                g.command({0.04f, 0.01f, 0.2f}, t);
                g.step(kDt, t);
            }
            g.command({0, 0, 0}, t);
            float waited = 0;
            const float cycle = g.period();  // adaptive cadence: the cycle in use when it stopped
            while (!g.standing() && waited < 2 * cycle + kDt) {
                g.step(kDt, t);
                t += kDt;
                waited += kDt;
            }
            char msg[64];
            snprintf(msg, sizeof msg, "duty %.2f period %.1f", duty, period);
            TEST_ASSERT_TRUE_MESSAGE(g.standing(), msg);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, g.forced_stands(), msg);
            for (int leg = 0; leg < kLegs; ++leg) {
                const Vec3 n = neutral_foot(minihexa_geometry(), leg);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3f, n.x, g.foot(leg).x, msg);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3f, n.y, g.foot(leg).y, msg);
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6f, n.z, g.foot(leg).z, msg);
            }
        }
    }
}

// Lift-offs of leg 0 while walking `cmd` for `seconds` from a stand.
static int steps_walking(const BodyTwist& cmd, float seconds) {
    gait.init(&minihexa_geometry(), params);
    float t = 0;
    int lifts = 0;
    bool was_down = true;
    for (float s = 0; s < seconds; s += kDt, t += kDt) {
        gait.command(cmd, t);
        gait.step(kDt, t);
        const bool down = gait.foot(0).z <= minihexa_geometry().stand_z + 1e-6f;
        if (was_down && !down) ++lifts;
        was_down = down;
    }
    return lifts;
}

// Adaptive cadence: a slow walk slows the cycle (up to max_period_s) to keep its strides long, so
// it takes fewer steps for the same distance; at top speed the cycle is period_s, as before.
static void test_slow_walk_takes_fewer_longer_steps() {
    // 0.02 m/s: 0.057 m / (0.7 x 0.02) = 4.1 s wanted, 3.0 s the limit: 20 s is ~7 steps, not ~13.
    const int slow = steps_walking({0.02f, 0, 0}, 20);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, params.max_period_s, gait.period());
    TEST_ASSERT_TRUE(slow >= 6 && slow <= 8);
    // 0.04 m/s: 2.04 s; the stride stays at 95 % of the longest.
    steps_walking({0.04f, 0, 0}, 10);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.95f * params.max_stride / (params.duty * 0.04f), gait.period());
    // Top speed and beyond: the fastest cycle, the command clamped as before.
    steps_walking({0.2f, 0, 0}, 6);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, params.period_s, gait.period());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, params.max_stride / (params.duty * params.period_s), gait.target().vx);
}

// A command that moves no foot 2 mm/s does not start the gait: no stepping on the spot.
static void test_tiny_command_does_not_step() {
    float t = 0;
    for (int i = 0; i < 200; ++i, t += kDt) {
        gait.command({0.001f, 0.0005f, 0.003f}, t);
        gait.step(kDt, t);
        TEST_ASSERT_TRUE(gait.standing());
    }
}

// A plant that turns 60 % of the gait's yaw rate and drifts -0.012 rad/s (slip, as on tile): walk
// `cmd` for `seconds` with or without YawHold; returns the body's heading change.
static float walk_with_slip(const BodyTwist& cmd, float seconds, bool hold, float* odom_wz_mean) {
    gait.init(&minihexa_geometry(), params);
    YawHold yh;
    float t = 0, yaw = 0, gz = 0, odom = 0;
    int n = 0;
    for (float s = 0; s < seconds; s += kDt, t += kDt) {
        gait.command(cmd, t);
        gait.set_yaw_trim(yh.update(gait.commanded().wz, gz, hold && !gait.standing(), kDt));
        gait.step(kDt, t);
        gz = gait.standing() ? 0.0f : 0.6f * gait.target().wz - 0.012f;
        yaw += gz * kDt;
        odom += gait.applied().wz;
        ++n;
    }
    *odom_wz_mean = odom / n;
    return yaw;
}

// YawHold keeps a straight walk's heading against slip drift, makes a turn reach its commanded
// rate, and leaves the gait's odometry (applied()) on the commanded rate either way.
static void test_yaw_hold_holds_heading_and_turn_rate() {
    float odom;
    const float drift = walk_with_slip({0.05f, 0, 0}, 20, false, &odom);
    TEST_ASSERT_TRUE(drift < -0.2f);  // without: -0.24 rad, as the robot drifted
    const float held = walk_with_slip({0.05f, 0, 0}, 20, true, &odom);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.0f, held);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, odom);  // odometry: no turn, as commanded
    const float open = walk_with_slip({0, 0, 0.3f}, 10, false, &odom);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 0.6f * 3.0f - 0.12f, open);  // ~55 % of the turn
    const float closed = walk_with_slip({0, 0, 0.3f}, 10, true, &odom);
    TEST_ASSERT_FLOAT_WITHIN(0.12f, 3.0f, closed);  // the commanded 3 rad, less the first second's lag
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f, odom);
}

// A lost command stream (no zero ever sent) also ends in a stand.
static void test_command_timeout_stands() {
    float t = 0;
    gait.command({0.05f, 0, 0}, t);
    float waited = 0;
    while (waited < params.cmd_timeout_s + 2 * params.period_s + kDt) {
        gait.step(kDt, t);
        t += kDt;
        waited += kDt;
        if (waited < params.cmd_timeout_s - kDt) TEST_ASSERT_FALSE(gait.standing());
    }
    TEST_ASSERT_TRUE(gait.standing());
    TEST_ASSERT_EQUAL_FLOAT(0, gait.target().vx);
}

// New parameters only while standing, only valid ones; walking keeps the old ones.
static void test_params_only_while_standing() {
    TripodGait g;
    g.init(&minihexa_geometry(), GaitParams());
    GaitParams p;
    p.period_s = 1.2f;
    p.duty = 0.6f;
    TEST_ASSERT_TRUE(g.set_params(p));
    TEST_ASSERT_EQUAL_FLOAT(1.2f, g.params().period_s);
    GaitParams bad = p;
    bad.duty = 0.4f;
    TEST_ASSERT_FALSE(g.set_params(bad));
    bad = p;
    bad.lift = 0.08f;
    TEST_ASSERT_FALSE(g.set_params(bad));
    g.command({0.03f, 0, 0}, 0.0f);
    for (int i = 0; i < 10; ++i) g.step(0.02f, 0.02f * i);
    TEST_ASSERT_FALSE(g.standing());
    GaitParams q = p;
    q.period_s = 2.0f;
    TEST_ASSERT_FALSE(g.set_params(q));
    TEST_ASSERT_EQUAL_FLOAT(1.2f, g.params().period_s);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_starts_standing);
    RUN_TEST(test_feet_reachable_at_max_speed);
    RUN_TEST(test_stride_clamped);
    RUN_TEST(test_stop_settles_to_neutral);
    RUN_TEST(test_stop_settles_for_every_duty_and_period);
    RUN_TEST(test_slow_walk_takes_fewer_longer_steps);
    RUN_TEST(test_tiny_command_does_not_step);
    RUN_TEST(test_yaw_hold_holds_heading_and_turn_rate);
    RUN_TEST(test_command_timeout_stands);
    RUN_TEST(test_params_only_while_standing);
    return UNITY_END();
}
