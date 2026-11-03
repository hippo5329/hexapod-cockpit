#include <unity.h>

#include <cmath>

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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_starts_standing);
    RUN_TEST(test_feet_reachable_at_max_speed);
    RUN_TEST(test_stride_clamped);
    RUN_TEST(test_stop_settles_to_neutral);
    RUN_TEST(test_command_timeout_stands);
    return UNITY_END();
}
