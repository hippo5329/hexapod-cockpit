// Body pose: the body shifts and turns over planted feet within PoseLimits, every corner reachable
// standing and walking, smoothly, eased out for lying down, and it is not locomotion (odometry and
// the simulated ground truth stay put).
#include <unity.h>

#include <cmath>
#include <cstdio>

#include "hexa_world.h"

using namespace hexa;

static const float kDt = 0.02f;

void setUp() {}
void tearDown() {}

static BodyPose corner(int bits) {
    const PoseLimits l;
    auto s = [&](int b, float v) { return (bits >> b & 1) ? v : -v; };
    return {s(0, l.xy), s(1, l.xy), s(2, l.z), s(3, l.angle), s(4, l.angle), s(5, l.angle)};
}

static void test_pose_transform_round_trips() {
    const BodyPose p{0.01f, -0.02f, 0.015f, 0.1f, -0.2f, 0.25f};
    const Vec3 f{0.12f, 0.07f, -0.06f};
    const Vec3 b = to_posed_body(p, f);
    const Vec3 back = from_posed_body(p, b);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, f.x, back.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, f.y, back.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, f.z, back.z);
    // Raising the body 15 mm puts the feet 15 mm lower in its frame; a positive pitch (nose down)
    // brings a front foot up in it.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, f.z - 0.015f, to_posed_body({0, 0, 0.015f, 0, 0, 0}, f).z);
    TEST_ASSERT_TRUE(to_posed_body({0, 0, 0, 0, 0.2f, 0}, f).z > f.z);
}

// Every corner of the limits (64), standing: reached without an unreachable foot or a clamped
// pulse, the feet staying on their spots, and the body not walking anywhere.
static void test_every_corner_standing() {
    for (int c = 0; c < 64; ++c) {
        SimRobot r;
        TEST_ASSERT_TRUE(r.init());
        r.set_pose(corner(c));
        float t = 0;
        for (int i = 0; i < 100; ++i, t += kDt) {  // 2 s: the slowest axis needs 1.05 s
            const StepCounts n = r.step(kDt, t);
            char msg[48];
            snprintf(msg, sizeof msg, "corner %d tick %d", c, i);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, n.unreachable, msg);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, n.clamped, msg);
        }
        TEST_ASSERT_TRUE(r.pose_settled());
        const Geometry& g = minihexa_geometry();
        for (int leg = 0; leg < kLegs; ++leg) {
            const Vec3 b = leg_fk(g, leg, {r.commanded(leg * 3), r.commanded(leg * 3 + 1), r.commanded(leg * 3 + 2)});
            const Vec3 f = from_posed_body(r.pose(), b);
            const Vec3 n = neutral_foot(g, leg);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, n.x, f.x);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, n.y, f.y);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, n.z, f.z);
        }
        TEST_ASSERT_EQUAL_DOUBLE(0, r.odom().x);
        TEST_ASSERT_FLOAT_WITHIN(0.004f, 0, r.truth().x);  // the servos' lag, no more
        TEST_ASSERT_FLOAT_WITHIN(0.004f, 0, r.truth().y);
    }
}

// Walking at top speed with the pose at every corner: every foot reachable.
static void test_every_corner_walking() {
    int worst = 0, worst_c = -1;
    for (int c = 0; c < 64; ++c) {
        SimRobot r;
        TEST_ASSERT_TRUE(r.init());
        r.set_pose(corner(c));
        float t = 0;
        int unreachable = 0;
        for (int i = 0; i < 300; ++i, t += kDt) {
            r.command({0.2f, 0.05f, 0.2f}, t);
            unreachable += r.step(kDt, t).unreachable;
        }
        if (unreachable > worst) { worst = unreachable; worst_c = c; }
    }
    char msg[48];
    snprintf(msg, sizeof msg, "worst corner %d", worst_c);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, worst, msg);
}

// Lying down eases the pose out (the rest pose is a neutral body's) and standing up brings it back;
// a pose change moves the body at the limits' rates, never in one tick.
static void test_rest_eases_the_pose_and_rates_hold() {
    SimRobot r;
    TEST_ASSERT_TRUE(r.init());
    r.set_pose({0, 0, 0.02f, 0.2f, 0, 0});
    float t = 0;
    r.step(kDt, t);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, PoseLimits().turn * kDt, r.pose().roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, PoseLimits().speed * kDt, r.pose().z);
    for (int i = 0; i < 100; ++i, t += kDt) r.step(kDt, t);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, r.pose().roll);
    r.rest(true);
    for (int i = 0; i < 175; ++i, t += kDt) TEST_ASSERT_EQUAL(0, r.step(kDt, t).unreachable);
    TEST_ASSERT_TRUE(r.resting());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, r.pose().roll);
    TEST_ASSERT_FALSE(r.pose_settled());
    r.rest(false);
    for (int i = 0; i < 175; ++i, t += kDt) r.step(kDt, t);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, r.pose().roll);
    TEST_ASSERT_TRUE(r.pose_settled());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pose_transform_round_trips);
    RUN_TEST(test_every_corner_standing);
    RUN_TEST(test_every_corner_walking);
    RUN_TEST(test_rest_eases_the_pose_and_rates_hold);
    return UNITY_END();
}
