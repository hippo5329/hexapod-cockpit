#include <unity.h>

#include <cmath>

#include "hexa_kin.h"
#include "hexa_servo.h"

using namespace hexa;

void setUp() {}
void tearDown() {}

static void test_neutral_reachable() {
    const Geometry& g = minihexa_geometry();
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        TEST_ASSERT_TRUE(leg_ik(g, leg, neutral_foot(g, leg), &a));
    }
}

// FK(IK(p)) == p over a box around every leg's neutral foot.
static void test_ik_fk_round_trip() {
    const Geometry& g = minihexa_geometry();
    int checked = 0;
    for (int leg = 0; leg < kLegs; ++leg) {
        const Vec3 n = neutral_foot(g, leg);
        for (float dx = -0.04f; dx <= 0.0401f; dx += 0.01f)
            for (float dy = -0.04f; dy <= 0.0401f; dy += 0.01f)
                for (float dz = 0.0f; dz <= 0.0301f; dz += 0.01f) {
                    const Vec3 p{n.x + dx, n.y + dy, n.z + dz};
                    LegAngles a;
                    TEST_ASSERT_TRUE_MESSAGE(leg_ik(g, leg, p, &a), "stride box out of reach");
                    const Vec3 q = leg_fk(g, leg, a);
                    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p.x, q.x);
                    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p.y, q.y);
                    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p.z, q.z);
                    ++checked;
                }
    }
    TEST_ASSERT_EQUAL(6 * 9 * 9 * 4, checked);
}

static void test_unreachable_refused() {
    const Geometry& g = minihexa_geometry();
    LegAngles a{1, 2, 3};
    const Vec3 far{g.hip[0].x + 1.0f, g.hip[0].y, g.stand_z};
    TEST_ASSERT_FALSE(leg_ik(g, 0, far, &a));
    TEST_ASSERT_EQUAL_FLOAT(1, a.coxa);  // untouched
}

// Neutral stance: knee above the foot, tibia folded down, coxa straight.
static void test_neutral_pose_shape() {
    const Geometry& g = minihexa_geometry();
    LegAngles a;
    TEST_ASSERT_TRUE(leg_ik(g, 0, neutral_foot(g, 0), &a));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, a.coxa);
    TEST_ASSERT_TRUE(a.femur > 0.0f);
    TEST_ASSERT_TRUE(a.tibia < 0.0f && a.tibia > -3.1416f);
}

static void test_servo_mapping() {
    ServoCal c = default_cal(2);  // a tibia: centred at -90 deg
    TEST_ASSERT_EQUAL(1500, angle_to_us(c, -1.57079633f));
    c.trim_us = 12;
    TEST_ASSERT_EQUAL(1512, angle_to_us(c, -1.57079633f));
    c.dir = -1;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.2f, us_to_angle(c, angle_to_us(c, -1.2f)));
    bool clamped = false;
    TEST_ASSERT_EQUAL(c.max_us, angle_to_us(c, -10.0f, &clamped));
    TEST_ASSERT_TRUE(clamped);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_neutral_reachable);
    RUN_TEST(test_ik_fk_round_trip);
    RUN_TEST(test_unreachable_refused);
    RUN_TEST(test_neutral_pose_shape);
    RUN_TEST(test_servo_mapping);
    return UNITY_END();
}
