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

// FK(IK(p)) == p over the gait's working space around every leg's neutral foot: a
// disc of the stride limit's radius (40 mm, any direction), from the ground up to 30 mm
// of lift. (A 40 mm square is NOT reachable: its corners are 57 mm out.)
static void test_ik_fk_round_trip() {
    const Geometry& g = minihexa_geometry();
    int checked = 0;
    for (int leg = 0; leg < kLegs; ++leg) {
        const Vec3 n = neutral_foot(g, leg);
        for (int ri = 0; ri <= 4; ++ri)
            for (int ai = 0; ai < 16; ++ai)
                for (int zi = 0; zi <= 3; ++zi) {
                    const float r = 0.01f * ri, a = ai * 3.14159265f / 8;
                    const Vec3 p{n.x + r * std::cos(a), n.y + r * std::sin(a), n.z + 0.01f * zi};
                    LegAngles q;
                    TEST_ASSERT_TRUE_MESSAGE(leg_ik(g, leg, p, &q), "working space out of reach");
                    const Vec3 f = leg_fk(g, leg, q);
                    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p.x, f.x);
                    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p.y, f.y);
                    TEST_ASSERT_FLOAT_WITHIN(1e-5f, p.z, f.z);
                    ++checked;
                }
    }
    TEST_ASSERT_EQUAL(6 * 5 * 16 * 4, checked);
}

// The edge of the envelope: a 40 mm square's corner that lies along the leg, 57 mm
// outward at ground level, is past full reach (femur + tibia, 125 mm).
static void test_square_corner_out_of_reach() {
    const Geometry& g = minihexa_geometry();
    const Vec3 n = neutral_foot(g, 0);
    const float c = std::cos(g.hip[0].mount_yaw), s = std::sin(g.hip[0].mount_yaw);
    const float out = 0.04f * 1.41421356f;
    const Vec3 p{n.x + out * c, n.y + out * s, n.z};
    LegAngles q;
    TEST_ASSERT_FALSE(leg_ik(g, 0, p, &q));
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

static void test_servo_bus_frame() {
    // One servo, id 1 at 1500 us over 500 ms, worked by hand from the frame layout.
    const hexa::BusServo one{1, 1500};
    uint8_t f[64];
    const uint8_t want[] = {0x55, 0x55, 0x01, 0x0A, 0x01, 0x01, 0xF4, 0x01, 0x01, 0xDC, 0x05, 0x8E};
    TEST_ASSERT_EQUAL(sizeof want, hexa::servo_bus_frame(&one, 1, 500, f, sizeof f));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, f, sizeof want);
    // 18 servos: 4 + 58 + 1 bytes, length byte 61; a short buffer, a bad pulse or count -> 0.
    hexa::BusServo all[18];
    for (int i = 0; i < 18; ++i) all[i] = {static_cast<uint8_t>(i + 1), 1500};
    TEST_ASSERT_EQUAL(63, hexa::servo_bus_frame(all, 18, 100, f, sizeof f));
    TEST_ASSERT_EQUAL(61, f[3]);
    TEST_ASSERT_EQUAL(0, hexa::servo_bus_frame(all, 18, 100, f, 62));
    all[3].us = 2600;
    TEST_ASSERT_EQUAL(0, hexa::servo_bus_frame(all, 18, 100, f, sizeof f));
    TEST_ASSERT_EQUAL(0, hexa::servo_bus_frame(all, 0, 100, f, sizeof f));
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
    RUN_TEST(test_square_corner_out_of_reach);
    RUN_TEST(test_unreachable_refused);
    RUN_TEST(test_neutral_pose_shape);
    RUN_TEST(test_servo_mapping);
    RUN_TEST(test_servo_bus_frame);
    return UNITY_END();
}
