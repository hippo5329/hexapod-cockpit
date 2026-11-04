// The board's AHRS on the simulated IMU: seeded level at once, roll and pitch held to the
// sensor's true tilt (the mount's) within what the message declares, yaw following the
// true turn by the gyro alone, a tilt step corrected at the gain's pace, and a stalled
// loop or a dead accelerometer never turned into motion or NaN.
#include <unity.h>

#include <cmath>
#include <cstdio>

#include "hexa_ahrs.h"
#include "hexa_sensors.h"
#include "hexa_world.h"

using namespace hexa;

static const float kDt = 0.02f;
static const double kDeg = 57.29578;

void setUp() {}
void tearDown() {}

static double wrap(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

struct Run {
    double tilt_rms = 0, tilt_max = 0, yaw_err = 0, yaw_truth = 0, yaw_ahrs = 0;
};

// The model, its IMU and the AHRS together for `seconds` at `cmd`; tilt error against the
// mount (the body is level in the model), after the first 2 s.
static Run drive(const BodyTwist& cmd, float seconds) {
    SimRobot r;
    TEST_ASSERT_TRUE(r.init(0.05));
    SimImu imu;
    imu.init();
    Ahrs a;
    a.init();
    const double mr = imu.params().mount_roll, mp = imu.params().mount_pitch;
    Run out;
    double se = 0;
    int n = 0;
    float t = 0;
    for (int i = 0; t < seconds; ++i, t += kDt) {
        r.command(cmd, t);
        r.step(kDt, t);
        const ImuSample s = imu.step(kDt, r.truth_twist());
        a.update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, kDt);
        if (t >= 2.0f) {
            const double er = a.roll() - mr, ep = a.pitch() - mp;
            se += er * er + ep * ep;
            n += 2;
            out.tilt_max = std::fmax(out.tilt_max, std::fmax(std::fabs(er), std::fabs(ep)));
        }
    }
    out.tilt_rms = std::sqrt(se / n);
    out.yaw_truth = r.truth().yaw;
    out.yaw_ahrs = a.yaw();
    out.yaw_err = std::fabs(wrap(a.yaw() - r.truth().yaw));
    return out;
}

static void report(const char* what, const Run& r) {
    char msg[200];
    std::snprintf(msg, sizeof msg, "%s: tilt rms %.3f deg, max %.3f deg; yaw %.3f rad, truth %.3f rad", what,
                  r.tilt_rms * kDeg, r.tilt_max * kDeg, r.yaw_ahrs, r.yaw_truth);
    TEST_MESSAGE(msg);
}

static void test_first_sample_seeds_the_tilt() {
    SimImu imu;
    imu.init();
    Ahrs a;
    a.init();
    TEST_ASSERT_FALSE(a.seeded());
    const ImuSample s = imu.step(kDt, {0, 0, 0});
    a.update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, kDt);
    TEST_ASSERT_TRUE(a.seeded());
    // One sample's noise: 0.0155 m/s^2 of 9.8 is under 0.1 deg per axis.
    TEST_ASSERT_DOUBLE_WITHIN(0.2 / kDeg, imu.params().mount_roll, a.roll());
    TEST_ASSERT_DOUBLE_WITHIN(0.2 / kDeg, imu.params().mount_pitch, a.pitch());
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, a.yaw());
    const Quat& q = a.q();
    TEST_ASSERT_DOUBLE_WITHIN(1e-5, 1.0, q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// The declared roll/pitch deviation must cover the measured error, and not by so much that
// it says nothing (a quarter of the declared variance at least).
static void check_tilt(const Run& r) {
    const double declared_sd = std::sqrt(AhrsParams().roll_pitch_var);
    TEST_ASSERT_TRUE(r.tilt_rms <= declared_sd);
    TEST_ASSERT_TRUE(r.tilt_rms >= declared_sd / 2);
    TEST_ASSERT_TRUE(r.tilt_max < 1.0 / kDeg);
}

static void test_standing_holds_the_tilt() {
    const Run r = drive({0, 0, 0}, 30);
    report("standing 30 s", r);
    check_tilt(r);
    // No magnetometer: yaw is the gyro bias's integral, inside its envelope.
    TEST_ASSERT_TRUE(r.yaw_err <= SimImu().params().gyro_bias_limit * 30);
}

static void test_turning_yaw_follows_the_truth() {
    const Run r = drive({0, 0, 0.4f}, 6);
    report("turning 0.4 rad/s 6 s", r);
    check_tilt(r);
    // The gyro alone: 1 % scale and a bias inside its envelope.
    TEST_ASSERT_TRUE(std::fabs(r.yaw_truth) > 2.0);
    TEST_ASSERT_TRUE(r.yaw_err <= 0.015 * std::fabs(r.yaw_truth) + SimImu().params().gyro_bias_limit * 6 + 0.01);
}

static void test_walking_jolts_do_not_tip_it() {
    const Run r = drive({0.05f, 0, 0}, 30);
    report("walking 0.05 m/s 30 s", r);
    check_tilt(r);
}

static void test_a_tilt_step_settles_at_the_gains_pace() {
    Ahrs a;
    a.init();
    a.update(0, 0, 0, 0, 0, 9.8f, kDt);  // seeded level
    const float th = 10 / kDeg;
    float t = 0;
    while (t < 5 && std::fabs(a.roll() - th) > 0.5 / kDeg) {
        a.update(0, 0, 0, 0, 9.8f * std::sin(th), 9.8f * std::cos(th), kDt);
        t += kDt;
    }
    char msg[80];
    std::snprintf(msg, sizeof msg, "10 deg roll step: within 0.5 deg after %.2f s", t);
    TEST_MESSAGE(msg);
    // Bounded by beta: at most ~2 beta rad/s of correction, so not instantly; but not slow.
    TEST_ASSERT_TRUE(t > 0.5f);
    TEST_ASSERT_TRUE(t < 1.5f);
    TEST_ASSERT_DOUBLE_WITHIN(0.05 / kDeg, 0.0, a.pitch());
}

static void test_stalls_and_bad_samples_are_not_motion() {
    Ahrs a;
    a.init();
    a.update(0, 0, 0, 0, 0, 0, kDt);  // no gravity: cannot seed
    TEST_ASSERT_FALSE(a.seeded());
    a.update(0, 0, 0, 0, 0, 9.8f, kDt);
    TEST_ASSERT_TRUE(a.seeded());
    // A 2 s gap at 1 rad/s would be 2 rad of yaw: held instead.
    a.update(0, 0, 1.0f, 0, 0, 9.8f, 2.0f);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, 0.0, a.yaw());
    // A dead accelerometer (zeros) or a NaN: the gyro alone, or nothing; never NaN.
    for (int i = 0; i < 50; ++i) a.update(0, 0, 0.5f, 0, 0, 0, kDt);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.5, a.yaw());
    a.update(NAN, 0, 0, 0, 0, 9.8f, kDt);
    a.update(0, 0, 0, NAN, 0, 9.8f, kDt);
    const Quat& q = a.q();
    TEST_ASSERT_TRUE(std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.5, a.yaw());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_first_sample_seeds_the_tilt);
    RUN_TEST(test_standing_holds_the_tilt);
    RUN_TEST(test_turning_yaw_follows_the_truth);
    RUN_TEST(test_walking_jolts_do_not_tip_it);
    RUN_TEST(test_a_tilt_step_settles_at_the_gains_pace);
    RUN_TEST(test_stalls_and_bad_samples_are_not_motion);
    return UNITY_END();
}
