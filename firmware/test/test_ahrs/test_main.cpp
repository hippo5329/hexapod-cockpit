// The board's AHRS on the simulated IMU: seeded level at once, roll and pitch held to the
// sensor's true tilt (the mount's) within what the message declares, yaw following the
// true turn by the gyro alone, a tilt step corrected at the gain's pace, and a stalled
// loop or a dead accelerometer never turned into motion or NaN.
#include <unity.h>

#include <cmath>
#include <initializer_list>
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

// The bias estimator on a known bias: nothing learned in the settle time, the bias within
// noise after a few seconds of standing, held while walking, and a hand turning the
// "standing" robot (a rate beyond any bias) not learned.
static void test_gyro_bias_is_learned_standing_and_held_walking() {
    GyroBias gb;
    gb.init();
    Rng rng(7);
    const float b[3] = {0.003f, -0.002f, 0.0035f}, noise = 0.0024495f;
    float t = 0;
    for (; t < 0.4f; t += kDt) gb.update(b[0] + rng.uniform(noise), b[1] + rng.uniform(noise), b[2] + rng.uniform(noise), true, kDt);
    TEST_ASSERT_FALSE_MESSAGE(gb.learned(), "learned during the settle time");
    for (; t < 5.0f; t += kDt) gb.update(b[0] + rng.uniform(noise), b[1] + rng.uniform(noise), b[2] + rng.uniform(noise), true, kDt);
    printf("bias after 5 s standing: %+.5f %+.5f %+.5f (true %+.4f %+.4f %+.4f)\n", gb.bias()[0], gb.bias()[1],
           gb.bias()[2], b[0], b[1], b[2]);
    for (int i = 0; i < 3; ++i) TEST_ASSERT_FLOAT_WITHIN(3e-4f, b[i], gb.bias()[i]);
    const float held = gb.bias()[2];
    for (int i = 0; i < 200; ++i) gb.update(0.0f, 0.0f, 0.4f + b[2], false, kDt);  // walking, turning
    TEST_ASSERT_EQUAL_FLOAT(held, gb.bias()[2]);
    for (int i = 0; i < 200; ++i) gb.update(0.0f, 0.0f, 0.3f, true, kDt);  // turned by hand while it stands
    TEST_ASSERT_EQUAL_FLOAT(held, gb.bias()[2]);
    float gx = 0.1f, gy = 0.2f, gz = 0.3f;
    gb.correct(&gx, &gy, &gz);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f - held, gz);
}

// The model's IMU (bias a random walk) with the estimator in front of the AHRS: standing,
// the AHRS's yaw drifts by what is left of the bias, not by the bias.
static void test_standing_yaw_holds_with_the_bias_taken_out() {
    SimRobot r;
    TEST_ASSERT_TRUE(r.init(0.05));
    SimImu imu;
    imu.init();
    GyroBias gb;
    gb.init();
    Ahrs raw, fixed;
    raw.init();
    fixed.init();
    double worst_raw = 0, worst_fixed = 0;
    float t = 0;
    for (; t < 120.0f; t += kDt) {
        r.command({0, 0, 0}, t);
        r.step(kDt, t);
        ImuSample s = imu.step(kDt, r.truth_twist());
        raw.update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, kDt);
        gb.update(s.gx, s.gy, s.gz, r.gait().standing(), kDt);
        gb.correct(&s.gx, &s.gy, &s.gz);
        fixed.update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, kDt);
        worst_raw = std::fmax(worst_raw, std::fabs(raw.yaw()));
        worst_fixed = std::fmax(worst_fixed, std::fabs(fixed.yaw()));
    }
    printf("standing 120 s: yaw drift worst %.4f rad raw, %.4f rad with the bias taken out (now %+.4f / %+.4f)\n",
           worst_raw, worst_fixed, raw.yaw(), fixed.yaw());
    TEST_ASSERT_TRUE_MESSAGE(gb.learned(), "nothing learned in 120 s standing");
    // Measured 0.055 raw, 0.0045 with the bias out (seed 5329).
    TEST_ASSERT_TRUE_MESSAGE(worst_fixed < 0.015, "standing yaw drifts with the bias taken out");
    TEST_ASSERT_TRUE_MESSAGE(worst_fixed < worst_raw / 4, "taking the bias out barely helps");
}

// Gait odometry's declared twist: loose while walking, tighter than the gyro while standing.
static void test_gait_odom_variance_is_tight_only_standing() {
    const OdomVariance walk = gait_odom_variance(false), stand = gait_odom_variance(true);
    const float gyro_var = uniform_var(ImuParams().gyro_noise);
    TEST_ASSERT_TRUE(walk.twist[5] > 100 * gyro_var);   // walking: the gyro leads the heading
    TEST_ASSERT_TRUE(stand.twist[5] < gyro_var / 100);  // standing: the gait's zero does
    TEST_ASSERT_TRUE(stand.twist[0] < 1e-6 && stand.twist[1] < 1e-6);
    for (int i : {2, 3, 4}) TEST_ASSERT_TRUE(walk.twist[i] >= 1e6 && stand.twist[i] >= 1e6);  // not measured
    for (int i = 0; i < 6; ++i) TEST_ASSERT_EQUAL_DOUBLE(walk.pose[i], stand.pose[i]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_first_sample_seeds_the_tilt);
    RUN_TEST(test_standing_holds_the_tilt);
    RUN_TEST(test_turning_yaw_follows_the_truth);
    RUN_TEST(test_walking_jolts_do_not_tip_it);
    RUN_TEST(test_a_tilt_step_settles_at_the_gains_pace);
    RUN_TEST(test_stalls_and_bad_samples_are_not_motion);
    RUN_TEST(test_gyro_bias_is_learned_standing_and_held_walking);
    RUN_TEST(test_standing_yaw_holds_with_the_bias_taken_out);
    RUN_TEST(test_gait_odom_variance_is_tight_only_standing);
    return UNITY_END();
}
