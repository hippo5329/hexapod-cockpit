// Simulated IMU and range: the noise they declare is the noise they make, gravity leaks
// through the mount tilt, the gyro follows the body, the range sees the room.
#include <unity.h>

#include <cmath>
#include <cstdio>

#include "hexa_sensors.h"
#include "hexa_world.h"

using namespace hexa;

static const float kDt = 0.02f;

void setUp() {}
void tearDown() {}

// The white noise's variance from first differences (a slow bias walk barely shows).
static double diff_var(const float* x, int n) {
    double s = 0;
    for (int i = 1; i < n; ++i) s += (x[i] - x[i - 1]) * (x[i] - x[i - 1]);
    return s / (n - 1) / 2;
}

static void test_still_imu_noise_tilt_and_bias() {
    SimImu imu;
    imu.init();
    const ImuParams& p = imu.params();
    const int n = 3000;  // 60 s at 50 Hz
    static float ax[n], az[n], gz[n];
    double max, may, maz, mgz;
    max = may = maz = mgz = 0;
    for (int i = 0; i < n; ++i) {
        const ImuSample s = imu.step(kDt, {0, 0, 0});
        ax[i] = s.ax; az[i] = s.az; gz[i] = s.gz;
        max += s.ax / n; may += s.ay / n; maz += s.az / n; mgz += s.gz / n;
    }
    const double g = p.gravity * (1 + p.scale_error);
    const double tol = p.accel_bias_limit + 0.005;
    // Mount tilt: -1.5 deg pitch puts +g sin(1.5 deg) on x, 1 deg roll +g sin(1 deg) on y.
    TEST_ASSERT_DOUBLE_WITHIN(tol, -std::sin(p.mount_pitch) * g, max);
    TEST_ASSERT_DOUBLE_WITHIN(tol, std::cos(p.mount_pitch) * std::sin(p.mount_roll) * g, may);
    TEST_ASSERT_DOUBLE_WITHIN(tol, std::cos(p.mount_pitch) * std::cos(p.mount_roll) * g, maz);
    TEST_ASSERT_TRUE(max > 0.2);  // the leak is there, not lost in the bias
    TEST_ASSERT_TRUE(std::fabs(mgz) <= p.gyro_bias_limit + 0.001);
    // Declared variance = actual, within 10 %.
    const double va = diff_var(ax, n), vaz = diff_var(az, n), vg = diff_var(gz, n);
    char msg[160];
    std::snprintf(msg, sizeof msg, "accel var x %.3g z %.3g (declared %.3g), gyro var %.3g (declared %.3g)", va, vaz,
                  uniform_var(p.accel_noise), vg, uniform_var(p.gyro_noise));
    TEST_MESSAGE(msg);
    TEST_ASSERT_DOUBLE_WITHIN(0.1 * uniform_var(p.accel_noise), uniform_var(p.accel_noise), va);
    TEST_ASSERT_DOUBLE_WITHIN(0.1 * uniform_var(p.accel_noise), uniform_var(p.accel_noise), vaz);
    TEST_ASSERT_DOUBLE_WITHIN(0.1 * uniform_var(p.gyro_noise), uniform_var(p.gyro_noise), vg);
    // Declared figures are the QMI8658's table entries.
    TEST_ASSERT_DOUBLE_WITHIN(1e-7, 8e-5, uniform_var(p.accel_noise));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2e-6, uniform_var(p.gyro_noise));
}

// Drive the model and its IMU together; returns the gyro's integrated yaw.
static double drive(SimRobot& r, SimImu& imu, const BodyTwist& cmd, float seconds, float* t, double* peak_ax_dev) {
    double yaw = 0;
    const double leak = -std::sin(imu.params().mount_pitch) * imu.params().gravity;
    for (float s = 0; s < seconds; s += kDt, *t += kDt) {
        r.command(cmd, *t);
        r.step(kDt, *t);
        const ImuSample m = imu.step(kDt, r.truth_twist());
        yaw += m.gz * kDt;
        if (peak_ax_dev) *peak_ax_dev = std::fmax(*peak_ax_dev, std::fabs(m.ax - leak));
    }
    return yaw;
}

static void test_gyro_follows_the_true_turn() {
    SimRobot r;
    TEST_ASSERT_TRUE(r.init(0.05));
    SimImu imu;
    imu.init();
    float t = 0;
    const double yaw0 = r.truth().yaw;
    double gyro = drive(r, imu, {0, 0, 0.4f}, 6, &t, nullptr);
    gyro += drive(r, imu, {0, 0, 0}, 3, &t, nullptr);
    const double truth = r.truth().yaw - yaw0;  // ~2.3 rad: no wrap
    char msg[120];
    std::snprintf(msg, sizeof msg, "gyro %.3f rad, truth %.3f rad, gait odom %.3f rad", gyro, truth, r.odom().yaw);
    TEST_MESSAGE(msg);
    // Scale (1 %), tilt (cos) and a bias inside its envelope over 9 s.
    TEST_ASSERT_DOUBLE_WITHIN(0.015 * truth + imu.params().gyro_bias_limit * 9 + 0.01, truth, gyro);
}

static void test_walk_accel_is_bounded() {
    SimRobot r;
    TEST_ASSERT_TRUE(r.init(0.05));
    SimImu imu;
    imu.init();
    float t = 0;
    double peak = 0;
    drive(r, imu, {0.05f, 0, 0}, 6, &t, &peak);
    char msg[100];
    std::snprintf(msg, sizeof msg, "walking 0.05 m/s: peak |ax - tilt leak| %.3f m/s^2", peak);
    TEST_MESSAGE(msg);
    // The tripod hand-over jerks the body; the band limit keeps it to a fraction of g.
    TEST_ASSERT_TRUE(peak > 0.02);  // stepping shows in the accelerometer
    TEST_ASSERT_TRUE(peak < 3.0);
}

static void test_range_sees_the_room() {
    Wall room[kRoomWalls];
    default_room(room);
    SimRange sonar;
    sonar.init();
    const RangeParams& p = sonar.params();
    // Facing the +x wall from the centre: 1.0 - 0.08.
    Pose2 at;
    for (int i = 0; i < 50; ++i) {
        const float m = sonar.measure(at, room, kRoomWalls);
        TEST_ASSERT_FLOAT_WITHIN(p.noise + p.noise_frac * 0.92f + 1e-4f, 0.92f, m);
    }
    // 20 deg off: the cone's nearest ray (12.5 deg) answers.
    at.yaw = 0.349066;
    const float expect = (1.0f - 0.08f * std::cos(0.349066f)) / std::cos(0.349066f - p.fov / 2);
    TEST_ASSERT_FLOAT_WITHIN(p.noise + p.noise_frac * expect + 1e-3f, expect, sonar.measure(at, room, kRoomWalls));
    // 50 deg off: every ray meets a wall more than 30 deg from its normal -> no echo.
    at.yaw = 0.872665;
    TEST_ASSERT_TRUE(std::isinf(sonar.measure(at, room, kRoomWalls)) && sonar.measure(at, room, kRoomWalls) > 0);
    // Too close: -inf. Out of range: +inf.
    at = Pose2();
    at.x = 0.91;
    TEST_ASSERT_TRUE(std::isinf(sonar.measure(at, room, kRoomWalls)) && sonar.measure(at, room, kRoomWalls) < 0);
    Wall big[kRoomWalls];
    default_room(big, 10.0f);
    TEST_ASSERT_TRUE(std::isinf(sonar.measure(Pose2(), big, kRoomWalls)));
}

static void test_range_follows_the_walk() {
    SimRobot r;
    TEST_ASSERT_TRUE(r.init(0.05));
    Wall room[kRoomWalls];
    default_room(room);
    SimRange sonar;
    sonar.init();
    float t = 0;
    for (float s = 0; s < 6; s += kDt, t += kDt) {
        r.command({0.05f, 0, 0}, t);
        r.step(kDt, t);
    }
    const float expect = 1.0f - 0.08f - static_cast<float>(r.truth().x);
    TEST_ASSERT_FLOAT_WITHIN(0.003f + 0.005f * expect + 2e-3f, expect, sonar.measure(r.truth(), room, kRoomWalls));
}

// Trims (by servo id) go into every pulse; the simulated servos read pulses through the
// same calibration, so the model stays where it was.
static void test_trims_reach_the_pulses_not_the_model() {
    SimRobot r;
    TEST_ASSERT_TRUE(r.init());
    const int before = r.pulse_us(0);  // rf coxa = servo id 7
    const float angle = r.joint_angle(0);
    int16_t t[kJoints] = {};
    t[6] = 30;
    r.set_trims(t);
    TEST_ASSERT_EQUAL(before + 30, r.pulse_us(0));
    for (int i = 0; i < 50; ++i) r.step(0.02f, i * 0.02f);
    TEST_ASSERT_EQUAL(before + 30, r.pulse_us(0));
    TEST_ASSERT_FLOAT_WITHIN(0.003f, angle, r.joint_angle(0));
}

// A board mounted like the simulated one: the tilt read off a still minute, then every sample
// turned back into the body frame: gravity on +z, a turn's rate on z.
static void test_imu_mount_levels_the_board() {
    SimImu imu;
    imu.init();
    const ImuParams& p = imu.params();
    ImuSample mean{};
    const int n = 3000;
    for (int i = 0; i < n; ++i) {
        const ImuSample s = imu.step(kDt, BodyTwist{0, 0, 0});
        mean.ax += s.ax / n;
        mean.ay += s.ay / n;
        mean.az += s.az / n;
    }
    float roll, pitch;
    ImuMount::level_from(mean, &roll, &pitch);
    // The accelerometer's bias (0.03 m/s^2 at most) is about 0.2 deg of tilt.
    TEST_ASSERT_FLOAT_WITHIN(0.004f, p.mount_roll, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.004f, p.mount_pitch, pitch);
    ImuMount mount;
    mount.set(roll, pitch);
    ImuSample s = mean;
    mount.apply(&s);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, s.ax);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, s.ay);
    TEST_ASSERT_FLOAT_WITHIN(0.15f, p.gravity, s.az);
    // A pure yaw rate in the body reads on all three sensor axes; the mount puts it back on z.
    float gx = 0, gy = 0, gz = 0;
    for (int i = 0; i < 500; ++i) {
        ImuSample t = imu.step(kDt, BodyTwist{0, 0, 1.0f});
        mount.apply(&t);
        gx += t.gx / 500;
        gy += t.gy / 500;
        gz += t.gz / 500;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gx);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, gy);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 1.0f, gz);
    // Unset, a mount changes nothing.
    ImuMount none;
    ImuSample u = mean;
    none.apply(&u);
    TEST_ASSERT_EQUAL_FLOAT(mean.ax, u.ax);
    TEST_ASSERT_EQUAL_FLOAT(mean.az, u.az);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_still_imu_noise_tilt_and_bias);
    RUN_TEST(test_gyro_follows_the_true_turn);
    RUN_TEST(test_walk_accel_is_bounded);
    RUN_TEST(test_range_sees_the_room);
    RUN_TEST(test_range_follows_the_walk);
    RUN_TEST(test_trims_reach_the_pulses_not_the_model);
    RUN_TEST(test_imu_mount_levels_the_board);
    return UNITY_END();
}
