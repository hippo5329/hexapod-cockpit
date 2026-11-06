// miniHexa firmware: one binary, the application chosen by the NVS `app` key.
//
//   base     micro-ROS node: /cmd_vel in; /joint_states and /odom/gait out, and in sim
//            mode the simulated /imu/data (with the board's AHRS orientation), /sonar
//            and /sim/ground_truth. The gait
//            ticks at 50 Hz whether or not the agent is there; a lost command stream
//            ends in a stand (hexa_gait's timeout), never a frozen stride.
//   console  line commands on the serial port: info, link, i2c, servo, bat, sonar, imu, app,
//            sim, gait, pose, reboot, shutdown. On a Wi-Fi link the base app takes them too (Serial is free).
//
// The final shutdown (/shutdown, the console's `shutdown`, or a flat pack: NVS `low_bat_mv`):
// the robot lies down, then the ESP32 goes into deep sleep until BOOT or EN is pressed.
//
// The link (NVS `link`, link.h): micro-ROS over USB serial, or over UDP on Wi-Fi with
// ArduinoOTA beside it. On Wi-Fi the console and OTA work in either app.
//
// Press BOOT within the first two seconds after a reset to get the console whatever
// `app` says. Sim mode (NVS `sim`, default on) drives simulated servos; real mode is
// refused until board_minihexa.h is verified.
#include <Arduino.h>
#include <cmath>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_chip_info.h>
#include <driver/rtc_io.h>
#include <esp_mac.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>

// The entity limits are compiled into libmicroros from esp32.meta, and micro_ros_platformio
// builds that library only when it is missing: a raised limit does not reach a cached build.
// Sim mode creates 6 publishers (joint_states, odom/gait, battery, imu/data, sonar, ground
// truth) and 4 subscriptions; with a stale library the 6th failed and every session was torn
// down at once. Delete .pio/libdeps/<env>/micro_ros_platformio/libmicroros after editing the meta.
static_assert(RMW_UXRCE_MAX_PUBLISHERS >= 6, "libmicroros built with fewer publishers than esp32.meta: rebuild it");
static_assert(RMW_UXRCE_MAX_SUBSCRIPTIONS >= 5, "libmicroros built with fewer subscriptions than esp32.meta: rebuild it");
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/range.h>
#ifdef HEXA_CMD_VEL_STAMPED
#include <geometry_msgs/msg/twist_stamped.h>
#else
#include <geometry_msgs/msg/twist.h>
#endif

#include "board_io.h"
#include "board_minihexa.h"
#include "hexa_ahrs.h"
#include "hexa_power.h"
#include "hexa_sensors.h"
#include "hexa_world.h"
#include "link.h"
#include "settings.h"
#include "syslog.h"

using namespace hexa;

namespace {

Preferences prefs;
bool sim_mode = true;
hexa_link::Config link_cfg;

// ---------------------------------------------------------------- robot model
// One model for both modes (hexa_world): gait, IK and pulses. In sim mode its simulated
// servos, true pose, IMU and range are what the board reports. In real mode the pulses
// go to the servos, every tick in one frame to the servo controller, and /joint_states
// carries the COMMANDED angles, since a PWM servo reports nothing back; no sensor is
// published until its driver reads the real part (none yet: never a simulated one). The AHRS fuses whichever IMU samples there
// are, simulated or (later) the real QMI8658A's.
const float kTickS = 0.02f;  // 50 Hz servo frame
SimRobot robot;
uint32_t unreachable = 0;
SimImu sim_imu;
ImuSample imu_sample{};
GyroBias gyro_bias;
Ahrs ahrs;
SimRange sim_sonar;
Wall room[kRoomWalls];

// Per-servo trims (NVS "trims": 18 comma-separated us, by servo id), the vendor's
// "deviation" values read off this robot before our firmware replaced the vendor's.
int16_t trims[kJoints] = {};

// The real IMU's tilt on the board (NVS "imu_level": "roll,pitch" in degrees, each within
// +-10, measured with the body level: the console's `imu` prints it). Every sample, real or
// simulated, is turned into the body frame before the bias learner and the AHRS.
ImuMount imu_mount;

// One decimal field, optional sign: digits and at most one point.
bool parse_float(String f, float* out) {
    f.trim();
    const unsigned start = (!f.isEmpty() && (f[0] == '-' || f[0] == '+')) ? 1 : 0;
    int digits = 0, points = 0;
    for (unsigned i = start; i < f.length(); ++i) {
        if (isdigit(static_cast<unsigned char>(f[i]))) ++digits;
        else if (f[i] == '.') ++points;
        else return false;
    }
    if (digits == 0 || points > 1) return false;
    *out = f.toFloat();
    return true;
}

// "roll,pitch" in degrees, each within +-10; the mount untouched unless both are good.
bool parse_imu_level(const String& csv, ImuMount* out) {
    const int comma = csv.indexOf(',');
    float r, p;
    if (comma < 0 || !parse_float(csv.substring(0, comma), &r) || !parse_float(csv.substring(comma + 1), &p) ||
        fabsf(r) > 10 || fabsf(p) > 10)
        return false;
    out->set(r * DEG_TO_RAD, p * DEG_TO_RAD);
    return true;
}

// One integer field, optional sign, digits only.
bool parse_int(String f, long* out) {
    f.trim();
    if (f.isEmpty()) return false;
    for (unsigned i = (f[0] == '-' || f[0] == '+') ? 1 : 0; i < f.length(); ++i)
        if (!isdigit(static_cast<unsigned char>(f[i]))) return false;
    if (f.length() == 1 && !isdigit(static_cast<unsigned char>(f[0]))) return false;
    *out = f.toInt();
    return true;
}

// 18 comma-separated trims, each within +-200 us; out untouched unless all are good.
bool parse_trims(const String& csv, int16_t* out) {
    int16_t v[kJoints];
    int n = 0, from = 0;
    for (;;) {
        const int comma = csv.indexOf(',', from);
        long t = 0;
        if (n == kJoints || !parse_int(csv.substring(from, comma < 0 ? csv.length() : comma), &t) || t < -200 ||
            t > 200)
            return false;
        v[n++] = static_cast<int16_t>(t);
        if (comma < 0) break;
        from = comma + 1;
    }
    if (n != kJoints) return false;
    for (int i = 0; i < kJoints; ++i) out[i] = v[i];
    return true;
}

// The gait at boot: NVS "gait" tripod (default) or quad (hexa_gait.h quad_gait_params(): the middle
// legs up, the corner legs crawl); "com" x,y mm, the centre of mass the quadruped's sway places.
// /gait_params's fifth value and the console's `gait` switch at run time (not kept).
bool boot_quad = false;
float com_x_m = 0, com_y_m = 0;

// A mode's defaults, with the robot's centre of mass.
GaitParams gait_preset(bool quad) {
    GaitParams p = quad ? quad_gait_params() : GaitParams();
    p.com_x = com_x_m;
    p.com_y = com_y_m;
    return p;
}

void robot_init() {
    robot.init();  // slip 0.05, as the host simulator; the tripod
    // The quadruped from boot: taken at once (the gait stands), the middle feet rise in the first second.
    if (boot_quad) robot.set_gait_params(gait_preset(true));
    robot.set_trims(trims);
    sim_imu.init();
    gyro_bias.init();
    ahrs.init();
    sim_sonar.init();
    default_room(room);
}

// The real QMI8658 (real mode, when it answers; linorobot2-cockpit's driver). Its axes as
// mounted on the board, in the body frame (x forward, y left, z up): chip x points left and chip y
// back, z up: body = (-chip y, chip x, chip z), for both the accelerometer and the gyro
// (imu_to_body). Measured on the robot with body poses: a commanded roll of +10 deg read as pitch,
// a pitch as roll, until this mapping (2026-11-06); gravity on +z and the sign of z against a
// commanded turn were the first checks and hold either way.
bool real_imu = false;
ImuSample imu_to_body(const float a[3], const float g[3]) { return {-a[1], a[0], a[2], -g[1], g[0], g[2]}; }

// Idle mode (SimRobot::rest): after idle_s (NVS "idle_s", default 30; 0: never) without a
// command that moves, the robot lies down on its belly; the next command that moves stands it
// up, then walks. From boot too: a robot left on lies down after idle_s.
uint32_t idle_s = 30;

// Heading hold / turn-rate feedback (hexa_gait.h YawHold) from the bias-corrected gyro, sim or
// real; NVS "yaw_hold" (gain 1/s, default 1.0; 0 off).
YawHold yaw_hold;
// Tilt feedback (hexa_world.h TiltHold): standing, the AHRS's roll and pitch against the pose's; real
// IMU only (the simulated one does not feel a body pose, and the simulated servos do not sag); NVS
// "tilt_hold" (gain 1/s, default 1.5; 0 off).
TiltHold tilt_hold;
float last_move_s = 0;


// The final shutdown: lying down once it is asked for (its reason), then deep sleep. Nothing
// stands it up again: commands are dropped. The servo controller has no "servos off" and
// keeps holding the lying pose; the ESP32 itself drops from ~100 mA (Wi-Fi) to its sleep
// current. BOOT or EN wakes it: a boot, so the robot stands up into its boot pose.
const char* shutdown_why = nullptr;
float shutdown_t0 = 0;
constexpr float kShutdownMaxS = 15;  // a stride to finish and 3 s down; then it sleeps anyway
// A pack under low_bat_mv (NVS, default 6400: 3.2 V a cell; 0: never) for 30 s in a row while
// the robot stands or lies still. The board's monitor, read at 1 Hz with or without an agent.
LowBattery low_battery;
int battery_mv = -1;

void shutdown_request(const char* why) {
    if (shutdown_why) return;
    shutdown_why = why;
    shutdown_t0 = millis() / 1000.0f;
    robot.command({0, 0, 0}, shutdown_t0);  // the stride finishes, the gait stands, then it lies down
    hexa_log::log(hexa_log::kNotice, "shutdown (%s): lying down, then deep sleep; BOOT or EN wakes it (battery %d mV)",
                  why, battery_mv);
}

// New gait parameters from gait_params, waiting for the gait to stand (gait_callback).
GaitParams pending_gait;
bool gait_pending = false;

// The real ultrasonic (real mode, when it answers): Hiwonder's I2C module (0x77) times its
// own echo and hands back millimetres; no trigger or echo pin, so linorobot2-cockpit's
// interrupt-driven HC-SR04 driver does not apply, but its /range semantics do (range.cpp):
// "nothing within reach" is max_range, never +INF or 0 -- an out-of-span reading makes
// nav2_collision_monitor call the source invalid and stop the robot -- and the message is
// seeded clear. Span and cone: the cockpit's HC-SR04 figures until this module's are measured.
bool real_sonar = false;
constexpr float kSonarMin = 0.02f, kSonarMax = 4.0f, kSonarFov = 15.0f * 0.0174533f;

float sonar_clamped(float metres) {
    if (!std::isfinite(metres) || metres > kSonarMax || metres <= 0.0f) return kSonarMax;
    if (metres < kSonarMin) return kSonarMin;
    return metres;
}

void robot_tick(float now_s) {
    if (gait_pending && robot.set_gait_params(pending_gait)) gait_pending = false;
    if (shutdown_why || (idle_s && now_s - last_move_s > idle_s)) robot.rest(true);
    // The last tick's gyro sample against what the gait is asked to turn.
    const bool walking = !robot.gait().standing() && robot.rest_level() == 0 && (sim_mode || real_imu);
    robot.set_yaw_trim(yaw_hold.update(robot.gait().commanded().wz, imu_sample.gz, walking, kTickS));
    {
        const Quat& q = ahrs.q();
        const float mr = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        const float mp = std::asin(std::fmax(-1.0f, std::fmin(1.0f, 2 * (q.w * q.y - q.z * q.x))));
        const bool still = real_imu && ahrs.seeded() && robot.gait().standing() && robot.rest_level() == 0 && !shutdown_why;
        // Integrate only once the pose is at its target: on the ramp the error is the ramp's, and
        // integrating it overshot the next step by 1-2 deg on the robot.
        // A foot off its spot (the balance probe): the trims stay as they are, so a tip shows.
        if (robot.foot_offset_active()) {
        } else if ((!still || robot.pose_at_target()) &&
            tilt_hold.update(robot.pose_target().roll, robot.pose_target().pitch, mr, mp, still, kTickS))
            robot.set_tilt_trim(tilt_hold.roll(), tilt_hold.pitch());
    }
    unreachable += robot.step(kTickS, now_s).unreachable;
    if (sim_mode) {
        imu_sample = sim_imu.step(kTickS, robot.truth_twist());
        imu_mount.apply(&imu_sample);  // the same path as the real board's (unset: as mounted)
        // Every sample, connected or not: the estimates are settled when a session starts.
        // The gyro's bias is learned while the gait stands and taken out of every sample.
        gyro_bias.update(imu_sample.gx, imu_sample.gy, imu_sample.gz, robot.gait().standing() && robot.pose_settled(), kTickS);
        gyro_bias.correct(&imu_sample.gx, &imu_sample.gy, &imu_sample.gz);
        ahrs.update(imu_sample.gx, imu_sample.gy, imu_sample.gz, imu_sample.ax, imu_sample.ay, imu_sample.az,
                    kTickS);
    } else if (real_imu) {
        float a[3], g[3];
        if (board_io::imu_read(a, g)) {
            imu_sample = imu_to_body(a, g);
            imu_mount.apply(&imu_sample);
            // Commanded standing, not measured: the servos lag the gait by a little; the learner
            // waits 0.5 s of standing before it learns.
            gyro_bias.update(imu_sample.gx, imu_sample.gy, imu_sample.gz, robot.gait().standing() && robot.pose_settled(), kTickS);
            gyro_bias.correct(&imu_sample.gx, &imu_sample.gy, &imu_sample.gz);
            ahrs.update(imu_sample.gx, imu_sample.gy, imu_sample.gz, imu_sample.ax, imu_sample.ay, imu_sample.az,
                        kTickS);
        }
    }
    if (!sim_mode) {
        // Every tick the 18 pulses in one frame, each to be reached in one tick. The first
        // frame after boot takes the legs from wherever they hang to the stand over 1 s,
        // and nothing else is sent until it is done (a later frame would cut it short).
        static uint32_t hold_until = 0;
        static bool first = true;
        if (static_cast<int32_t>(millis() - hold_until) < 0) return;
        BusServo s[kJoints];
        for (int k = 0; k < kJoints; ++k) s[k] = {servo_id(k), static_cast<uint16_t>(robot.pulse_us(k))};
        const uint16_t ms = first ? 1000 : static_cast<uint16_t>(kTickS * 1000);
        if (board_io::servos(s, kJoints, ms) && first) {
            first = false;
            hold_until = millis() + ms;
        }
    }
}

float joint_angle(int k) { return sim_mode ? robot.joint_angle(k) : robot.commanded(k); }

// ---------------------------------------------------------------- micro-ROS base
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t cmd_sub, gait_sub, shutdown_sub, pose_sub, foot_sub;
// gait_params (std_msgs/Float32MultiArray): period s, duty, lift m, max stride m [, mode: 0 tripod,
// 1 quad]. With a mode, its defaults first (gait_preset) and a value of 0 keeps the default's: [0, 0,
// 0, 0, 1] is the quadruped as it comes. Applied when the gait next stands, if valid
// (TripodGait::params_valid); the console's `info` shows them.
std_msgs__msg__Float32MultiArray gait_msg;
float gait_msg_data[8];
// shutdown (std_msgs/String): the final shutdown when data is "shutdown"; anything else is
// ignored, so a stray empty message on the topic does not put the robot to sleep.
std_msgs__msg__String shutdown_msg;
char shutdown_msg_data[16];
// body_pose (geometry_msgs/Pose): the body's shift (position, m) and turn (orientation) over planted
// feet (hexa_world.h BodyPose; clamped, and only as far as the legs reach). Kept until the next one.
geometry_msgs__msg__Pose pose_msg;
// foot_offset (std_msgs/Float32MultiArray): leg (0-5: rf rm rr lr lm lf), dx, dy, dz (m, stance frame;
// |dx|, |dy| <= 0.02, dz 0-0.03): the balance probe (SimRobot::set_foot_offset), standing only; leg
// -1 (or an empty array) puts it back. The tilt hold keeps its trims meanwhile.
std_msgs__msg__Float32MultiArray foot_msg;
float foot_msg_data[4];

rcl_publisher_t joint_pub, odom_pub, imu_pub, sonar_pub, truth_pub, battery_pub;
#ifdef HEXA_CMD_VEL_STAMPED
geometry_msgs__msg__TwistStamped cmd_msg;
#else
geometry_msgs__msg__Twist cmd_msg;
#endif
sensor_msgs__msg__JointState joint_msg;
nav_msgs__msg__Odometry odom_msg, truth_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range sonar_msg;
sensor_msgs__msg__BatteryState battery_msg;
rosidl_runtime_c__String joint_names[kJoints];
double joint_pos[kJoints];

enum class Agent { kWaiting, kAvailable, kConnected, kDisconnected } agent = Agent::kWaiting;
uint32_t sessions = 0;  // sessions created since boot

void set_string(rosidl_runtime_c__String* s, const char* v) {
    s->data = const_cast<char*>(v);
    s->size = strlen(v);
    s->capacity = s->size + 1;
}

void cmd_callback(const void* msg) {
#ifdef HEXA_CMD_VEL_STAMPED
    const geometry_msgs__msg__Twist& t = static_cast<const geometry_msgs__msg__TwistStamped*>(msg)->twist;
#else
    const geometry_msgs__msg__Twist& t = *static_cast<const geometry_msgs__msg__Twist*>(msg);
#endif
    if (shutdown_why) return;
    const float now_s = millis() / 1000.0f;
    if (t.linear.x != 0 || t.linear.y != 0 || t.angular.z != 0) last_move_s = now_s;
    robot.command({static_cast<float>(t.linear.x), static_cast<float>(t.linear.y),
                  static_cast<float>(t.angular.z)},
                 now_s);
}

void gait_callback(const void* msg) {
    const auto* m = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    if (m->data.size != 4 && m->data.size != 5) return;
    const float* d = m->data.data;
    const bool mode = m->data.size == 5;
    if (mode && d[4] != 0 && d[4] != 1) return;
    GaitParams p = mode ? gait_preset(d[4] == 1) : robot.gait_params();
    float* const f[4] = {&p.period_s, &p.duty, &p.lift, &p.max_stride};
    for (int i = 0; i < 4; ++i)
        if (!mode || d[i] != 0) *f[i] = d[i];
    if (p.quad && p.max_period_s < p.period_s) p.max_period_s = p.period_s;
    if (!TripodGait::params_valid(p)) {
        hexa_log::log(hexa_log::kWarning, "gait_params: refused (out of range)");
        return;
    }
    pending_gait = p;
    gait_pending = true;
    hexa_log::log(hexa_log::kInfo, "gait_params: %s, period %.2f s, duty %.2f, lift %.3f m, stride %.3f m (when it stands)",
                  p.quad ? "quad" : "tripod", p.period_s, p.duty, p.lift, p.max_stride);
}

void foot_callback(const void* msg) {
    const auto* m = static_cast<const std_msgs__msg__Float32MultiArray*>(msg);
    if (shutdown_why) return;
    const float* d = m->data.data;
    if (m->data.size == 0 || (m->data.size == 4 && d[0] < 0)) {
        robot.set_foot_offset(-1, {0, 0, 0});
        hexa_log::log(hexa_log::kInfo, "foot_offset: back down");
        return;
    }
    const int leg = m->data.size == 4 ? static_cast<int>(d[0]) : -2;
    if (leg < 0 || leg >= kLegs || d[0] != leg) {
        hexa_log::log(hexa_log::kWarning, "foot_offset: refused (leg dx dy dz)");
        return;
    }
    last_move_s = millis() / 1000.0f;  // use: the idle timer starts again
    const bool ok = robot.set_foot_offset(leg, {d[1], d[2], d[3]});
    hexa_log::log(ok ? hexa_log::kInfo : hexa_log::kWarning, "foot_offset: %s %+.0f %+.0f %+.0f mm%s", kLegNames[leg],
                  d[1] * 1000, d[2] * 1000, d[3] * 1000,
                  ok ? "" : ": refused (out of range, another foot off, walking or lying)");
}

void shutdown_callback(const void* msg) {
    const auto* m = static_cast<const std_msgs__msg__String*>(msg);
    if (m->data.size == 8 && strncmp(m->data.data, "shutdown", 8) == 0) shutdown_request("/shutdown");
}

void set_body_pose(const BodyPose& p, const char* from) {
    if (shutdown_why) return;  // lying down for good
    // A pose is use: the idle timer starts again, and a robot lying down stands up for it.
    last_move_s = millis() / 1000.0f;
    robot.rest(false);
    robot.set_pose(p);
    const BodyPose& t = robot.pose_target();
    hexa_log::log(hexa_log::kInfo, "pose (%s): roll %+.1f pitch %+.1f yaw %+.1f deg  x %+.0f y %+.0f z %+.0f mm", from,
                  t.roll * RAD_TO_DEG, t.pitch * RAD_TO_DEG, t.yaw * RAD_TO_DEG, t.x * 1000, t.y * 1000, t.z * 1000);
}

void pose_callback(const void* msg) {
    const auto* m = static_cast<const geometry_msgs__msg__Pose*>(msg);
    const auto& q = m->orientation;
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(n > 0.5 && n < 1.5)) return;  // not a rotation
    const double w = q.w / n, x = q.x / n, y = q.y / n, z = q.z / n;
    BodyPose p;
    p.roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    p.pitch = std::asin(std::fmax(-1.0, std::fmin(1.0, 2 * (w * y - z * x))));
    p.yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    p.x = m->position.x;
    p.y = m->position.y;
    p.z = m->position.z;
    set_body_pose(p, "/body_pose");
}

void msgs_init() {
    shutdown_msg.data.data = shutdown_msg_data;
    shutdown_msg.data.capacity = sizeof shutdown_msg_data;
    shutdown_msg.data.size = 0;
    gait_msg.data.data = gait_msg_data;
    gait_msg.data.capacity = sizeof gait_msg_data / sizeof gait_msg_data[0];
    gait_msg.data.size = 0;
    foot_msg.data.data = foot_msg_data;
    foot_msg.data.capacity = sizeof foot_msg_data / sizeof foot_msg_data[0];
    foot_msg.data.size = 0;
    for (int k = 0; k < kJoints; ++k) set_string(&joint_names[k], kJointNames[k]);
    joint_msg.name.data = joint_names;
    joint_msg.name.size = joint_msg.name.capacity = kJoints;
    joint_msg.position.data = joint_pos;
    joint_msg.position.size = joint_msg.position.capacity = kJoints;
    set_string(&odom_msg.header.frame_id, "odom");
    set_string(&odom_msg.child_frame_id, "base_footprint");
    set_string(&truth_msg.header.frame_id, "odom");
    set_string(&truth_msg.child_frame_id, "base_footprint");
    // The simulated IMU declares the white noise it makes; the orientation is the AHRS's
    // (publish_imu), unknown (REP 145: -1) until it has seeded.
    set_string(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1;
    for (int i = 0; i < 3; ++i) {
        imu_msg.angular_velocity_covariance[i * 4] = uniform_var(sim_imu.params().gyro_noise);
        imu_msg.linear_acceleration_covariance[i * 4] = uniform_var(sim_imu.params().accel_noise);
    }
    set_string(&sonar_msg.header.frame_id, "sonar_link");
    sonar_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    sonar_msg.field_of_view = real_sonar ? kSonarFov : sim_sonar.params().fov;
    sonar_msg.min_range = real_sonar ? kSonarMin : sim_sonar.params().min_range;
    sonar_msg.max_range = real_sonar ? kSonarMax : sim_sonar.params().max_range;
    sonar_msg.range = sonar_msg.max_range;  // seeded clear
    // The real pack (2S Li-ion) in both modes, read off the board's monitor: only the voltage is
    // known; everything the board cannot measure is NaN or unknown (sensor_msgs/BatteryState).
    set_string(&battery_msg.header.frame_id, "base_link");
    set_string(&battery_msg.location, "");
    set_string(&battery_msg.serial_number, "");
    battery_msg.current = battery_msg.charge = battery_msg.capacity = NAN;
    battery_msg.design_capacity = battery_msg.percentage = battery_msg.temperature = NAN;
    battery_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    battery_msg.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
    battery_msg.present = true;
}

// Every subscription is BEST EFFORT, keep-last 1, as linorobot2-cockpit's cmdSubscriptionInit.
// rclc's default (reliable, keep-last 10) is the wrong pair for a board built with
// RMW_UXRCE_MAX_HISTORY=1 (esp32.meta): the client has ONE static input buffer, shared by every
// subscription, and with depth 10 a message arriving while one is buffered asks the empty pool
// for another and is DROPPED: the board keeps the oldest, not the newest. Depth 1 holds the
// newest, and best effort costs no acknowledgements. A reliable publisher matches a best-effort
// subscriber. The one-shot topics (gait_params, shutdown, body_pose, foot_offset) are idempotent:
// a sender that must be sure repeats the message (or reads the log line it gives).
rcl_ret_t sub_init(rcl_subscription_t* sub, const rosidl_message_type_support_t* ts, const char* topic) {
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos.depth = 1;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    return rclc_subscription_init(sub, &node, ts, topic, &qos);
}

bool entities_create() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "hexapod_base", "", &support) != RCL_RET_OK) return false;
#ifdef HEXA_CMD_VEL_STAMPED
    const rosidl_message_type_support_t* cmd_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped);
#else
    const rosidl_message_type_support_t* cmd_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);
#endif
    if (sub_init(&cmd_sub, cmd_ts, "cmd_vel") != RCL_RET_OK) return false;
    if (sub_init(&gait_sub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "gait_params") != RCL_RET_OK)
        return false;
    if (sub_init(&shutdown_sub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "shutdown") != RCL_RET_OK)
        return false;
    if (sub_init(&pose_sub, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose), "body_pose") != RCL_RET_OK)
        return false;
    if (sub_init(&foot_sub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "foot_offset") != RCL_RET_OK)
        return false;
    if (rclc_publisher_init_best_effort(&joint_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                        "joint_states") != RCL_RET_OK)
        return false;
    if (rclc_publisher_init_best_effort(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                        "odom/gait") != RCL_RET_OK)
        return false;
    if (rclc_publisher_init_best_effort(&battery_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
                                        "battery") != RCL_RET_OK)
        return false;
    if (sim_mode || real_imu) {
        if (rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                            "imu/data") != RCL_RET_OK)
            return false;
    }
    if (sim_mode || real_sonar) {
        if (rclc_publisher_init_best_effort(&sonar_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                            "sonar") != RCL_RET_OK)
            return false;
    }
    if (sim_mode) {
        if (rclc_publisher_init_best_effort(&truth_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                            "sim/ground_truth") != RCL_RET_OK)
            return false;
    }
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 5, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA) != RCL_RET_OK)
        return false;
    if (rclc_executor_add_subscription(&executor, &gait_sub, &gait_msg, &gait_callback, ON_NEW_DATA) != RCL_RET_OK)
        return false;
    if (rclc_executor_add_subscription(&executor, &shutdown_sub, &shutdown_msg, &shutdown_callback, ON_NEW_DATA) !=
        RCL_RET_OK)
        return false;
    if (rclc_executor_add_subscription(&executor, &pose_sub, &pose_msg, &pose_callback, ON_NEW_DATA) != RCL_RET_OK)
        return false;
    if (rclc_executor_add_subscription(&executor, &foot_sub, &foot_msg, &foot_callback, ON_NEW_DATA) != RCL_RET_OK)
        return false;
    rmw_uros_sync_session(1000);
    return true;
}

void entities_destroy() {
    rmw_context_t* rmw = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
    if (sim_mode) rcl_publisher_fini(&truth_pub, &node);
    if (sim_mode || real_sonar) rcl_publisher_fini(&sonar_pub, &node);
    if (sim_mode || real_imu) rcl_publisher_fini(&imu_pub, &node);
    rcl_publisher_fini(&battery_pub, &node);
    rcl_publisher_fini(&odom_pub, &node);
    rcl_publisher_fini(&joint_pub, &node);
    rcl_subscription_fini(&cmd_sub, &node);
    rcl_subscription_fini(&gait_sub, &node);
    rcl_subscription_fini(&shutdown_sub, &node);
    rcl_subscription_fini(&pose_sub, &node);
    rcl_subscription_fini(&foot_sub, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void stamp(builtin_interfaces__msg__Time* t) {
    const int64_t ns = rmw_uros_epoch_nanos();
    t->sec = static_cast<int32_t>(ns / 1000000000LL);
    t->nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

void set_pose(nav_msgs__msg__Odometry* m, const Pose2& p) {
    m->pose.pose.position.x = p.x;
    m->pose.pose.position.y = p.y;
    m->pose.pose.orientation.z = sin(p.yaw / 2);
    m->pose.pose.orientation.w = cos(p.yaw / 2);
}

void publish_state() {
    stamp(&joint_msg.header.stamp);
    for (int k = 0; k < kJoints; ++k) joint_pos[k] = joint_angle(k);
    rcl_publish(&joint_pub, &joint_msg, nullptr);

    odom_msg.header.stamp = joint_msg.header.stamp;
    set_pose(&odom_msg, robot.odom());
    const BodyTwist tw = robot.odom_twist();
    odom_msg.twist.twist.linear.x = tw.vx;
    odom_msg.twist.twist.linear.y = tw.vy;
    odom_msg.twist.twist.angular.z = tw.wz;
    const OdomVariance var = gait_odom_variance(robot.gait().standing());
    for (int i = 0; i < 6; ++i) {
        odom_msg.pose.covariance[i * 7] = var.pose[i];
        odom_msg.twist.covariance[i * 7] = var.twist[i];
    }
    rcl_publish(&odom_pub, &odom_msg, nullptr);
}

void publish_truth() {
    stamp(&truth_msg.header.stamp);
    set_pose(&truth_msg, robot.truth());
    const BodyTwist& tt = robot.truth_twist();
    truth_msg.twist.twist.linear.x = tt.vx;
    truth_msg.twist.twist.linear.y = tt.vy;
    truth_msg.twist.twist.angular.z = tt.wz;
    rcl_publish(&truth_pub, &truth_msg, nullptr);
}

void publish_imu() {
    stamp(&imu_msg.header.stamp);
    imu_msg.linear_acceleration.x = imu_sample.ax;
    imu_msg.linear_acceleration.y = imu_sample.ay;
    imu_msg.linear_acceleration.z = imu_sample.az;
    imu_msg.angular_velocity.x = imu_sample.gx;
    imu_msg.angular_velocity.y = imu_sample.gy;
    imu_msg.angular_velocity.z = imu_sample.gz;
    if (ahrs.seeded()) {
        const Quat& q = ahrs.q();
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        // Roll and pitch levelled against gravity; yaw unanchored (no magnetometer).
        imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = ahrs.params().roll_pitch_var;
        imu_msg.orientation_covariance[8] = ahrs.params().yaw_var;
    }
    rcl_publish(&imu_pub, &imu_msg, nullptr);
}

// 1 Hz: the last reading of the board's battery monitor; a board without one publishes nothing.
void publish_battery() {
    if (battery_mv <= 0) return;
    stamp(&battery_msg.header.stamp);
    battery_msg.voltage = battery_mv / 1000.0f;
    rcl_publish(&battery_pub, &battery_msg, nullptr);
}

// The real ultrasonic at 10 Hz, with or without an agent: the reading for /sonar, and its two
// lights: blue clear, yellow under kNearM, red under kCloseM (20 mm of hysteresis), a green
// heartbeat while the robot lies down (idle mode: idle_heartbeat), off at the final shutdown.
// Solid colours are written only on a change; each obstacle change is logged.
constexpr float kNearM = 0.40f, kCloseM = 0.20f;
enum Light { kClear, kNear, kClose, kIdle, kOff };
int sonar_level = -1;  // kClear, kNear, kClose by range; -1 none yet
int light_shown = -1;
void sonar_light(int light) {
    if (light == light_shown) return;
    if (light == kIdle) {  // drawn by idle_heartbeat()
        light_shown = kIdle;
        return;
    }
    static const uint8_t rgb[5][3] = {{0, 0, 255}, {255, 160, 0}, {255, 0, 0}, {0, 24, 0}, {0, 0, 0}};
    if (board_io::sonar_rgb(rgb[light][0], rgb[light][1], rgb[light][2])) light_shown = light;  // else next read
}
// Idle: a heartbeat on the lights ("lub-dub" every 1.6 s, green, never fully dark), so a robot
// lying still shows it is alive: a frozen board leaves a steady light. 25 Hz, two I2C writes
// (~1.5 ms at 100 kHz) while it lies down only.
void idle_heartbeat(float t) {
    const float ph = std::fmod(t, 1.6f);
    auto bump = [](float x, float c) { return std::exp(-((x - c) / 0.07f) * ((x - c) / 0.07f)); };
    const uint8_t g = static_cast<uint8_t>(6 + 90 * (bump(ph, 0.15f) + 0.6f * bump(ph, 0.45f)));
    static uint8_t last = 0;
    if (g != last && board_io::sonar_rgb(0, g, 0)) last = g;
}

void read_sonar() {
    const int mm = board_io::sonar_mm();
    if (mm >= 0) {  // a bus error keeps the last reading
        sonar_msg.range = sonar_clamped(mm / 1000.0f);
        const float r = sonar_msg.range, h = sonar_level < 0 ? 0.0f : 0.02f;
        const int level =
            r < kCloseM + (sonar_level == kClose ? h : 0) ? kClose : r < kNearM + (sonar_level >= kNear ? h : 0) ? kNear : kClear;
        if (level != sonar_level && sonar_level >= 0)
            hexa_log::log(level == kClose ? hexa_log::kWarning : hexa_log::kInfo, "sonar: %s at %.2f m",
                          level == kClose ? "obstacle close" : level == kNear ? "obstacle near" : "clear", r);
        sonar_level = level;
    }
    if (sonar_level >= 0) sonar_light(robot.resting() ? kIdle : sonar_level);
}

void publish_sonar() {
    stamp(&sonar_msg.header.stamp);
    if (real_sonar) {
        rcl_publish(&sonar_pub, &sonar_msg, nullptr);
        return;
    }
    sonar_msg.range = sim_sonar.measure(robot.truth(), room, kRoomWalls);
    rcl_publish(&sonar_pub, &sonar_msg, nullptr);
}

void agent_step() {
    static uint32_t last_check = 0;
    const uint32_t now = millis();
    switch (agent) {
        case Agent::kWaiting:
            if (now - last_check > 500 && hexa_link::network_up()) {
                last_check = now;
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) agent = Agent::kAvailable;
            }
            break;
        case Agent::kAvailable:
            agent = entities_create() ? Agent::kConnected : Agent::kWaiting;
            if (agent == Agent::kConnected) {
                ++sessions;
                hexa_log::log(hexa_log::kInfo, "agent: session %u up", static_cast<unsigned>(sessions));
            }
            if (agent == Agent::kWaiting) entities_destroy();
            break;
        case Agent::kConnected:
            if (now - last_check > 1000) {
                last_check = now;
                // Serial answers a ping within a few ms, so one miss is a lost agent. Wi-Fi
                // replies take 3-200 ms (median ~20): a single late one tore the session down
                // 23 times in 2.5 min (agent log), each costing every entity. There a ping
                // waits 50 ms, the loop's budget, and only 5 misses in a row (5 s) disconnect.
                const bool wifi = link_cfg.kind == hexa_link::Kind::kWifi;
                static int misses = 0;
                misses = rmw_uros_ping_agent(wifi ? 50 : 100, 1) == RMW_RET_OK ? 0 : misses + 1;
                if (misses >= (wifi ? 5 : 1)) {
                    misses = 0;
                    agent = Agent::kDisconnected;
                    break;
                }
            }
            // Timeout 0: one non-blocking read of what is buffered. With 1 ms the XRCE client's
            // framed read takes each read's elapsed time off its timeout in whole milliseconds;
            // reads returning in under 1 ms take off 0, and the loop sometimes never left
            // (core dump: loopTask in uxr_read_framed_msg under spin_some; about half the
            // sessions at full load). 0 makes each of its loops run once; the 50 Hz loop polls.
            rclc_executor_spin_some(&executor, 0);
            break;
        case Agent::kDisconnected:
            hexa_log::log(hexa_log::kWarning, "agent: session %u lost", static_cast<unsigned>(sessions));
            entities_destroy();
            agent = Agent::kWaiting;
            break;
    }
}

void console_info();

// The ESP32's lowest state that BOOT can end: deep sleep, the RTC domain kept for the
// BOOT pin's wake (GPIO0, pressed = low). Wi-Fi and the CPUs are off; a wake is a boot.
void deep_sleep() {
    if (real_sonar) sonar_light(kOff);  // the module stays powered through the sleep
    hexa_log::log(hexa_log::kNotice, "shutdown: deep sleep now; press BOOT or EN to wake");
    Serial.flush();
    delay(100);  // the datagram out before the radio stops
    board_io::before_deep_sleep();
    pinMode(HEXA_LED_PIN, OUTPUT);
    digitalWrite(HEXA_LED_PIN, LOW);
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(HEXA_BOOT_PIN));
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(HEXA_BOOT_PIN));
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(HEXA_BOOT_PIN), 0);
    esp_deep_sleep_start();
}

void base_setup() {
    robot_init();
    msgs_init();
    hexa_link::set_microros_transport(link_cfg);
    // A Wi-Fi link leaves Serial free: say what is running.
    if (link_cfg.kind == hexa_link::Kind::kWifi) {
        Serial.println("\nminiHexa base");
        console_info();
    }
}

void base_loop() {
    static uint32_t next_tick = millis();
    static uint32_t frame = 0;
    // Loop health on a Wi-Fi link (Serial is free there): the slowest loop and the ticks
    // that ran more than one frame late, every 30 s.
    static uint32_t loop_max_ms = 0, late_ticks = 0, last_report = 0, loop_t0 = millis();
    const uint32_t t_start = millis();
    loop_max_ms = max(loop_max_ms, t_start - loop_t0);
    loop_t0 = t_start;
    if (link_cfg.kind == hexa_link::Kind::kWifi && t_start - last_report > 30000) {
        last_report = t_start;
        hexa_log::log(hexa_log::kInfo,
                      "health: battery %d mV  rssi %d dBm  sessions %u  loop max %u ms  late ticks %u  gait %s%s  "
                      "cycle %.2f s  yaw lost %+.3f rad  pose %+.0f/%+.0f/%+.0f deg %+.0f mm  tilt trim %+.1f/%+.1f  sonar %.2f m  unreachable %u",
                      battery_mv, static_cast<int>(WiFi.RSSI()), static_cast<unsigned>(sessions),
                      static_cast<unsigned>(loop_max_ms), static_cast<unsigned>(late_ticks),
                      robot.resting() ? "lying" : robot.rest_level() > 0 ? "between" : robot.gait().standing() ? "standing" : "walking",
                      robot.gait_params().quad ? " (quad)" : "", robot.gait().period(), yaw_hold.error(), robot.pose().roll * RAD_TO_DEG,
                      robot.pose().pitch * RAD_TO_DEG, robot.pose().yaw * RAD_TO_DEG, robot.pose().z * 1000,
                      tilt_hold.roll() * RAD_TO_DEG, tilt_hold.pitch() * RAD_TO_DEG, sonar_msg.range, static_cast<unsigned>(unreachable));
        loop_max_ms = late_ticks = 0;
    }
    agent_step();
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - next_tick) >= 0) {
        if (static_cast<int32_t>(now - next_tick) > static_cast<int32_t>(kTickS * 1000)) ++late_ticks;
        next_tick += static_cast<uint32_t>(kTickS * 1000);
        if (static_cast<int32_t>(now - next_tick) > 100) next_tick = now;  // fell behind: no burst
        robot_tick(now / 1000.0f);
        ++frame;
        if (real_sonar && frame % 5 == 0) read_sonar();
        if (real_sonar && light_shown == kIdle && frame % 2 == 0) idle_heartbeat(now / 1000.0f);
        // Idle and the gait's backstop, in the log.
        static bool was_resting = false;
        static int forced = 0;
        if (robot.resting() != was_resting) {
            was_resting = robot.resting();
            hexa_log::log(hexa_log::kInfo, "idle: %s", was_resting ? "lying down" : "standing up (a command moved it)");
        }
        if (robot.gait().forced_stands() != forced) {
            forced = robot.gait().forced_stands();
            hexa_log::log(hexa_log::kErr, "gait: a stop did not settle in two cycles; feet put down (%d so far)", forced);
        }
        if (frame % 50 == 25) {
            battery_mv = board_io::battery_i2c_mv();
            if (low_battery.update(battery_mv, robot.gait().standing())) shutdown_request("battery low");
        }
        // Lying down (the last frame went out this tick), or out of time: the session closed
        // so the agent drops it at once, then sleep.
        if (shutdown_why && (robot.resting() || now / 1000.0f - shutdown_t0 > kShutdownMaxS)) {
            if (agent == Agent::kConnected) entities_destroy();
            deep_sleep();
        }
        // The serial link's budget (docs/firmware.md): ~50 KB/s of messages, about half of
        // 921600 baud. The full set at 25 Hz ground truth was ~65 KB/s and saturated it: rates
        // fell, and on Lyrical the loop sometimes hung (the watchdog in setup() now resets it).
        if (agent == Agent::kConnected) {
            if (frame % 2 == 0) publish_state();  // 25 Hz: joint_states 452 B, odom 732 B
            if (frame % 50 == 25) publish_battery();  // 1 Hz, ~100 B
            if (real_imu) publish_imu();                   // 50 Hz: the real QMI8658
            if (real_sonar && frame % 5 == 0) publish_sonar();  // 10 Hz: the real ultrasonic (read_sonar)
            if (sim_mode) {
                publish_imu();                              // 50 Hz, 332 B
                if (frame % 5 == 0) publish_sonar();        // 10 Hz, 56 B: an HC-SR04's pace
                if (frame % 10 == 1) publish_truth();       //  5 Hz, 732 B: evaluation only
            }
        }
    }
}

// ---------------------------------------------------------------- console
String line;
bool console = false;

void console_info() {
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    Serial.printf("chip %s rev %d cores %d flash %u MB\n", ESP.getChipModel(), ci.revision, ci.cores,
                  static_cast<unsigned>(ESP.getFlashChipSize() / (1024 * 1024)));
    Serial.printf("mac %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("last reset: %s\n", esp_reset_reason() == ESP_RST_TASK_WDT ? "loop watchdog (the loop hung)"
                                      : esp_reset_reason() == ESP_RST_PANIC  ? "panic"
                                      : esp_reset_reason() == ESP_RST_POWERON ? "power-on or EN"
                                      : esp_reset_reason() == ESP_RST_DEEPSLEEP ? "woken from the final shutdown"
                                                                              : "other");
    Serial.printf("app %s  sim %s  board %s\n", prefs.getString("app", "base").c_str(), sim_mode ? "on" : "off",
                  HEXA_BOARD_VERIFIED ? "verified" : "NOT identified (real mode refused)");
    Serial.printf("imu %s  mount roll %+.2f pitch %+.2f deg\n",
                  sim_mode ? "simulated" : real_imu ? "QMI8658 (real)" : "none (the QMI8658 did not answer)",
                  imu_mount.roll() * RAD_TO_DEG, imu_mount.pitch() * RAD_TO_DEG);
    Serial.printf("sonar %s\n", sim_mode ? "simulated" : real_sonar ? "I2C 0x77 (real)" : "none (did not answer)");
    const GaitParams& gp = robot.gait_params();
    Serial.printf("gait %s  period %.2f-%.2f s (in use %.2f)  duty %.2f  lift %.3f m  max stride %.3f m%s\n",
                  gp.quad ? "quad" : "tripod", gp.period_s, gp.max_period_s, robot.gait().period(), gp.duty, gp.lift,
                  gp.max_stride, gait_pending ? "  (new ones pending: applied when it stands)" : "");
    if (gp.quad)
        Serial.printf("quad: margin %.0f mm  sway %.2f m/s (now %+.0f, %+.0f mm)  centre of mass %+.1f, %+.1f mm  middle feet %s\n",
                      gp.margin * 1000, gp.sway_speed, robot.gait().sway_x() * 1000, robot.gait().sway_y() * 1000,
                      gp.com_x * 1000, gp.com_y * 1000, robot.gait().mids_home() ? "up" : "on their way");
    Serial.printf("yaw hold gain %.2f /s (0: off)  tilt hold gain %.2f /s, trim roll %+.1f pitch %+.1f deg\n",
                  yaw_hold.params().gain, tilt_hold.params().gain, tilt_hold.roll() * RAD_TO_DEG,
                  tilt_hold.pitch() * RAD_TO_DEG);
    const BodyPose& bp = robot.pose();
    Serial.printf("body pose roll %+.1f pitch %+.1f yaw %+.1f deg  x %+.0f y %+.0f z %+.0f mm%s\n", bp.roll * RAD_TO_DEG,
                  bp.pitch * RAD_TO_DEG, bp.yaw * RAD_TO_DEG, bp.x * 1000, bp.y * 1000, bp.z * 1000,
                  robot.pose_settled() ? "" : " (moving)");
    Serial.printf("idle rest after %u s (0: never)  now %s\n", static_cast<unsigned>(idle_s),
                  robot.resting() ? "lying down" : robot.rest_level() > 0 ? "between" : "up");
    Serial.printf("shutdown when the battery stays under %d mV for 30 s (0: never)  battery %d mV\n",
                  low_battery.floor_mv(), board_io::battery_i2c_mv());
    Serial.printf("settings over Wi-Fi at boot: %s\n", hexa_settings::last_result());
    Serial.printf("syslog %s\n", hexa_log::sink());
    Serial.printf("build %s %s\n", __DATE__, __TIME__);
    hexa_link::print_status(Serial);
}

void console_i2c() {
    board_io::begin();
    int found = 0;
    for (uint8_t a = 1; a < 127; ++a) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("i2c 0x%02X\n", a);
            ++found;
        }
    }
    Serial.printf("i2c scan on SDA %d SCL %d: %d device(s)\n", HEXA_I2C_SDA, HEXA_I2C_SCL, found);
}

void console_command(String cmd) {
    cmd.trim();
    if (cmd == "info") {
        console_info();
    } else if (cmd == "link") {
        hexa_link::print_status(Serial);
    } else if (cmd == "i2c") {
        console_i2c();
    } else if (cmd.startsWith("servo ")) {
        // One servo, for the bench: the robot on a stand, one joint at a time. Never while a
        // gait drives the servos (real mode, not possible until the board is verified).
        unsigned id = 0, us = 0, ms = 500;
        const int got = sscanf(cmd.c_str() + 6, "%u %u %u", &id, &us, &ms);
        if (!sim_mode) Serial.println("refused: the gait drives the servos in real mode");
        else if (got < 2 || ms > 5000 || !board_io::servo(id, us, ms))
            Serial.println("usage: servo <id 1-18> <us 500-2500> [ms <= 5000]");
        else Serial.printf("servo %u -> %u us over %u ms\n", id, us, ms);
    } else if (cmd.startsWith("foot ")) {
        // One leg's foot moved by (dx, dy, dz) mm from its standing position (body frame:
        // x forward, y left, z up), through the gait's IK and the servo calibration: the
        // bench check of each leg's mapping, on a stand, in sim mode only.
        int leg = -1, ms = 600;
        float d[3] = {0, 0, 0};
        const int got = sscanf(cmd.c_str() + 5, "%d %f %f %f %d", &leg, &d[0], &d[1], &d[2], &ms);
        const Geometry& g = minihexa_geometry();
        Vec3 f{};
        LegAngles a{};
        if (!sim_mode) {
            Serial.println("refused: the gait drives the servos in real mode");
        } else if (got < 4 || leg < 0 || leg >= kLegs || ms < 100 || ms > 5000 ||
                   std::fabs(d[0]) > 40 || std::fabs(d[1]) > 40 || std::fabs(d[2]) > 40) {
            Serial.println("usage: foot <leg 0-5: rf rm rr lr lm lf> <dx> <dy> <dz mm, |d| <= 40> [ms 100-5000]");
        } else if (f = neutral_foot(g, leg), f.x += d[0] / 1000, f.y += d[1] / 1000, f.z += d[2] / 1000,
                   !leg_ik(g, leg, f, &a)) {
            Serial.println("foot: out of reach");
        } else {
            const float ang[3] = {a.coxa, a.femur, a.tibia};
            BusServo s[3];
            bool clamped = false;
            for (int j = 0; j < 3; ++j) {
                ServoCal c = default_cal(leg * 3 + j);
                c.trim_us = trims[servo_id(leg * 3 + j) - 1];
                bool cl = false;
                s[j] = {servo_id(leg * 3 + j), static_cast<uint16_t>(angle_to_us(c, ang[j], &cl))};
                clamped |= cl;
            }
            board_io::servos(s, 3, static_cast<uint16_t>(ms));
            Serial.printf("foot %s -> ids %u %u %u = %u %u %u us%s\n", kLegNames[leg], s[0].id, s[1].id, s[2].id,
                          s[0].us, s[1].us, s[2].us, clamped ? " (clamped)" : "");
        }
    } else if (cmd == "trims") {
        Serial.print("trims (us, servo id 1-18):");
        for (int i = 0; i < kJoints; ++i) Serial.printf(" %d", trims[i]);
        Serial.println();
    } else if (cmd == "bat") {
        Serial.printf("battery adc %d mV  i2c %d mV\n", board_io::battery_adc_mv(), board_io::battery_i2c_mv());
    } else if (cmd == "sonar") {
        Serial.printf("sonar %d mm\n", board_io::sonar_mm());
    } else if (cmd == "imu") {
        Serial.printf("imu who_am_i %d (QMI8658: 5)\n", board_io::imu_who_am_i());
        float a[3], g[3];
        if (board_io::imu_read(a, g))
        {
            const ImuSample b = imu_to_body(a, g);
            Serial.printf("imu accel %+.3f %+.3f %+.3f m/s2  gyro %+.4f %+.4f %+.4f rad/s (body axes)\n", b.ax, b.ay,
                          b.az, b.gx, b.gy, b.gz);
            // Still, with the body level (a level on the top plate): this is the robot's imu_level.
            float r, p;
            ImuMount::level_from(b, &r, &p);
            Serial.printf("imu tilt of this reading: imu_level=%.2f,%.2f (in use %.2f,%.2f)\n", r * RAD_TO_DEG,
                          p * RAD_TO_DEG, imu_mount.roll() * RAD_TO_DEG, imu_mount.pitch() * RAD_TO_DEG);
        }
        board_io::imu_status(Serial);
    } else if (cmd == "app base" || cmd == "app console") {
        prefs.putString("app", cmd.substring(4));
        Serial.printf("app = %s (after reboot)\n", cmd.substring(4).c_str());
    } else if (cmd == "sim on") {
        prefs.putBool("sim", true);
        Serial.println("sim = on (after reboot)");
    } else if (cmd == "sim off") {
        if (!HEXA_BOARD_VERIFIED) {
            Serial.println("refused: board pin map not identified (board_minihexa.h)");
        } else {
            prefs.putBool("sim", false);
            Serial.println("sim = off (after reboot)");
        }
    } else if (cmd == "reboot") {
        ESP.restart();
    } else if (cmd == "gait quad" || cmd == "gait tripod") {
        // The mode at run time (base app; not kept: NVS "gait" is the boot's), taken when it stands.
        if (console) Serial.println("refused: the console app drives no gait");
        else {
            pending_gait = gait_preset(cmd == "gait quad");
            gait_pending = true;
            Serial.printf("gait %s: taken when the robot stands\n", cmd.c_str() + 5);
        }
    } else if (cmd.startsWith("pose")) {
        // The body pose from the console (base app on Wi-Fi): roll pitch yaw in degrees, x y z in mm;
        // "pose" alone goes back to neutral.
        float v[6] = {0, 0, 0, 0, 0, 0};
        const int got = sscanf(cmd.c_str() + 4, "%f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
        if (console) Serial.println("refused: the console app drives no gait");
        else if (got > 0 || cmd == "pose")
            set_body_pose({v[3] / 1000, v[4] / 1000, v[5] / 1000, v[0] * DEG_TO_RAD, v[1] * DEG_TO_RAD, v[2] * DEG_TO_RAD},
                          "console");
        else Serial.println("usage: pose [roll pitch yaw (deg) [x y z (mm)]]");
    } else if (cmd == "shutdown") {
        // The base app lies down first; the console drives no gait, so it sleeps at once.
        if (console) deep_sleep();
        else shutdown_request("console");
    } else if (cmd.length()) {
        Serial.println("commands: info | link | i2c | servo <id> <us> [ms] | foot <leg> <dx> <dy> <dz> [ms] | trims | bat | sonar | imu | app base|console | sim on|off | gait tripod|quad | reboot | shutdown | pose [r p y [x y z]]");
    }
}

void console_setup() {
    Serial.println("\nminiHexa console");
    console_info();
    Serial.print("> ");
}

void console_loop() {
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\r' || c == '\n') {
            Serial.println();
            console_command(line);
            line = "";
            Serial.print("> ");
        } else if (line.length() < 64) {
            line += c;
            Serial.print(c);
        }
    }
}

// BOOT pressed in the first two seconds -> console. Polled, so a reset into the ROM
// bootloader (BOOT held through reset) never gets here.
bool boot_button_pressed(uint32_t window_ms) {
    pinMode(HEXA_BOOT_PIN, INPUT_PULLUP);
    const uint32_t t0 = millis();
    while (millis() - t0 < window_ms) {
        if (digitalRead(HEXA_BOOT_PIN) == LOW) return true;
        delay(10);
    }
    return false;
}

}  // namespace

void setup() {
    // A loop stuck for 5 s (the task watchdog's timeout) panics and reboots: the board comes
    // back and reconnects, rather than freezing the gait mid-stride. `info` shows the reason.
    enableLoopWDT();
    Serial.begin(HEXA_SERIAL_BAUD);
    prefs.begin("hexa", false);
    hexa_settings::apply_staged(prefs);  // settings sent over Wi-Fi, before anything reads NVS
    sim_mode = prefs.getBool("sim", true) || !HEXA_BOARD_VERIFIED;
    // The real IMU in real mode only: sim mode publishes the simulated one, never both.
    if (!sim_mode) {
        real_imu = board_io::imu_begin();
        // The module may be measuring and not answer one read: it is there if any of 10 do.
        for (int i = 0; i < 10 && !real_sonar; ++i) {
            real_sonar = board_io::sonar_mm() >= 0;
            if (!real_sonar) delay(50);
        }
    }
    const String trims_csv = prefs.getString("trims", "");
    if (!trims_csv.isEmpty() && !parse_trims(trims_csv, trims)) Serial.println("trims: NVS value refused, none used");
    long idle = 0;
    const String idle_str = prefs.getString("idle_s", "");
    if (!idle_str.isEmpty()) {
        if (parse_int(idle_str, &idle) && idle >= 0 && idle <= 3600) idle_s = static_cast<uint32_t>(idle);
        else Serial.println("idle_s: NVS value refused, 30 s used");
    }
    const String tilt_str = prefs.getString("tilt_hold", "");
    if (!tilt_str.isEmpty()) {
        const float g = tilt_str.toFloat();
        if (g >= 0 && g <= 5) tilt_hold.set_gain(g);
        else Serial.println("tilt_hold: NVS value refused, 1.5 used");
    }
    const String gait_str = prefs.getString("gait", "");
    if (gait_str == "quad") boot_quad = true;
    else if (!gait_str.isEmpty() && gait_str != "tripod") Serial.println("gait: NVS value refused, tripod used");
    const String com_str = prefs.getString("com", "");
    if (!com_str.isEmpty()) {
        const int comma = com_str.indexOf(',');
        const float cx = com_str.substring(0, comma).toFloat(), cy = com_str.substring(comma + 1).toFloat();
        if (comma > 0 && std::fabs(cx) <= 30 && std::fabs(cy) <= 30) {
            com_x_m = cx / 1000;
            com_y_m = cy / 1000;
        } else {
            Serial.println("com: NVS value refused, 0,0 used");
        }
    }
    const String yaw_str = prefs.getString("yaw_hold", "");
    if (!yaw_str.isEmpty()) {
        const float g = yaw_str.toFloat();
        if (g >= 0 && g <= 5) yaw_hold.set_gain(g);
        else Serial.println("yaw_hold: NVS value refused, 1.0 used");
    }
    long low_bat = 6400;
    const String low_bat_str = prefs.getString("low_bat_mv", "");
    if (!low_bat_str.isEmpty() &&
        !(parse_int(low_bat_str, &low_bat) && (low_bat == 0 || (low_bat >= 5600 && low_bat <= 8000)))) {
        Serial.println("low_bat_mv: NVS value refused, 6400 mV used");
        low_bat = 6400;
    }
    low_battery.init(static_cast<int>(low_bat));
    const String level_csv = prefs.getString("imu_level", "");
    if (!level_csv.isEmpty() && !parse_imu_level(level_csv, &imu_mount))
        Serial.println("imu_level: NVS value refused, the board taken as level");
    link_cfg = hexa_link::load(prefs);
    hexa_link::begin(link_cfg);
    // The loop stops for an update's length; the gait is told to stand first.
    hexa_link::on_ota([]() { robot.command({0, 0, 0}, millis() / 1000.0f); }, nullptr);
    // A wake from the final shutdown is a BOOT press: its release waited for, so the press
    // that woke the board is not taken as "console".
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        rtc_gpio_deinit(static_cast<gpio_num_t>(HEXA_BOOT_PIN));  // back from the RTC domain
        pinMode(HEXA_BOOT_PIN, INPUT_PULLUP);
        for (int i = 0; i < 500 && digitalRead(HEXA_BOOT_PIN) == LOW; ++i) delay(10);
    }
    console = boot_button_pressed(2000) || prefs.getString("app", "base") == "console";
    hexa_log::begin(prefs, link_cfg.kind == hexa_link::Kind::kWifi, link_cfg.kind == hexa_link::Kind::kWifi || console);
    const esp_reset_reason_t why = esp_reset_reason();
    hexa_log::log(hexa_log::kNotice, "boot: %s, %s %s, app %s, sim %s, reset %s, settings %s, idle %u s, low_bat %d mV",
                  hexa_link::elf_sha(), __DATE__, __TIME__, console ? "console" : "base", sim_mode ? "on" : "off",
                  why == ESP_RST_POWERON ? "power-on/EN" : why == ESP_RST_DEEPSLEEP ? "deep-sleep wake"
                  : why == ESP_RST_TASK_WDT ? "loop watchdog" : why == ESP_RST_PANIC ? "panic"
                  : why == ESP_RST_SW ? "software" : "other",
                  hexa_settings::last_result(), static_cast<unsigned>(idle_s), low_battery.floor_mv());
    if (console) console_setup();
    else base_setup();
}

void loop() {
    hexa_link::loop();
    hexa_log::loop(hexa_link::network_up());
    hexa_settings::trial_loop(prefs, hexa_link::network_up());
    if (console) {
        console_loop();
    } else {
        base_loop();
        if (link_cfg.kind == hexa_link::Kind::kWifi) console_loop();  // Serial is free
    }
}
