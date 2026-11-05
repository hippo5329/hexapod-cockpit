// miniHexa firmware: one binary, the application chosen by the NVS `app` key.
//
//   base     micro-ROS node: /cmd_vel in; /joint_states and /odom/gait out, and in sim
//            mode the simulated /imu/data (with the board's AHRS orientation), /sonar
//            and /sim/ground_truth. The gait
//            ticks at 50 Hz whether or not the agent is there; a lost command stream
//            ends in a stand (hexa_gait's timeout), never a frozen stride.
//   console  line commands on the serial port: info, link, i2c, servo, bat, sonar, imu, app,
//            sim, reboot. On a Wi-Fi link the base app takes them too (Serial is free).
//
// The link (NVS `link`, link.h): micro-ROS over USB serial, or over UDP on Wi-Fi with
// ArduinoOTA beside it. On Wi-Fi the console and OTA work in either app.
//
// Press BOOT within the first two seconds after a reset to get the console whatever
// `app` says. Sim mode (NVS `sim`, default on) drives simulated servos; real mode is
// refused until board_minihexa.h is verified.
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_chip_info.h>
#include <esp_mac.h>
#include <esp_system.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
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
#include "hexa_sensors.h"
#include "hexa_world.h"
#include "link.h"

using namespace hexa;

namespace {

Preferences prefs;
bool sim_mode = true;
hexa_link::Config link_cfg;

// ---------------------------------------------------------------- robot model
// One model for both modes (hexa_world): gait, IK and pulses. In sim mode its simulated
// servos, true pose, IMU and range are what the board reports. In real mode the pulses
// go to the servos (none until the board is identified) and /joint_states carries the
// COMMANDED angles, since a PWM servo reports nothing back; no sensor is published until
// its bus and pins are read off the board. The AHRS fuses whichever IMU samples there
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

void robot_init() {
    robot.init();  // slip 0.05, as the host simulator
    sim_imu.init();
    gyro_bias.init();
    ahrs.init();
    sim_sonar.init();
    default_room(room);
}

void robot_tick(float now_s) {
    unreachable += robot.step(kTickS, now_s).unreachable;
    if (sim_mode) {
        imu_sample = sim_imu.step(kTickS, robot.truth_twist());
        // Every sample, connected or not: the estimates are settled when a session starts.
        // The gyro's bias is learned while the gait stands and taken out of every sample.
        gyro_bias.update(imu_sample.gx, imu_sample.gy, imu_sample.gz, robot.gait().standing(), kTickS);
        gyro_bias.correct(&imu_sample.gx, &imu_sample.gy, &imu_sample.gz);
        ahrs.update(imu_sample.gx, imu_sample.gy, imu_sample.gz, imu_sample.ax, imu_sample.ay, imu_sample.az,
                    kTickS);
    }
    // Real output: robot.pulse_us(k) to each servo, once the board is identified.
}

float joint_angle(int k) { return sim_mode ? robot.joint_angle(k) : robot.commanded(k); }

// ---------------------------------------------------------------- micro-ROS base
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t cmd_sub;
rcl_publisher_t joint_pub, odom_pub, imu_pub, sonar_pub, truth_pub;
#ifdef HEXA_CMD_VEL_STAMPED
geometry_msgs__msg__TwistStamped cmd_msg;
#else
geometry_msgs__msg__Twist cmd_msg;
#endif
sensor_msgs__msg__JointState joint_msg;
nav_msgs__msg__Odometry odom_msg, truth_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range sonar_msg;
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
    robot.command({static_cast<float>(t.linear.x), static_cast<float>(t.linear.y),
                  static_cast<float>(t.angular.z)},
                 millis() / 1000.0f);
}

void msgs_init() {
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
    sonar_msg.field_of_view = sim_sonar.params().fov;
    sonar_msg.min_range = sim_sonar.params().min_range;
    sonar_msg.max_range = sim_sonar.params().max_range;
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
    if (rclc_subscription_init_default(&cmd_sub, &node, cmd_ts, "cmd_vel") != RCL_RET_OK) return false;
    if (rclc_publisher_init_best_effort(&joint_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                        "joint_states") != RCL_RET_OK)
        return false;
    if (rclc_publisher_init_best_effort(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                        "odom/gait") != RCL_RET_OK)
        return false;
    if (sim_mode) {
        if (rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                            "imu/data") != RCL_RET_OK)
            return false;
        if (rclc_publisher_init_best_effort(&sonar_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                            "sonar") != RCL_RET_OK)
            return false;
        if (rclc_publisher_init_best_effort(&truth_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                                            "sim/ground_truth") != RCL_RET_OK)
            return false;
    }
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA) != RCL_RET_OK)
        return false;
    rmw_uros_sync_session(1000);
    return true;
}

void entities_destroy() {
    rmw_context_t* rmw = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
    if (sim_mode) {
        rcl_publisher_fini(&truth_pub, &node);
        rcl_publisher_fini(&sonar_pub, &node);
        rcl_publisher_fini(&imu_pub, &node);
    }
    rcl_publisher_fini(&odom_pub, &node);
    rcl_publisher_fini(&joint_pub, &node);
    rcl_subscription_fini(&cmd_sub, &node);
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

void publish_sonar() {
    stamp(&sonar_msg.header.stamp);
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
            if (agent == Agent::kConnected) ++sessions;
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
            entities_destroy();
            agent = Agent::kWaiting;
            break;
    }
}

void console_info();

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
        Serial.printf("base sessions %u  loop max %u ms  late ticks %u\n", static_cast<unsigned>(sessions),
                      static_cast<unsigned>(loop_max_ms), static_cast<unsigned>(late_ticks));
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
        // The serial link's budget (docs/firmware.md): ~50 KB/s of messages, about half of
        // 921600 baud. The full set at 25 Hz ground truth was ~65 KB/s and saturated it: rates
        // fell, and on Lyrical the loop sometimes hung (the watchdog in setup() now resets it).
        if (agent == Agent::kConnected) {
            if (frame % 2 == 0) publish_state();  // 25 Hz: joint_states 452 B, odom 732 B
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
                                                                              : "other");
    Serial.printf("app %s  sim %s  board %s\n", prefs.getString("app", "base").c_str(), sim_mode ? "on" : "off",
                  HEXA_BOARD_VERIFIED ? "verified" : "NOT identified (real mode refused)");
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
    } else if (cmd == "bat") {
        Serial.printf("battery adc %d mV  i2c %d mV\n", board_io::battery_adc_mv(), board_io::battery_i2c_mv());
    } else if (cmd == "sonar") {
        Serial.printf("sonar %d mm\n", board_io::sonar_mm());
    } else if (cmd == "imu") {
        Serial.printf("imu who_am_i %d (QMI8658: 5)\n", board_io::imu_who_am_i());
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
    } else if (cmd.length()) {
        Serial.println("commands: info | link | i2c | servo <id> <us> [ms] | bat | sonar | imu | app base|console | sim on|off | reboot");
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

bool console = false;

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
    sim_mode = prefs.getBool("sim", true) || !HEXA_BOARD_VERIFIED;
    link_cfg = hexa_link::load(prefs);
    hexa_link::begin(link_cfg);
    // The loop stops for an update's length; the gait is told to stand first.
    hexa_link::on_ota([]() { robot.command({0, 0, 0}, millis() / 1000.0f); }, nullptr);
    console = boot_button_pressed(2000) || prefs.getString("app", "base") == "console";
    if (console) console_setup();
    else base_setup();
}

void loop() {
    hexa_link::loop();
    if (console) {
        console_loop();
    } else {
        base_loop();
        if (link_cfg.kind == hexa_link::Kind::kWifi) console_loop();  // Serial is free
    }
}
