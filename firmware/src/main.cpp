// miniHexa firmware: one binary, the application chosen by the NVS `app` key.
//
//   base     micro-ROS node: /cmd_vel in; /joint_states and /odom/gait out. The gait
//            ticks at 50 Hz whether or not the agent is there; a lost command stream
//            ends in a stand (hexa_gait's timeout), never a frozen stride.
//   console  line commands on the serial port: info, i2c, app, sim, reboot.
//
// Press BOOT within the first two seconds after a reset to get the console whatever
// `app` says. Sim mode (NVS `sim`, default on) drives simulated servos; real mode is
// refused until board_minihexa.h is verified.
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_chip_info.h>
#include <esp_mac.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#ifdef HEXA_CMD_VEL_STAMPED
#include <geometry_msgs/msg/twist_stamped.h>
#else
#include <geometry_msgs/msg/twist.h>
#endif

#include "board_minihexa.h"
#include "hexa_gait.h"
#include "hexa_kin.h"
#include "hexa_servo.h"

using namespace hexa;

namespace {

Preferences prefs;
bool sim_mode = true;

// ---------------------------------------------------------------- robot model
const float kTickS = 0.02f;  // 50 Hz servo frame
TripodGait gait;
ServoCal cal[kJoints];
SimServo sim_servo[kJoints];
SimServoParams sim_params;
float commanded[kJoints];
uint32_t unreachable = 0;
struct {
    double x = 0, y = 0, yaw = 0;
} gait_pose;
BodyTwist gait_twist{0, 0, 0};

void robot_init() {
    const Geometry& g = minihexa_geometry();
    gait.init(&g, GaitParams());
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        leg_ik(g, leg, gait.foot(leg), &a);
        const float ang[3] = {a.coxa, a.femur, a.tibia};
        for (int j = 0; j < 3; ++j) {
            const int k = leg * 3 + j;
            cal[k] = default_cal(k);  // TODO: trims from NVS (servo_trim tool)
            commanded[k] = ang[j];
            sim_servo[k].reset(cal[k], ang[j]);
        }
    }
}

void robot_tick(float now_s) {
    gait.step(kTickS, now_s);
    gait_twist = gait.applied();
    const double c = cos(gait_pose.yaw), s = sin(gait_pose.yaw);
    gait_pose.x += (c * gait_twist.vx - s * gait_twist.vy) * kTickS;
    gait_pose.y += (s * gait_twist.vx + c * gait_twist.vy) * kTickS;
    gait_pose.yaw = remainder(gait_pose.yaw + gait_twist.wz * kTickS, 2 * M_PI);

    const Geometry& g = minihexa_geometry();
    for (int leg = 0; leg < kLegs; ++leg) {
        LegAngles a;
        if (!leg_ik(g, leg, gait.foot(leg), &a)) {
            ++unreachable;  // keep the leg's last angles
            continue;
        }
        commanded[leg * 3] = a.coxa;
        commanded[leg * 3 + 1] = a.femur;
        commanded[leg * 3 + 2] = a.tibia;
    }
    for (int k = 0; k < kJoints; ++k) {
        const int us = angle_to_us(cal[k], commanded[k]);
        if (sim_mode) {
            sim_servo[k].command_us(us);
            sim_servo[k].step(kTickS, sim_params);
        }
        // Real output: none until the board is identified (board_minihexa.h).
    }
}

// What /joint_states reports: the simulated servo's angle in sim mode; in real mode
// the COMMANDED angle, since a PWM servo reports nothing back.
float joint_angle(int k) { return sim_mode ? sim_servo[k].angle() : commanded[k]; }

// ---------------------------------------------------------------- micro-ROS base
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t cmd_sub;
rcl_publisher_t joint_pub, odom_pub;
#ifdef HEXA_CMD_VEL_STAMPED
geometry_msgs__msg__TwistStamped cmd_msg;
#else
geometry_msgs__msg__Twist cmd_msg;
#endif
sensor_msgs__msg__JointState joint_msg;
nav_msgs__msg__Odometry odom_msg;
rosidl_runtime_c__String joint_names[kJoints];
double joint_pos[kJoints];

enum class Agent { kWaiting, kAvailable, kConnected, kDisconnected } agent = Agent::kWaiting;

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
    gait.command({static_cast<float>(t.linear.x), static_cast<float>(t.linear.y),
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
    set_string(&odom_msg.child_frame_id, "base_link");
    // Dead reckoning from commanded steps: loose, and yaw the loosest (no magnetometer).
    const double pose_var[6] = {0.01, 0.01, 1e6, 1e6, 1e6, 0.05};
    const double twist_var[6] = {0.004, 0.004, 1e6, 1e6, 1e6, 0.02};
    for (int i = 0; i < 6; ++i) {
        odom_msg.pose.covariance[i * 7] = pose_var[i];
        odom_msg.twist.covariance[i * 7] = twist_var[i];
    }
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

void publish_state() {
    stamp(&joint_msg.header.stamp);
    for (int k = 0; k < kJoints; ++k) joint_pos[k] = joint_angle(k);
    rcl_publish(&joint_pub, &joint_msg, nullptr);

    odom_msg.header.stamp = joint_msg.header.stamp;
    odom_msg.pose.pose.position.x = gait_pose.x;
    odom_msg.pose.pose.position.y = gait_pose.y;
    odom_msg.pose.pose.orientation.z = sin(gait_pose.yaw / 2);
    odom_msg.pose.pose.orientation.w = cos(gait_pose.yaw / 2);
    odom_msg.twist.twist.linear.x = gait_twist.vx;
    odom_msg.twist.twist.linear.y = gait_twist.vy;
    odom_msg.twist.twist.angular.z = gait_twist.wz;
    rcl_publish(&odom_pub, &odom_msg, nullptr);
}

void agent_step() {
    static uint32_t last_check = 0;
    const uint32_t now = millis();
    switch (agent) {
        case Agent::kWaiting:
            if (now - last_check > 500) {
                last_check = now;
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) agent = Agent::kAvailable;
            }
            break;
        case Agent::kAvailable:
            agent = entities_create() ? Agent::kConnected : Agent::kWaiting;
            if (agent == Agent::kWaiting) entities_destroy();
            break;
        case Agent::kConnected:
            if (now - last_check > 1000) {
                last_check = now;
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    agent = Agent::kDisconnected;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            break;
        case Agent::kDisconnected:
            entities_destroy();
            agent = Agent::kWaiting;
            break;
    }
}

void base_setup() {
    robot_init();
    msgs_init();
    set_microros_serial_transports(Serial);
}

void base_loop() {
    static uint32_t next_tick = millis();
    static uint8_t frame = 0;
    agent_step();
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - next_tick) >= 0) {
        next_tick += static_cast<uint32_t>(kTickS * 1000);
        if (static_cast<int32_t>(now - next_tick) > 100) next_tick = now;  // fell behind: no burst
        robot_tick(now / 1000.0f);
        if (agent == Agent::kConnected && (++frame & 1) == 0) publish_state();  // 25 Hz
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
    Serial.printf("app %s  sim %s  board %s\n", prefs.getString("app", "base").c_str(), sim_mode ? "on" : "off",
                  HEXA_BOARD_VERIFIED ? "verified" : "NOT identified (real mode refused)");
    Serial.printf("build %s %s\n", __DATE__, __TIME__);
}

void console_i2c() {
    Wire.begin(HEXA_I2C_SDA, HEXA_I2C_SCL);
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
    } else if (cmd == "i2c") {
        console_i2c();
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
        Serial.println("commands: info | i2c | app base|console | sim on|off | reboot");
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
    Serial.begin(HEXA_SERIAL_BAUD);
    prefs.begin("hexa", false);
    sim_mode = prefs.getBool("sim", true) || !HEXA_BOARD_VERIFIED;
    console = boot_button_pressed(2000) || prefs.getString("app", "base") == "console";
    if (console) console_setup();
    else base_setup();
}

void loop() {
    if (console) console_loop();
    else base_loop();
}
