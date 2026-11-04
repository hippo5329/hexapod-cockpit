// The host simulator as a ROS 2 node: stands in for the firmware's base application.
//
// The same topics, rates and QoS as the firmware's sim mode, from the same models
// (firmware/lib: hexa_world, hexa_sensors), stepped at the firmware's 50 Hz frame:
//   /cmd_vel           in: Twist on Jazzy, TwistStamped on Lyrical, as the firmware is built
//   /joint_states      25 Hz, the simulated servos' angles
//   /odom/gait         25 Hz, gait odometry
//   /sim/ground_truth   5 Hz, the body's true pose from the grounded feet (with slip)
//   /imu/data          50 Hz, simulated QMI8658A: accel + gyro, no orientation (no AHRS yet)
//   /sonar             10 Hz, simulated ultrasonic, cone raycast against the room's walls
// all best effort; plus /sim/walls (latched markers of the room) for RViz.
//
// Parameters:
//   slip             fraction of stance motion lost to the floor (default 0.05)
//   room_half_size   the square room around the start pose, m (default 1.0: 2 m x 2 m)
//   seed             sensor noise seed (default 5329)
//   cmd_vel_stamped  TwistStamped on /cmd_vel (default: true unless ROS_DISTRO is jazzy)
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "hexa_sensors.h"
#include "hexa_world.h"

using namespace std::chrono_literals;

namespace {

constexpr float kTickS = 0.02f;  // the firmware's 50 Hz servo frame

bool default_stamped() {
    const char* d = std::getenv("ROS_DISTRO");
    return !(d && std::strcmp(d, "jazzy") == 0);
}

void set_pose(nav_msgs::msg::Odometry& m, const hexa::Pose2& p) {
    m.pose.pose.position.x = p.x;
    m.pose.pose.position.y = p.y;
    m.pose.pose.position.z = 0;
    m.pose.pose.orientation.x = 0;
    m.pose.pose.orientation.y = 0;
    m.pose.pose.orientation.z = std::sin(p.yaw / 2);
    m.pose.pose.orientation.w = std::cos(p.yaw / 2);
}

class HexapodSim : public rclcpp::Node {
public:
    HexapodSim() : Node("hexapod_sim") {
        const double slip = declare_parameter("slip", 0.05);
        const bool stamped = declare_parameter("cmd_vel_stamped", default_stamped());
        const double room = declare_parameter("room_half_size", 1.0);
        const int seed = declare_parameter("seed", 5329);
        if (!robot_.init(slip)) throw std::runtime_error("neutral stance unreachable");
        imu_.init(hexa::ImuParams(), static_cast<uint32_t>(seed));
        sonar_.init(hexa::RangeParams(), static_cast<uint32_t>(seed) * 2654435761u);
        hexa::default_room(walls_, static_cast<float>(room));

        joint_msg_.name.assign(hexa::kJointNames, hexa::kJointNames + hexa::kJoints);
        joint_msg_.position.resize(hexa::kJoints);
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_footprint";
        // Same covariances as the firmware: dead reckoning, yaw the loosest.
        const double pose_var[6] = {0.01, 0.01, 1e6, 1e6, 1e6, 0.05};
        const double twist_var[6] = {0.004, 0.004, 1e6, 1e6, 1e6, 0.02};
        for (int i = 0; i < 6; ++i) {
            odom_msg_.pose.covariance[i * 7] = pose_var[i];
            odom_msg_.twist.covariance[i * 7] = twist_var[i];
        }
        truth_msg_.header.frame_id = "odom";
        truth_msg_.child_frame_id = "base_footprint";
        // The simulated IMU declares the white noise it makes; no orientation (REP 145: -1).
        imu_msg_.header.frame_id = "imu_link";
        imu_msg_.orientation_covariance[0] = -1;
        for (int i = 0; i < 3; ++i) {
            imu_msg_.angular_velocity_covariance[i * 4] = hexa::uniform_var(imu_.params().gyro_noise);
            imu_msg_.linear_acceleration_covariance[i * 4] = hexa::uniform_var(imu_.params().accel_noise);
        }
        sonar_msg_.header.frame_id = "sonar_link";
        sonar_msg_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        sonar_msg_.field_of_view = sonar_.params().fov;
        sonar_msg_.min_range = sonar_.params().min_range;
        sonar_msg_.max_range = sonar_.params().max_range;

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/gait", rclcpp::SensorDataQoS());
        truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("sim/ground_truth", rclcpp::SensorDataQoS());
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
        sonar_pub_ = create_publisher<sensor_msgs::msg::Range>("sonar", rclcpp::SensorDataQoS());
        walls_pub_ = create_publisher<visualization_msgs::msg::Marker>("sim/walls",
                                                                     rclcpp::QoS(1).transient_local());
        publish_walls();
        if (stamped)
            stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                "cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { on_cmd(m->twist); });
        else
            twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { on_cmd(*m); });

        start_ = std::chrono::steady_clock::now();
        timer_ = create_wall_timer(20ms, [this] { tick(); });
        RCLCPP_INFO(get_logger(), "miniHexa sim: slip %.2f, room %.1f x %.1f m, /cmd_vel %s", slip, 2 * room, 2 * room,
                    stamped ? "TwistStamped" : "Twist");
    }

private:
    float now_s() const {
        return std::chrono::duration<float>(std::chrono::steady_clock::now() - start_).count();
    }

    void on_cmd(const geometry_msgs::msg::Twist& t) {
        robot_.command({static_cast<float>(t.linear.x), static_cast<float>(t.linear.y),
                        static_cast<float>(t.angular.z)},
                       now_s());
    }

    void tick() {
        const hexa::StepCounts n = robot_.step(kTickS, now_s());
        if (n.unreachable || n.clamped)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "gait fault: %d legs unreachable, %d pulses clamped",
                                 n.unreachable, n.clamped);
        const hexa::ImuSample m = imu_.step(kTickS, robot_.truth_twist());
        ++frame_;
        if (frame_ % 2 == 0) publish();  // 25 Hz, as the firmware
        publish_imu(m);                  // 50 Hz
        if (frame_ % 5 == 0) publish_sonar();  // 10 Hz
        if (frame_ % 10 == 1) publish_truth();  //  5 Hz
    }

    void publish_imu(const hexa::ImuSample& m) {
        imu_msg_.header.stamp = now();
        imu_msg_.linear_acceleration.x = m.ax;
        imu_msg_.linear_acceleration.y = m.ay;
        imu_msg_.linear_acceleration.z = m.az;
        imu_msg_.angular_velocity.x = m.gx;
        imu_msg_.angular_velocity.y = m.gy;
        imu_msg_.angular_velocity.z = m.gz;
        imu_pub_->publish(imu_msg_);
    }

    void publish_sonar() {
        sonar_msg_.header.stamp = now();
        sonar_msg_.range = sonar_.measure(robot_.truth(), walls_, hexa::kRoomWalls);
        sonar_pub_->publish(sonar_msg_);
    }

    void publish_walls() {
        visualization_msgs::msg::Marker w;
        w.header.frame_id = "odom";
        w.ns = "sim_walls";
        w.type = visualization_msgs::msg::Marker::LINE_LIST;
        w.scale.x = 0.01;
        w.color.r = 0.85f;
        w.color.g = 0.85f;
        w.color.b = 0.85f;
        w.color.a = 1.0f;
        w.pose.orientation.w = 1.0;
        for (const hexa::Wall& s : walls_) {
            geometry_msgs::msg::Point a, b;
            a.x = s.x1; a.y = s.y1; a.z = 0.02;
            b.x = s.x2; b.y = s.y2; b.z = 0.02;
            w.points.push_back(a);
            w.points.push_back(b);
        }
        walls_pub_->publish(w);
    }

    void publish() {
        const rclcpp::Time stamp = now();
        joint_msg_.header.stamp = stamp;
        for (int k = 0; k < hexa::kJoints; ++k) joint_msg_.position[k] = robot_.joint_angle(k);
        joint_pub_->publish(joint_msg_);

        odom_msg_.header.stamp = stamp;
        set_pose(odom_msg_, robot_.odom());
        const hexa::BodyTwist tw = robot_.odom_twist();
        odom_msg_.twist.twist.linear.x = tw.vx;
        odom_msg_.twist.twist.linear.y = tw.vy;
        odom_msg_.twist.twist.angular.z = tw.wz;
        odom_pub_->publish(odom_msg_);
    }

    // Ground truth: the true pose, and the true body twist over the last step. 5 Hz, as the
    // firmware (whose serial link has no room for more).
    void publish_truth() {
        const hexa::BodyTwist& tt = robot_.truth_twist();
        truth_msg_.header.stamp = now();
        set_pose(truth_msg_, robot_.truth());
        truth_msg_.twist.twist.linear.x = tt.vx;
        truth_msg_.twist.twist.linear.y = tt.vy;
        truth_msg_.twist.twist.angular.z = tt.wz;
        truth_pub_->publish(truth_msg_);
    }

    hexa::SimRobot robot_;
    hexa::SimImu imu_;
    hexa::SimRange sonar_;
    hexa::Wall walls_[hexa::kRoomWalls];
    std::chrono::steady_clock::time_point start_;
    unsigned frame_ = 0;
    sensor_msgs::msg::JointState joint_msg_;
    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::Range sonar_msg_;
    nav_msgs::msg::Odometry odom_msg_, truth_msg_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, truth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr walls_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodSim>());
    rclcpp::shutdown();
    return 0;
}
