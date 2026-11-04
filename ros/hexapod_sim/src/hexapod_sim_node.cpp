// The host simulator as a ROS 2 node: stands in for the firmware's base application.
//
// /cmd_vel in (Twist on Jazzy, TwistStamped on Lyrical, as the firmware is built);
// /joint_states and /odom/gait out at 25 Hz, best effort, like the firmware; plus
// /sim/ground_truth, the body's true pose from the grounded feet (with slip), which no
// real robot has. The world is sim/hexa_world: the firmware's own gait, IK and servo
// mapping driving simulated servos, stepped at the firmware's 50 Hz frame.
//
// Parameters:
//   slip             fraction of stance motion lost to the floor (default 0.05)
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
#include <sensor_msgs/msg/joint_state.hpp>

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
        if (!robot_.init(slip)) throw std::runtime_error("neutral stance unreachable");

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

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/gait", rclcpp::SensorDataQoS());
        truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("sim/ground_truth", 10);
        if (stamped)
            stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
                "cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { on_cmd(m->twist); });
        else
            twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { on_cmd(*m); });

        start_ = std::chrono::steady_clock::now();
        timer_ = create_wall_timer(20ms, [this] { tick(); });
        RCLCPP_INFO(get_logger(), "miniHexa sim: slip %.2f, /cmd_vel %s", slip, stamped ? "TwistStamped" : "Twist");
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
        if ((++frame_ & 1) == 0) publish();  // 25 Hz, as the firmware
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

        // Ground truth twist: the true pose change since the last publish, in the body frame.
        const hexa::Pose2& p = robot_.truth();
        const double dt = 2 * kTickS;
        const double dx = p.x - truth_prev_.x, dy = p.y - truth_prev_.y;
        const double c = std::cos(truth_prev_.yaw), s = std::sin(truth_prev_.yaw);
        truth_msg_.header.stamp = stamp;
        set_pose(truth_msg_, p);
        truth_msg_.twist.twist.linear.x = (c * dx + s * dy) / dt;
        truth_msg_.twist.twist.linear.y = (-s * dx + c * dy) / dt;
        truth_msg_.twist.twist.angular.z = std::remainder(p.yaw - truth_prev_.yaw, 2 * M_PI) / dt;
        truth_pub_->publish(truth_msg_);
        truth_prev_ = p;
    }

    hexa::SimRobot robot_;
    hexa::Pose2 truth_prev_;
    std::chrono::steady_clock::time_point start_;
    unsigned frame_ = 0;
    sensor_msgs::msg::JointState joint_msg_;
    nav_msgs::msg::Odometry odom_msg_, truth_msg_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, truth_pub_;
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
