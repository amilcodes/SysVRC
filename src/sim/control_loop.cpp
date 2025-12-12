// sim/src/visbot_controller_node.cpp
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class VisBotController : public rclcpp::Node {
public:
    VisBotController()
        : Node("visbot_controller"),
          last_tick_(this->now()),
          target_period_(8.333ms),   // 120 Hz
          max_latency_(4ms)          // we allow ~4 ms jitter over the ideal period
    {
        // Publishers
        cmd_drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/visbot/cmd_drive", 10);

        // Subscriptions (wire these to your Gazebo plugins)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/visbot/imu", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                last_imu_ = *msg;
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/visbot/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                last_odom_ = *msg;
            });

        // 120 Hz control timer
        control_timer_ = this->create_wall_timer(
            target_period_,
            std::bind(&VisBotController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
                    "VisBot controller started at 120 Hz (period %.3f ms)",
                    std::chrono::duration<double, std::milli>(target_period_).count());
    }

private:
    void controlLoop() {
        rclcpp::Time now = this->now();
        auto dt = now - last_tick_;
        last_tick_ = now;

        // Check for latency spikes relative to ideal 120 Hz period
        auto dt_ms = std::chrono::duration<double, std::milli>(dt.to_chrono());
        auto ideal_ms = std::chrono::duration<double, std::milli>(target_period_);
        auto overrun = dt_ms - ideal_ms;

        if (overrun > max_latency_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Control loop latency spike: dt = %.3f ms (ideal %.3f ms, overrun %.3f ms)",
                dt_ms.count(), ideal_ms.count(), overrun.count());
        }

        // --- Read latest state ------------------------------------------------
        // In a real implementation you’d also guard against stale messages.
        const auto &odom = last_odom_;
        const auto &imu  = last_imu_;

        // Pose / heading from odom
        double x = odom.pose.pose.position.x;
        double y = odom.pose.pose.position.y;
        double yaw = imuToYaw(imu);

        // --- Control logic mirroring PROS routines ---------------------------
        // This is where you mirror things like:
        // - drive PID to a target (x*, y*, yaw*)
        // - sequences similar to worldsMogoRush / skills, but in sim
        //
        // For brevity we just do a simple "hold heading, drive forward" demo.

        geometry_msgs::msg::Twist cmd;
        double target_yaw = 0.0;  // hold field-forward heading

        double yaw_error = normalizeAngle(target_yaw - yaw);

        // Very simple P controller; in a real implementation use the same
        // PID gains you’ve tuned on the V5 / EZ-Template side.
        double kP_lin = 0.8;
        double kP_ang = 1.5;

        double v = kP_lin * 1.0;             // forward velocity command
        double w = kP_ang * yaw_error;       // angular velocity command

        // Saturate commands to be safe
        v = std::clamp(v, -1.0, 1.0);
        w = std::clamp(w, -5.0, 5.0);

        cmd.linear.x  = v;
        cmd.angular.z = w;

        cmd_drive_pub_->publish(cmd);
    }

    static double imuToYaw(const sensor_msgs::msg::Imu &imu) {
        // Convert quaternion to yaw. This is minimal and assumes normalized quaternion.
        const auto &q = imu.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static double normalizeAngle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    nav_msgs::msg::Odometry last_odom_{};
    sensor_msgs::msg::Imu  last_imu_{};

    rclcpp::Time last_tick_;
    std::chrono::milliseconds target_period_;
    std::chrono::milliseconds max_latency_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisBotController>();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
