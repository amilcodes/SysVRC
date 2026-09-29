// plant_node — the fast deterministic backend. Publishes the *same* topics
// Gazebo does (/visbot/imu, /visbot/joint_states, /visbot/odom) from the
// kinematic visbot::DiffDrivePlant, stepped at 1 kHz on its own thread.
//
// Sensor cadence and transport delay are parameters so the controller can be
// exercised against V5-like timing (10 ms sensors, a few ms of bus latency)
// without a physics engine in the loop.
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visbot/auton_file.hpp"
#include "visbot/visbot.hpp"
#include "visbot_control/latest_value.hpp"
#include "visbot_control/rt_loop.hpp"

namespace vc = visbot_control;

namespace {
struct TwistPod { double linear = 0, angular = 0; };
}

class PlantNode : public rclcpp::Node {
public:
    PlantNode() : Node("visbot_plant") {
        physicsHz_ = declare_parameter("physics_hz", 1000.0);
        imuHz_ = declare_parameter("imu_hz", 200.0);
        jointHz_ = declare_parameter("joint_state_hz", 100.0);
        odomHz_ = declare_parameter("odom_hz", 50.0);
        latencyMs_ = declare_parameter("sensor_latency_ms", 3.0);
        rp_.trackWidthIn = declare_parameter("track_width_in", rp_.trackWidthIn);
        rp_.wheelDiameterIn = declare_parameter("wheel_diameter_in", rp_.wheelDiameterIn);
        rp_.wheelRpm = declare_parameter("wheel_rpm", rp_.wheelRpm);
        pp_.motorTauSec = declare_parameter("motor_tau_s", pp_.motorTauSec);
        pp_.imuNoiseStdDeg = declare_parameter("imu_noise_std_deg", pp_.imuNoiseStdDeg);
        pp_.imuDriftDegPerS = declare_parameter("imu_drift_deg_per_s", pp_.imuDriftDegPerS);
        pp_.slipFraction = declare_parameter("slip_fraction", pp_.slipFraction);
        visbot::Pose start;
        start.x = declare_parameter("start_x_in", 0.0);
        start.y = declare_parameter("start_y_in", 0.0);
        start.theta = declare_parameter("start_theta_deg", 0.0);
        // Start where the routine physically starts (its @field_start, or the
        // pose odom_xyt_set declares), unless start_* was given explicitly.
        const std::string mission = declare_parameter("mission", std::string(""));
        const char* env = std::getenv("SYSVRC_AUTONS");
        const std::string autonsDir = declare_parameter("autons_dir", std::string(env && *env ? env : "/ws/src/sysvrc/autons"));
        if (!mission.empty() && start.x == 0.0 && start.y == 0.0 && start.theta == 0.0) {
            const visbot::AutonParseResult r = visbot::resolveMission(mission, autonsDir);
            if (r.ok()) {
                start = visbot::physicalStart(r.def);
                pp_.walls = r.def.hasFieldStart;
            }
        }

        plant_ = std::make_unique<visbot::DiffDrivePlant>(rp_, pp_);
        plant_->reset(start);

        cmdSub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/visbot/cmd_vel", 10, [this](const geometry_msgs::msg::Twist& t) { cmd_.write({t.linear.x, t.angular.z}); });
        imuPub_ = create_publisher<sensor_msgs::msg::Imu>("/visbot/imu", rclcpp::SensorDataQoS());
        jointPub_ = create_publisher<sensor_msgs::msg::JointState>("/visbot/joint_states", rclcpp::SensorDataQoS());
        odomPub_ = create_publisher<nav_msgs::msg::Odometry>("/visbot/odom", 10);

        // The plant runs on a normal-priority thread: it is the "world", not
        // the thing under test. (Under Gazebo the physics thread plays this role.)
        vc::RtConfig cfg;
        cfg.rateHz = physicsHz_;
        cfg.schedPolicy = SCHED_OTHER;
        cfg.lockMemory = false;
        loop_ = std::make_unique<vc::RtLoop>(cfg);
        loop_->start([this](const vc::TickContext& c) { step(c); });
        RCLCPP_INFO(get_logger(), "plant backend: physics %.0f Hz, imu %.0f Hz, joints %.0f Hz, latency %.1f ms",
                    physicsHz_, imuHz_, jointHz_, latencyMs_);
    }
    ~PlantNode() override { loop_->stop(); }

private:
    struct Sample { visbot::SensorSnapshot s; visbot::Pose truth; double vl, vr; int64_t tNs; };

    void step(const vc::TickContext& c) {
        const TwistPod t = cmd_.read().value;
        // REP-103 twist -> V5 wheel command
        const double vmax = rp_.maxWheelSpeedInPerSec();
        const double v = visbot::m2in(t.linear);
        const double wCw = -t.angular;                       // CCW yaw rate -> CW compass rate
        const double vl = v + wCw * rp_.trackWidthIn / 2.0;
        const double vr = v - wCw * rp_.trackWidthIn / 2.0;
        plant_->step({vl / vmax * 127.0, vr / vmax * 127.0}, c.dtSeconds);

        // Delay line so sensors arrive `latencyMs_` after the physics that produced them.
        std::lock_guard<std::mutex> lk(m_);
        delay_.push_back({plant_->sensors(), plant_->truth(), plant_->leftSpeed(), plant_->rightSpeed(), c.wakeNs});
        const int64_t latNs = int64_t(latencyMs_ * 1e6);
        while (delay_.size() > 1 && c.wakeNs - delay_[1].tNs >= latNs) delay_.pop_front();
        const Sample& s = delay_.front();
        if (c.wakeNs - s.tNs < latNs) return;

        if (due(nextImu_, imuHz_, c.wakeNs)) publishImu(s);
        if (due(nextJoint_, jointHz_, c.wakeNs)) publishJoints(s);
        if (due(nextOdom_, odomHz_, c.wakeNs)) publishOdom(s);
    }

    static bool due(int64_t& next, double hz, int64_t now) {
        if (now < next) return false;
        next = (next == 0 ? now : next) + int64_t(1e9 / hz);
        if (next < now) next = now;
        return true;
    }

    void publishImu(const Sample& s) {
        sensor_msgs::msg::Imu m;
        m.header.stamp = now();
        m.header.frame_id = "imu_link";
        const double yaw = visbot::headingToYaw(s.s.imuHeadingDeg);
        m.orientation.z = std::sin(yaw / 2);
        m.orientation.w = std::cos(yaw / 2);
        m.angular_velocity.z = -(s.vl - s.vr) / rp_.trackWidthIn;
        imuPub_->publish(m);
    }
    void publishJoints(const Sample& s) {
        sensor_msgs::msg::JointState m;
        m.header.stamp = now();
        m.name = {"left_wheel_joint", "right_wheel_joint"};
        m.position = {visbot::deg2rad(s.s.enc.leftDeg), visbot::deg2rad(s.s.enc.rightDeg)};
        const double r = rp_.wheelDiameterIn / 2.0;
        m.velocity = {s.vl / r, s.vr / r};
        jointPub_->publish(m);
    }
    void publishOdom(const Sample& s) {
        nav_msgs::msg::Odometry m;
        m.header.stamp = now();
        m.header.frame_id = "field";
        m.child_frame_id = "base_link";
        m.pose.pose.position.x = visbot::in2m(s.truth.x);
        m.pose.pose.position.y = visbot::in2m(s.truth.y);
        const double yaw = visbot::headingToYaw(s.truth.theta);
        m.pose.pose.orientation.z = std::sin(yaw / 2);
        m.pose.pose.orientation.w = std::cos(yaw / 2);
        m.twist.twist.linear.x = visbot::in2m(0.5 * (s.vl + s.vr));
        m.twist.twist.angular.z = -(s.vl - s.vr) / rp_.trackWidthIn;
        odomPub_->publish(m);
    }

    double physicsHz_, imuHz_, jointHz_, odomHz_, latencyMs_;
    visbot::RobotParams rp_;
    visbot::PlantParams pp_;
    std::unique_ptr<visbot::DiffDrivePlant> plant_;
    vc::LatestValue<TwistPod> cmd_;
    std::unique_ptr<vc::RtLoop> loop_;
    std::mutex m_;
    std::deque<Sample> delay_;
    int64_t nextImu_ = 0, nextJoint_ = 0, nextOdom_ = 0;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlantNode>());
    rclcpp::shutdown();
    return 0;
}
