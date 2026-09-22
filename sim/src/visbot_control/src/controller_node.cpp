// controller_node — runs the shared visbot::MotionController inside a
// bounded-latency 120 Hz RT thread and bridges it to ROS 2 topics.
//
//   sensors  (executor thread)  --LatestValue-->  RT tick  --RealtimeBox-->  publisher thread
//
// The RT tick never touches rclcpp: it reads two lock-free mailboxes, runs
// the identical controller code the V5 brain runs, and drops its outputs in
// a mailbox for a non-RT thread to publish.
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cstdio>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visbot/visbot.hpp"
#include "visbot_control/latest_value.hpp"
#include "visbot_control/rt_loop.hpp"
#include "visbot_msgs/msg/control_state.hpp"
#include "visbot_msgs/msg/control_stats.hpp"

using namespace std::chrono_literals;
namespace vc = visbot_control;

namespace {

// Plain-old-data copies of the sensor messages: trivially copyable so they
// can live in a SeqLock.
struct ImuSample { double yawRad = 0.0; double stampSec = 0.0; };
struct WheelSample { double leftRad = 0.0, rightRad = 0.0; double stampSec = 0.0; };

struct CmdOut {
    double left = 0, right = 0;      // V5 units
    double linear = 0, angular = 0;  // m/s, rad/s
    uint64_t seq = 0;
};

struct StateSnapshot {
    visbot::Pose pose;
    visbot::MotionStatus status;
    visbot::SensorSnapshot sensors;
    double sensorAgeUs = 0;
    CmdOut cmd;
    uint64_t staleTicks = 0;
    double sensorAgeSumUs = 0;
    uint64_t sensorAgeCount = 0;
};

double yawFromQuat(double x, double y, double z, double w) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/// Render one instruction the way it reads in autons.cpp.
std::string instrLabel(const visbot::Instr& in) {
    char buf[72];
    using Op = visbot::Instr::Op;
    switch (in.op) {
        case Op::DriveSet:
            std::snprintf(buf, sizeof buf, "drive %+.0f in @%.0f%s", in.a, in.speed, in.slew ? "" : " noslew");
            break;
        case Op::TurnSet:
            std::snprintf(buf, sizeof buf, "turn to %.0f deg @%.0f", in.a, in.speed);
            break;
        case Op::SwingSet:
            std::snprintf(buf, sizeof buf, "swing %s to %.0f deg @%.0f",
                          in.side == visbot::SwingSide::Left ? "L" : "R", in.a, in.speed);
            break;
        case Op::OdomSet:
            std::snprintf(buf, sizeof buf, "odom %s(%.0f, %.0f) @%.0f",
                          in.dir == visbot::DriveDirection::Reverse ? "rev " : "", in.a, in.b, in.speed);
            break;
        case Op::Wait:               std::snprintf(buf, sizeof buf, "wait"); break;
        case Op::WaitUntil:          std::snprintf(buf, sizeof buf, "wait_until %+.1f", in.a); break;
        case Op::WaitQuickChain:     std::snprintf(buf, sizeof buf, "wait_quick_chain"); break;
        case Op::Delay:              std::snprintf(buf, sizeof buf, "delay %.0f ms", in.a); break;
        case Op::SpeedMax:           std::snprintf(buf, sizeof buf, "speed_max %.0f", in.speed); break;
        case Op::DriveChainConstant: std::snprintf(buf, sizeof buf, "chain_const %.0f in", in.a); break;
        case Op::Action:             std::snprintf(buf, sizeof buf, "> %s", visbot::toString(in.action)); break;
    }
    return buf;
}

}  // namespace

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("visbot_controller") {
        // ---- parameters -------------------------------------------------------
        cfg_.rateHz = declare_parameter("rate_hz", 120.0);
        cfg_.schedPolicy = vc::RtLoop::policyFromName(declare_parameter("sched_policy", std::string("fifo")));
        cfg_.priority = declare_parameter("priority", 80);
        cfg_.cpu = declare_parameter("cpu", -1);
        cfg_.lockMemory = declare_parameter("lock_memory", true);
        cfg_.pollIdleCompanion = declare_parameter("poll_idle", false);
        cfg_.spinBeforeDeadlineNs = int64_t(declare_parameter("spin_before_deadline_us", 0.0) * 1000.0);
        staleLimitNs_ = int64_t(declare_parameter("stale_limit_ms", 50.0) * 1e6);
        missionName_ = declare_parameter("mission", std::string("skills"));
        leftJoint_ = declare_parameter("left_wheel_joint", std::string("left_wheel_joint"));
        rightJoint_ = declare_parameter("right_wheel_joint", std::string("right_wheel_joint"));
        params_.trackWidthIn = declare_parameter("track_width_in", params_.trackWidthIn);
        params_.wheelDiameterIn = declare_parameter("wheel_diameter_in", params_.wheelDiameterIn);
        params_.wheelRpm = declare_parameter("wheel_rpm", params_.wheelRpm);
        startPose_.x = declare_parameter("start_x_in", 0.0);
        startPose_.y = declare_parameter("start_y_in", 0.0);
        startPose_.theta = declare_parameter("start_theta_deg", 0.0);
        allianceBlue_ = declare_parameter("alliance_blue", true);
        // Only override the mission's own start pose if the caller asked.
        startPoseOverridden_ = startPose_.x != 0.0 || startPose_.y != 0.0 || startPose_.theta != 0.0;
        const double startDelay = declare_parameter("start_delay_s", 2.0);
        loopMission_ = declare_parameter("loop_mission", false);

        ctrl_ = std::make_unique<visbot::MotionController>(params_);

        // ---- ROS I/O (all on the executor thread) ------------------------------
        imuSub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/visbot/imu", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu& m) {
                const auto& q = m.orientation;
                imu_.write({yawFromQuat(q.x, q.y, q.z, q.w), rclcpp::Time(m.header.stamp).seconds()});
            });
        jointSub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/visbot/joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState& m) {
                WheelSample w;
                bool l = false, r = false;
                for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
                    if (m.name[i] == leftJoint_)  { w.leftRad = m.position[i]; l = true; }
                    if (m.name[i] == rightJoint_) { w.rightRad = m.position[i]; r = true; }
                }
                if (l && r) { w.stampSec = rclcpp::Time(m.header.stamp).seconds(); wheels_.write(w); }
            });

        cmdPub_ = create_publisher<geometry_msgs::msg::Twist>("/visbot/cmd_vel", 10);
        statsPub_ = create_publisher<visbot_msgs::msg::ControlStats>("/visbot/control_stats", 10);
        statePub_ = create_publisher<visbot_msgs::msg::ControlState>("/visbot/control_state", 10);
        posePub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/visbot/pose_estimate", 10);
        pathPub_ = create_publisher<nav_msgs::msg::Path>("/visbot/path_estimate", rclcpp::QoS(1).transient_local());
        path_.header.frame_id = "field";

        statsTimer_ = create_wall_timer(100ms, [this] { publishStats(); });
        stateTimer_ = create_wall_timer(33ms, [this] { publishState(); });
        startTimer_ = create_wall_timer(std::chrono::duration<double>(startDelay), [this] {
            startTimer_->cancel();
            armMission();
        });

        // Publisher thread: wakes when the RT tick drops a new command.
        pubThread_ = std::thread([this] { publisherLoop(); });

        // ---- the RT loop ---------------------------------------------------------
        loop_ = std::make_unique<vc::RtLoop>(cfg_);
        const std::string note = loop_->start([this](const vc::TickContext& c) { tick(c); });
        const auto st = loop_->stats();
        RCLCPP_INFO(get_logger(), "RT loop up: %.0f Hz, %s prio %d, cpu %d, mlock=%s%s%s",
                    cfg_.rateHz, vc::RtLoop::policyName(st.effectivePolicy), st.effectivePriority,
                    st.effectiveCpu, st.memoryLocked ? "yes" : "no", note.empty() ? "" : " — ", note.c_str());
    }

    ~ControllerNode() override {
        loop_->stop();
        pubRunning_ = false;
        pubCv_.notify_all();
        if (pubThread_.joinable()) pubThread_.join();
    }

private:
    // Called from the executor thread once sensors are flowing.
    void armMission() {
        const auto imu = imu_.read();
        const auto wh = wheels_.read();
        if (imu.seq == 0 || wh.seq == 0) {
            RCLCPP_WARN(get_logger(), "waiting for /visbot/imu and /visbot/joint_states before starting mission");
            startTimer_ = create_wall_timer(500ms, [this] { startTimer_->cancel(); armMission(); });
            return;
        }
        const visbot::MissionDef def = visbot::missions::byName(missionName_, allianceBlue_);
        PendingArm a;
        // captured before `a` is moved into the mailbox below

        // A routine's start pose is part of the routine (odom_xyt_set at the
        // top of every auton), so take it from the mission unless the launch
        // file overrode it.
        a.pose = startPoseOverridden_ ? startPose_ : def.start;
        a.mission = def.mission;
        stepLabels_.clear();
        stepLabels_.reserve(a.mission.size());
        for (const auto& in : a.mission) stepLabels_.push_back(instrLabel(in));
        const size_t instrCount = a.mission.size();
        const visbot::Pose armedPose = a.pose;
        {
            std::lock_guard<std::mutex> lk(armMutex_);
            pendingArm_ = std::move(a);
        }
        armRequested_.store(true, std::memory_order_release);
        // Measure the loop over the run, not over process start-up: the first
        // ticks include lazy allocation and first-touch page faults that say
        // nothing about steady-state scheduling.
        loop_->resetStats();
        RCLCPP_INFO(get_logger(), "mission '%s' armed (%zu instructions) from (%.1f, %.1f, %.0f deg)",
                    def.name.c_str(), instrCount, armedPose.x, armedPose.y, armedPose.theta);
    }

    // ======================= RT thread: the 120 Hz tick =======================
    void tick(const vc::TickContext& c) {
        const auto imu = imu_.read();
        const auto wh = wheels_.read();

        // Unwrap wheel angles in case the backend reports them in (-pi, pi].
        unwrap(wh.value.leftRad, prevLeftRad_, leftTurns_);
        unwrap(wh.value.rightRad, prevRightRad_, rightTurns_);
        visbot::SensorSnapshot s;
        s.enc.leftDeg = visbot::rad2deg(wh.value.leftRad + leftTurns_ * 2 * visbot::kPi);
        s.enc.rightDeg = visbot::rad2deg(wh.value.rightRad + rightTurns_ * 2 * visbot::kPi);
        s.imuHeadingDeg = visbot::yawToHeading(imu.value.yawRad);

        const int64_t age = (imu.seq && wh.seq) ? std::max(c.wakeNs - imu.rxNs, c.wakeNs - wh.rxNs) : INT64_MAX;
        const bool stale = age > staleLimitNs_;

        // Mission (re)arm request from the executor thread: take it under a
        // try-lock so the RT thread never blocks on the non-RT side.
        if (armRequested_.load(std::memory_order_acquire) && armMutex_.try_lock()) {
            ctrl_->resetPose(pendingArm_.pose, s);
            ctrl_->setMission(pendingArm_.mission);
            path_.poses.clear();
            armRequested_.store(false, std::memory_order_release);
            armMutex_.unlock();
        }

        visbot::WheelCmd wc;
        if (stale) {
            ++state_.staleTicks;        // hold zero output; the controller keeps its state
        } else {
            wc = ctrl_->tick(s, c.dtSeconds);
            if (loopMission_ && ctrl_->status().done)
                ctrl_->setMission(visbot::missions::byName(missionName_, allianceBlue_).mission);
        }

        // V5 units -> body twist (REP-103) for the diff-drive plugin.
        const double vmax = params_.maxWheelSpeedInPerSec();
        const double vl = wc.left / 127.0 * vmax, vr = wc.right / 127.0 * vmax;
        CmdOut out;
        out.left = wc.left; out.right = wc.right;
        out.linear = visbot::in2m(0.5 * (vl + vr));
        out.angular = -(vl - vr) / params_.trackWidthIn;   // CW(+) compass -> CCW(+) yaw
        out.seq = c.seq;
        if (cmdBox_.tryStore(out)) pubCv_.notify_one();

        // Telemetry snapshot for the 30 Hz state publisher.
        state_.pose = ctrl_->pose();
        state_.status = ctrl_->status();
        state_.sensors = s;
        state_.sensorAgeUs = age == INT64_MAX ? -1.0 : double(age) * 1e-3;
        if (age != INT64_MAX) { state_.sensorAgeSumUs += double(age) * 1e-3; ++state_.sensorAgeCount; }
        state_.cmd = out;
        stateLock_.write(state_);
    }

    static void unwrap(double now, double& prev, long& turns) {
        const double d = now - prev;
        if (d > visbot::kPi) --turns;
        else if (d < -visbot::kPi) ++turns;
        prev = now;
    }

    // ======================= non-RT side =======================
    void publisherLoop() {
        pthread_setname_np(pthread_self(), "visbot_pub");
        geometry_msgs::msg::Twist tw;
        CmdOut out;
        std::unique_lock<std::mutex> lk(pubMutex_);
        while (pubRunning_) {
            pubCv_.wait_for(lk, 5ms);
            while (cmdBox_.take(out)) {
                tw.linear.x = out.linear;
                tw.angular.z = out.angular;
                cmdPub_->publish(tw);
            }
        }
    }

    void publishStats() {
        const vc::RtStats st = loop_->stats();
        const StateSnapshot ss = stateLock_.read();
        visbot_msgs::msg::ControlStats m;
        m.header.stamp = now();
        m.rate_hz = cfg_.rateHz;
        m.period_us = 1e6 / cfg_.rateHz;
        m.sched_policy = vc::RtLoop::policyName(st.effectivePolicy);
        m.priority = st.effectivePriority;
        m.cpu = st.effectiveCpu;
        m.memory_locked = st.memoryLocked;
        m.ticks = st.ticks;
        m.overruns = st.overruns;
        m.missed_deadlines = st.missedDeadlines;
        m.stale_sensor_ticks = ss.staleTicks;
        m.wake_latency_mean_us = st.wake.mean() * 1e-3;
        m.wake_latency_p50_us = st.wake.percentile(0.50) * 1e-3;
        m.wake_latency_p99_us = st.wake.percentile(0.99) * 1e-3;
        m.wake_latency_max_us = st.wake.max * 1e-3;
        m.jitter_rms_us = st.period.rms() * 1e-3;
        m.exec_mean_us = st.exec.mean() * 1e-3;
        m.exec_p50_us = st.exec.percentile(0.50) * 1e-3;
        m.exec_p99_us = st.exec.percentile(0.99) * 1e-3;
        m.exec_max_us = st.exec.max * 1e-3;
        m.sensor_age_mean_us = ss.sensorAgeCount ? ss.sensorAgeSumUs / double(ss.sensorAgeCount) : -1.0;
        m.window_seconds = st.ticks ? double(st.lastTickNs - st.startNs) * 1e-9 : 0.0;
        m.hist_bucket_us = st.wake.bucketNs * 1e-3;
        m.wake_latency_hist.assign(st.wake.counts.begin(), st.wake.counts.end());
        m.exec_hist_bucket_us = st.exec.bucketNs * 1e-3;
        m.exec_hist.assign(st.exec.counts.begin(), st.exec.counts.end());
        statsPub_->publish(m);
    }

    void publishState() {
        const StateSnapshot ss = stateLock_.read();
        visbot_msgs::msg::ControlState m;
        m.header.stamp = now();
        m.header.frame_id = "field";
        m.x = ss.pose.x;
        m.y = ss.pose.y;
        m.theta_deg = ss.pose.wrappedTheta();
        m.mission = missionName_;
        m.step_labels = stepLabels_;
        m.step_index = ss.status.pc;
        m.step_count = int32_t(ctrl_->mission().size());
        m.step_name = ss.status.instr;
        m.step_error = ss.status.error;
        m.step_elapsed_ms = ss.status.instrElapsedMs;
        m.mode = visbot::toString(ss.status.mode);
        m.interfered = ss.status.interfered;
        m.last_action = visbot::toString(ss.status.lastAction);
        m.last_action_at_ms = ss.status.lastActionAtMs;
        m.last_exit = visbot::toString(ss.status.lastExit);
        m.done = ss.status.done;
        m.imu_heading_deg = ss.sensors.imuHeadingDeg;
        m.enc_left_deg = ss.sensors.enc.leftDeg;
        m.enc_right_deg = ss.sensors.enc.rightDeg;
        m.sensor_age_us = ss.sensorAgeUs;
        m.cmd_left = ss.cmd.left; m.cmd_right = ss.cmd.right;
        m.cmd_linear_mps = ss.cmd.linear; m.cmd_angular_rps = ss.cmd.angular;
        statePub_->publish(m);

        geometry_msgs::msg::PoseStamped ps;
        ps.header = m.header;
        ps.pose.position.x = visbot::in2m(ss.pose.x);
        ps.pose.position.y = visbot::in2m(ss.pose.y);
        const double yaw = visbot::headingToYaw(ss.pose.wrappedTheta());
        ps.pose.orientation.z = std::sin(yaw / 2);
        ps.pose.orientation.w = std::cos(yaw / 2);
        posePub_->publish(ps);
        if (path_.poses.empty() || std::hypot(path_.poses.back().pose.position.x - ps.pose.position.x,
                                              path_.poses.back().pose.position.y - ps.pose.position.y) > 0.01) {
            path_.poses.push_back(ps);
            if (path_.poses.size() > 4000) path_.poses.erase(path_.poses.begin());
            path_.header.stamp = m.header.stamp;
            pathPub_->publish(path_);
        }
    }

    struct PendingArm { visbot::Pose pose; visbot::Mission mission; };

    // config
    vc::RtConfig cfg_;
    visbot::RobotParams params_;
    visbot::Pose startPose_;
    std::string missionName_, leftJoint_, rightJoint_;
    std::vector<std::string> stepLabels_;
    bool allianceBlue_ = true;
    bool startPoseOverridden_ = false;
    int64_t staleLimitNs_ = 50'000'000;
    bool loopMission_ = false;

    // RT-side state (owned by the RT thread after start)
    std::unique_ptr<visbot::MotionController> ctrl_;
    StateSnapshot state_{};
    double prevLeftRad_ = 0, prevRightRad_ = 0;
    long leftTurns_ = 0, rightTurns_ = 0;

    // mailboxes
    vc::LatestValue<ImuSample> imu_;
    vc::LatestValue<WheelSample> wheels_;
    vc::RealtimeBox<CmdOut> cmdBox_;
    vc::SeqLock<StateSnapshot> stateLock_;
    std::atomic<bool> armRequested_{false};
    std::mutex armMutex_;
    PendingArm pendingArm_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;
    rclcpp::Publisher<visbot_msgs::msg::ControlStats>::SharedPtr statsPub_;
    rclcpp::Publisher<visbot_msgs::msg::ControlState>::SharedPtr statePub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
    rclcpp::TimerBase::SharedPtr statsTimer_, stateTimer_, startTimer_;
    nav_msgs::msg::Path path_;

    // threads
    std::unique_ptr<vc::RtLoop> loop_;
    std::thread pubThread_;
    std::atomic<bool> pubRunning_{true};
    std::mutex pubMutex_;
    std::condition_variable pubCv_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
