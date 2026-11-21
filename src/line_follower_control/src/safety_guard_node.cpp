#include <algorithm>
#include <chrono>
#include <csignal>
#include <thread>
#include <memory>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "line_follower_control/safety_policy.hpp"
#include "line_follower_interfaces/msg/mission_state.hpp"
#include "line_follower_interfaces/msg/path_status.hpp"
#include "line_follower_interfaces/msg/perception_status.hpp"
#include "line_follower_interfaces/msg/safety_status.hpp"
#include "line_follower_interfaces/msg/velocity_candidate.hpp"

namespace line_follower_control
{

using line_follower_interfaces::msg::MissionState;
using line_follower_interfaces::msg::PathStatus;
using line_follower_interfaces::msg::PerceptionStatus;
using line_follower_interfaces::msg::SafetyStatus;
using line_follower_interfaces::msg::VelocityCandidate;
using SteadyTime = std::chrono::steady_clock::time_point;

// The ONLY publisher of /cmd_vel (design §5.3-5, §9.2).
class SafetyGuardNode : public rclcpp::Node
{
public:
  SafetyGuardNode()
  : Node("safety_guard")
  {
    limits_.max_linear = declare_parameter("max_linear", limits_.max_linear);
    limits_.max_angular = declare_parameter("max_angular", limits_.max_angular);
    limits_.max_linear_accel = declare_parameter("max_linear_accel", limits_.max_linear_accel);
    limits_.max_angular_accel = declare_parameter("max_angular_accel", limits_.max_angular_accel);
    limits_.mission_timeout = declare_parameter("mission_timeout_sec", limits_.mission_timeout);
    limits_.candidate_timeout =
      declare_parameter("candidate_timeout_sec", limits_.candidate_timeout);
    limits_.path_status_timeout =
      declare_parameter("path_status_timeout_sec", limits_.path_status_timeout);
    limits_.perception_timeout =
      declare_parameter("perception_timeout_sec", limits_.perception_timeout);
    limits_.odom_timeout = declare_parameter("odom_timeout_sec", limits_.odom_timeout);
    const double rate_hz = declare_parameter("control_rate_hz", 50.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = create_publisher<SafetyStatus>("/safety/status", 10);

    candidate_sub_ = create_subscription<VelocityCandidate>(
      "/control/cmd_vel_candidate", 10, [this](VelocityCandidate::ConstSharedPtr m) {
        candidate_ = *m;
        candidate_wall_ = std::chrono::steady_clock::now();
      });
    mission_sub_ = create_subscription<MissionState>(
      "/mission/state", 10, [this](MissionState::ConstSharedPtr m) {
        mission_ = *m;
        mission_wall_ = std::chrono::steady_clock::now();
      });
    path_sub_ = create_subscription<PathStatus>(
      "/path/status", 10, [this](PathStatus::ConstSharedPtr m) {
        path_ = *m;
        path_wall_ = std::chrono::steady_clock::now();
      });
    perception_sub_ = create_subscription<PerceptionStatus>(
      "/perception/status", 10, [this](PerceptionStatus::ConstSharedPtr m) {
        perception_ = *m;
        perception_wall_ = std::chrono::steady_clock::now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_stamp_ = rclcpp::Time(m->header.stamp);
        odom_wall_ = std::chrono::steady_clock::now();
      });

    period_ = 1.0 / rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_), [this]() {on_timer();});

    RCLCPP_INFO(get_logger(), "safety_guard up (sole /cmd_vel publisher)");
  }

  void stop_before_shutdown()
  {
    // Called after the single-threaded executor has stopped: no timer can overwrite
    // this final zero. Keep the context and reliable DDS writer alive for delivery.
    timer_->cancel();
    prev_linear_ = prev_angular_ = 0.0;
    for (int i = 0; i < 5; ++i) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const bool acknowledged = cmd_pub_->wait_for_all_acked(std::chrono::milliseconds(150));
    RCLCPP_INFO(get_logger(), "shutdown zero published (DDS acknowledged=%s)",
      acknowledged ? "true" : "false");
  }

private:
  // Age = max(wall time since receipt, sim time since stamp): catches both dead publishers
  // and old data even if /clock freezes.
  double age(const std::optional<SteadyTime> & wall, const std::optional<rclcpp::Time> & stamp)
  const
  {
    if (!wall) {
      return SafetyInputs::kNever;
    }
    double a = std::chrono::duration<double>(std::chrono::steady_clock::now() - *wall).count();
    if (stamp && stamp->get_clock_type() == now().get_clock_type()) {
      a = std::max(a, (now() - *stamp).seconds());
    }
    return a;
  }

  void on_timer()
  {
    SafetyInputs in;
    if (mission_) {
      in.drive_enabled = mission_->drive_enabled;
      in.mission_age = age(mission_wall_, std::nullopt);
    }
    if (candidate_) {
      in.candidate_age = age(candidate_wall_, rclcpp::Time(candidate_->header.stamp));
      in.candidate_expired = now() > rclcpp::Time(candidate_->valid_until);
      in.candidate_linear = candidate_->twist.linear.x;
      in.candidate_angular = candidate_->twist.angular.z;
    }
    if (path_) {
      in.path_valid = path_->valid;
      in.path_status_age = age(path_wall_, std::nullopt);
    }
    if (perception_) {
      in.perception_healthy = perception_->healthy;
      in.perception_age = age(perception_wall_, rclcpp::Time(perception_->last_capture_stamp));
    }
    in.odom_age = age(odom_wall_, odom_stamp_);

    const auto d = evaluate_safety(in, limits_, prev_linear_, prev_angular_, period_);
    prev_linear_ = d.linear;
    prev_angular_ = d.angular;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = d.linear;
    cmd.angular.z = d.angular;
    cmd_pub_->publish(cmd);

    SafetyStatus status;
    status.header.stamp = now();
    status.command_approved = d.approved;
    status.stop_reason = d.reason;
    status.commanded_linear = static_cast<float>(d.linear);
    status.commanded_angular = static_cast<float>(d.angular);
    status_pub_->publish(status);
  }

  SafetyLimits limits_;
  double period_{0.02};
  double prev_linear_{0.0};
  double prev_angular_{0.0};

  std::optional<VelocityCandidate> candidate_;
  std::optional<MissionState> mission_;
  std::optional<PathStatus> path_;
  std::optional<PerceptionStatus> perception_;
  std::optional<rclcpp::Time> odom_stamp_;
  std::optional<SteadyTime> candidate_wall_, mission_wall_, path_wall_, perception_wall_,
    odom_wall_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<VelocityCandidate>::SharedPtr candidate_sub_;
  rclcpp::Subscription<MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<PathStatus>::SharedPtr path_sub_;
  rclcpp::Subscription<PerceptionStatus>::SharedPtr perception_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace line_follower_control

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<line_follower_control::SafetyGuardNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !stop_requested) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  if (rclcpp::ok()) {node->stop_before_shutdown();}
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
