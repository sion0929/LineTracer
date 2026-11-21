#pragma once

#include <deque>
#include <chrono>
#include <optional>
#include <string>
#include <utility>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "line_follower_interfaces/msg/localization_estimate.hpp"

namespace line_follower_navigation
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Robot pose from /odom + /imu (design §7.3). EKF is optional and only adopted after T-25.
class PoseEstimator
{
public:
  explicit PoseEstimator(std::size_t history_size = 600, std::string frame = "odom")
  : history_size_(history_size), frame_(std::move(frame)) {}

  void use_imu_yaw(bool enabled) {imu_yaw_enabled_ = enabled;}
  void add_odom(const nav_msgs::msg::Odometry & odom);
  void add_imu(const sensor_msgs::msg::Imu & imu);

  // Pose at the camera capture stamp.
  // Interpolates inside the history only; stale captures are never extrapolated.
  std::optional<Pose2D> pose_at(const rclcpp::Time & stamp, double max_age_sec = .5) const;
  std::optional<Pose2D> latest() const;
  double pitch() const {return pitch_;}

  bool healthy(const rclcpp::Time & now, double timeout_sec) const;
  line_follower_interfaces::msg::LocalizationEstimate estimate(
    const rclcpp::Time & now, double timeout_sec) const;

  void reset();

private:
  bool imu_yaw_enabled_{false};
  std::deque<std::pair<int64_t, double>> gyro_;
  std::optional<double> gyro_anchor_;
  double gyro_rate_{0.};
  std::optional<double> gyro_at(int64_t stamp) const;
  std::size_t history_size_;
  std::deque<std::pair<rclcpp::Time, Pose2D>> history_;
  double pitch_{0.0};
  std::optional<int64_t> imu_stamp_;
  std::optional<std::chrono::steady_clock::time_point> odom_received_, imu_received_;
  std::string frame_;
};

}  // namespace line_follower_navigation
