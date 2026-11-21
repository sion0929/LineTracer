#include "line_follower_navigation/pose_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace line_follower_navigation
{

namespace
{
double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double pitch_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}
}  // namespace

void PoseEstimator::add_odom(const nav_msgs::msg::Odometry & odom)
{
  if (!odom.header.frame_id.empty() && odom.header.frame_id != frame_) {return;}
  const auto & p = odom.pose.pose;
  if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y)) {return;}
  const auto stamp = rclcpp::Time(odom.header.stamp);
  if (!history_.empty() && stamp.nanoseconds() < history_.back().first.nanoseconds()) {reset();}
  if (!history_.empty() && stamp.nanoseconds() == history_.back().first.nanoseconds()) {
    history_.pop_back();
  }
  odom_received_ = std::chrono::steady_clock::now();
  double heading = yaw_from_quat(p.orientation);
  if(imu_yaw_enabled_) {
    if(auto gyro = gyro_at(stamp.nanoseconds())) {
      if(!gyro_anchor_) {gyro_anchor_ = heading - *gyro;}
      heading = std::remainder(*gyro_anchor_ + *gyro, 2 * M_PI);
    } else if(gyro_.empty() && !gyro_anchor_) {gyro_anchor_ = heading;}
  }
  history_.emplace_back(
    rclcpp::Time(odom.header.stamp),
    Pose2D{p.position.x, p.position.y, heading});
  while (history_.size() > history_size_) {
    history_.pop_front();
  }
}

void PoseEstimator::add_imu(const sensor_msgs::msg::Imu & imu)
{
  const auto & q = imu.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (imu.orientation_covariance[0] < 0 || !std::isfinite(norm) || std::abs(norm - 1.) > .05) {
    return;
  }
  const int64_t ns = rclcpp::Time(imu.header.stamp).nanoseconds();
  if(std::isfinite(imu.angular_velocity.z)) {
    if(!gyro_.empty() && ns < gyro_.back().first) {gyro_.clear();gyro_anchor_.reset();}
    if(gyro_.empty()) {gyro_.emplace_back(ns, 0.);} else if(ns > gyro_.back().first) {
      const double dt = (ns - gyro_.back().first) * 1e-9;
      if(dt < .2) {
        gyro_.emplace_back(ns,
            gyro_.back().second + .5 * (gyro_rate_ + imu.angular_velocity.z) * dt);
      } else {gyro_.clear();gyro_anchor_.reset();gyro_.emplace_back(ns, 0.);}
    }
    gyro_rate_ = imu.angular_velocity.z;
    while(gyro_.size() > history_size_ * 4) {gyro_.pop_front();}
  }
  pitch_ = pitch_from_quat(q);
  imu_received_ = std::chrono::steady_clock::now();
  imu_stamp_ = rclcpp::Time(imu.header.stamp).nanoseconds();
}

std::optional<double> PoseEstimator::gyro_at(int64_t ns) const
{
  if(gyro_.empty() || ns < gyro_.front().first || ns - gyro_.back().first > 20000000) {
    return std::nullopt;
  }
  auto after = std::lower_bound(gyro_.begin(), gyro_.end(), ns, [](const auto & p, int64_t t){
        return p.first < t;
                                                                                                                 });
  if(after == gyro_.end()) {
    return gyro_.back().second + (ns - gyro_.back().first) * 1e-9 * gyro_rate_;
  }
  if(after == gyro_.begin() || after->first == ns) {return after->second;}
  auto before = std::prev(after);
  return before->second + double(ns - before->first) / double(after->first - before->first) *
         (after->second - before->second);
}

std::optional<Pose2D> PoseEstimator::pose_at(const rclcpp::Time & stamp, double max_age_sec) const
{
  const auto ns = stamp.nanoseconds();
  if (history_.empty() || ns < history_.front().first.nanoseconds() ||
    ns > history_.back().first.nanoseconds() ||
    (history_.back().first.nanoseconds() - ns) * 1e-9 > max_age_sec) {return std::nullopt;}
  auto after = std::lower_bound(history_.begin(), history_.end(), ns,
      [](const auto & entry, int64_t t){return entry.first.nanoseconds() < t;});
  if (after == history_.begin() || after->first.nanoseconds() == ns) {return after->second;}
  const auto before = std::prev(after);
  const double alpha = double(ns - before->first.nanoseconds()) /
    double(after->first.nanoseconds() - before->first.nanoseconds());
  const double dyaw = std::remainder(after->second.yaw - before->second.yaw, 2 * M_PI);
  return Pose2D{before->second.x + alpha * (after->second.x - before->second.x),
    before->second.y + alpha * (after->second.y - before->second.y),
    std::remainder(before->second.yaw + alpha * dyaw, 2 * M_PI)};
}

std::optional<Pose2D> PoseEstimator::latest() const
{
  if (history_.empty()) {
    return std::nullopt;
  }
  return history_.back().second;
}

bool PoseEstimator::healthy(const rclcpp::Time & now, double timeout_sec) const
{
  if (history_.empty() || !imu_stamp_ || !odom_received_ || !imu_received_) {return false;}
  const auto wall = std::chrono::steady_clock::now();
  if(std::chrono::duration<double>(wall - *odom_received_).count() >= timeout_sec ||
    std::chrono::duration<double>(wall - *imu_received_).count() >= timeout_sec) {return false;}
  const double odom_age = (now.nanoseconds() - history_.back().first.nanoseconds()) * 1e-9;
  const double imu_age = (now.nanoseconds() - *imu_stamp_) * 1e-9;
  return odom_age >= -.05 && imu_age >= -.05 && odom_age < timeout_sec && imu_age < timeout_sec;
}

line_follower_interfaces::msg::LocalizationEstimate PoseEstimator::estimate(
  const rclcpp::Time & now, double timeout_sec) const
{
  line_follower_interfaces::msg::LocalizationEstimate est;
  est.header.frame_id = frame_;
  est.healthy = healthy(now, timeout_sec);
  est.pitch = static_cast<float>(pitch_);
  if (!history_.empty()) {
    const auto & [t, pose] = history_.back();
    est.header.stamp = t;
    est.pose.position.x = pose.x;
    est.pose.position.y = pose.y;
    est.pose.orientation.z = std::sin(pose.yaw / 2.0);
    est.pose.orientation.w = std::cos(pose.yaw / 2.0);
  }
  return est;
}

void PoseEstimator::reset()
{
  history_.clear();gyro_.clear();gyro_anchor_.reset();gyro_rate_ = 0.;
  pitch_ = 0.0;
  imu_stamp_.reset();odom_received_.reset();imu_received_.reset();
}

}  // namespace line_follower_navigation
