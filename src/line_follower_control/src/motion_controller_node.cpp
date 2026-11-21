#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include "line_follower_interfaces/msg/localization_estimate.hpp"

#include <rclcpp/rclcpp.hpp>

#include "line_follower_control/pure_pursuit.hpp"
#include "line_follower_control/rotate_pid.hpp"
#include "line_follower_interfaces/msg/drive_mode.hpp"
#include "line_follower_interfaces/msg/local_path.hpp"
#include "line_follower_interfaces/msg/mission_state.hpp"
#include "line_follower_interfaces/msg/velocity_candidate.hpp"

namespace line_follower_control
{

using line_follower_interfaces::msg::DriveMode;
using line_follower_interfaces::msg::LocalPath;
using line_follower_interfaces::msg::VelocityCandidate;

// Pure Pursuit + rotate-in-place PID. Publishes only a CANDIDATE; never /cmd_vel (design §4.2).
class MotionControllerNode : public rclcpp::Node
{
public:
  MotionControllerNode()
  : Node("motion_controller"),
    pid_(
      declare_parameter("rotate_kp", 1.5), declare_parameter("rotate_ki", 0.0),
      declare_parameter("rotate_kd", 0.05), declare_parameter("max_angular", 1.5))
  {
    max_linear_ = declare_parameter("max_linear", 0.1);
    max_angular_ = get_parameter("max_angular").as_double();
    candidate_validity_sec_ = declare_parameter("candidate_validity_sec", 0.2);
    path_timeout_sec_ = declare_parameter("path_timeout_sec", 0.5);

    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      position_x_=m->pose.pose.position.x;position_y_=m->pose.pose.position.y;speed_=std::hypot(m->twist.twist.linear.x,m->twist.twist.linear.y);odom_wall_=std::chrono::steady_clock::now();
    });
    pose_sub_=create_subscription<line_follower_interfaces::msg::LocalizationEstimate>("/localization/estimate",10,[this](line_follower_interfaces::msg::LocalizationEstimate::ConstSharedPtr m) {
      if(m->healthy) {const auto & q=m->pose.orientation;yaw_=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));pose_wall_=std::chrono::steady_clock::now();}
    });
    mission_sub_=create_subscription<line_follower_interfaces::msg::MissionState>("/mission/state",10,[this](line_follower_interfaces::msg::MissionState::ConstSharedPtr m) {
      if(!m->run_id.empty() && m->run_id!=run_id_) {run_id_=m->run_id;rotating_=rotate_fault_=false;rotate_episode_.reset();forward_origin_.reset();rotation_attempts_=0;pid_.reset();}
    });
    candidate_pub_ = create_publisher<VelocityCandidate>("/control/cmd_vel_candidate", 10);
    path_sub_ = create_subscription<LocalPath>(
      "/path/local", 10, [this](LocalPath::ConstSharedPtr msg) {on_path(*msg);});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {on_watchdog();});

    RCLCPP_INFO(get_logger(), "motion_controller up");
  }

private:
  void on_path(const LocalPath & path)
  {
    last_path_wall_ = std::chrono::steady_clock::now();
    const auto t = now();

    double linear = 0.0;
    double angular = 0.0;
    uint8_t mode = path.mode.value;

    if (t > rclcpp::Time(path.valid_until)) {
      mode = DriveMode::STOPPED;  // expired path -> zero candidate (design §8.2)
    }

    const auto wall=std::chrono::steady_clock::now();
    const bool pose_fresh=pose_wall_ && odom_wall_ && std::chrono::duration<double>(wall-*pose_wall_).count()<.5 && std::chrono::duration<double>(wall-*odom_wall_).count()<.5;
    const bool forward_mode=mode==DriveMode::VISION_TRACKING || mode==DriveMode::MEMORY_TRACKING ||
      mode==DriveMode::REACQUIRING || mode==DriveMode::SLOWDOWN || mode==DriveMode::IMAGE_TRACKING;
    const bool forward_path=forward_mode && pose_fresh && path.speed_limit>0. &&
      (path.image_tracking || (std::isfinite(path.target_point.x) && path.target_point.x>.05 &&
      std::isfinite(path.target_point.y)));
    if(mode==DriveMode::STOPPING || mode==DriveMode::STOPPED) rotating_=false;
    if(forward_path && !rotating_) {
      if(!forward_origin_) {forward_origin_=std::make_pair(position_x_,position_y_);forward_heading_=yaw_;}
      // A new observation stamp alone must not restart a stuck rotation budget.
      if(speed_>.015 && forward_progress(position_x_-forward_origin_->first,position_y_-forward_origin_->second,forward_heading_)) {
        rotate_episode_.reset();rotate_fault_=false;rotation_attempts_=0;
        forward_origin_=std::make_pair(position_x_,position_y_);forward_heading_=yaw_;
      }
    } else forward_origin_.reset();
    if(rotate_episode_ && pose_fresh && std::abs(std::remainder(rotate_goal_-yaw_,2*M_PI))<.08) {
      rotate_episode_.reset();rotating_=rotate_fault_=false;rotation_attempts_=0;pid_.reset();
    }
    const double target=yaw_+path.target_heading;
    const double stamp=rclcpp::Time(path.header.stamp).seconds();
    const bool new_target=pose_fresh && std::isfinite(target) &&
      std::abs(std::remainder(target-rotate_goal_,2*M_PI))>.35 && stamp>rotate_target_stamp_+.5;
    if(mode==DriveMode::ROTATE_IN_PLACE && !pose_fresh) {publish(0,0,DriveMode::STOPPED);return;}
    if(mode==DriveMode::ROTATE_IN_PLACE && !rotating_ && pose_fresh) {
      const bool retry=new_target && rotation_attempts_<2 &&
        (!rotate_fault_ || std::chrono::duration<double>(wall-rotate_fault_wall_).count()>=.8);
      if(rotate_fault_ && !retry) {publish(0,0,DriveMode::STOPPED);return;}
      if(!rotate_episode_ || retry) {rotate_episode_=wall;++rotation_attempts_;rotate_fault_=false;}
      rotate_goal_=target;rotate_target_stamp_=stamp;rotate_start_=*rotate_episode_;
      rotating_=true;pid_.reset();last_rotate_.reset();
    }
    // A fresh valid forward path may move at its conservative speed after a fault;
    // only measured progress above ends the old episode and restores its budget.
    if(rotate_fault_ && !forward_path) {publish(0,0,DriveMode::STOPPED);return;}
    if(rotating_) {
      const double error=std::remainder(rotate_goal_-yaw_,2*M_PI);
      if(!pose_fresh || std::chrono::duration<double>(wall-rotate_start_).count()>8.) {
        rotating_=false;rotate_fault_=true;rotate_fault_wall_=wall;RCLCPP_ERROR(get_logger(),"rotation feedback stale or eight-second budget exhausted; stopped pending fresh target or verified forward progress");publish(0,0,DriveMode::STOPPED);return;
      }
      if(std::abs(error)<.08) {rotate_episode_.reset();rotation_attempts_=0;rotating_=false;pid_.reset();publish(0,0,DriveMode::STOPPED);return;}
      const double dt=last_rotate_?(t-*last_rotate_).seconds():0.;last_rotate_=t;
      publish(0.,speed_<.008?std::clamp(pid_.update(error,dt),-.45,.45):0.,DriveMode::ROTATE_IN_PLACE);return;
    }
    if(path.image_tracking && (mode==DriveMode::IMAGE_TRACKING || mode==DriveMode::REACQUIRING)) {
      linear=std::clamp(double(path.speed_limit),0.,std::min(max_linear_,.06));
      if(!std::isfinite(path.image_offset) || !std::isfinite(path.image_heading)) {publish(0,0,DriveMode::STOPPED);return;}
      angular=image_pursuit_angular(path.image_offset,path.image_heading,linear,.4);
      pid_.reset();publish(linear,angular,mode);return;
    }
    switch (mode) {
      case DriveMode::VISION_TRACKING:
      case DriveMode::MEMORY_TRACKING:
      case DriveMode::REACQUIRING:
      case DriveMode::SLOWDOWN:
        pid_.reset();
        linear = std::clamp(static_cast<double>(path.speed_limit), 0.0, max_linear_);
        angular = pure_pursuit_angular(
          path.target_point.x, path.target_point.y, linear, max_angular_);
        break;
      case DriveMode::ROTATE_IN_PLACE:
        publish(0,0,DriveMode::STOPPED);return;
      default:
        pid_.reset();
        break;
    }
    if (mode != DriveMode::ROTATE_IN_PLACE) {
      last_rotate_.reset();
    }
    publish(linear, angular, mode);
  }

  void on_watchdog()
  {
    const bool stale = !last_path_wall_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - *last_path_wall_).count() >
      path_timeout_sec_;
    if (stale) {
      publish(0.0, 0.0, DriveMode::STOPPED);
    }
  }

  void publish(double linear, double angular, uint8_t mode)
  {
    VelocityCandidate c;
    c.header.stamp = now();
    c.twist.linear.x = linear;
    c.twist.angular.z = angular;
    c.valid_until = rclcpp::Time(c.header.stamp) +
      rclcpp::Duration::from_seconds(candidate_validity_sec_);
    c.mode.value = mode;
    candidate_pub_->publish(c);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<line_follower_interfaces::msg::LocalizationEstimate>::SharedPtr pose_sub_;
  std::optional<std::chrono::steady_clock::time_point> pose_wall_,odom_wall_;
  std::chrono::steady_clock::time_point rotate_start_;
  std::chrono::steady_clock::time_point rotate_fault_wall_;
  std::optional<std::pair<double,double>> forward_origin_;
  double position_x_{0.},position_y_{0.},forward_heading_{0.},rotate_target_stamp_{0.};int rotation_attempts_{0};
  double yaw_{0.},speed_{0.},rotate_goal_{0.};bool rotating_{false},rotate_fault_{false};
  rclcpp::Subscription<line_follower_interfaces::msg::MissionState>::SharedPtr mission_sub_;
  std::string run_id_;
  std::optional<std::chrono::steady_clock::time_point> rotate_episode_;
  RotatePid pid_;
  double max_linear_{0.1};
  double max_angular_{1.5};
  double candidate_validity_sec_{0.2};
  double path_timeout_sec_{0.5};
  std::optional<std::chrono::steady_clock::time_point> last_path_wall_;
  std::optional<rclcpp::Time> last_rotate_;

  rclcpp::Publisher<VelocityCandidate>::SharedPtr candidate_pub_;
  rclcpp::Subscription<LocalPath>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace line_follower_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<line_follower_control::MotionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
