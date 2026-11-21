#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <algorithm>
#include <stdexcept>
#include "line_follower_mission/freshness.hpp"
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "line_follower_interfaces/msg/line_observation.hpp"
#include "line_follower_interfaces/msg/mission_state.hpp"
#include "line_follower_interfaces/msg/perception_status.hpp"
#include "line_follower_interfaces/srv/mission_command.hpp"
#include "line_follower_mission/finish_judge.hpp"
#include "line_follower_mission/command_policy.hpp"
#include "line_follower_mission/snapshot.hpp"
#include "line_follower_interfaces/msg/gcs_snapshot.hpp"
#include "line_follower_interfaces/msg/localization_estimate.hpp"
#include "line_follower_mission/mission_state_machine.hpp"

namespace line_follower_mission
{

using line_follower_interfaces::msg::LineObservation;
using line_follower_interfaces::msg::MissionState;
using line_follower_interfaces::msg::PerceptionStatus;
using line_follower_interfaces::srv::MissionCommand;
using line_follower_interfaces::msg::GcsSnapshot;
using SteadyClock = std::chrono::steady_clock;

class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode()
  : Node("mission_manager"),
    start_(SteadyClock::now()),
    machine_(MissionParams{
      declare_parameter("startup_delay_sec", 20.0),
      declare_parameter("check_timeout_sec", 30.0),
      declare_parameter("auto_start", true)}),
    finish_(declare_parameter("min_travel_before_finish", 1.0))
  {
    sensor_timeout_sec_ = declare_parameter("sensor_timeout_sec", 1.0);
    stop_speed_epsilon_ = declare_parameter("stop_speed_epsilon", 0.01);
    const double rate_hz = declare_parameter("publish_rate_hz", 10.0);
    run_id_ = make_run_id();

    state_pub_ = create_publisher<MissionState>(
      "/mission/state", rclcpp::QoS(10).transient_local());
    perception_sub_ = create_subscription<PerceptionStatus>(
      "/perception/status", 10,
      [this](PerceptionStatus::ConstSharedPtr m) {
        perception_wall_ = SteadyClock::now();
        if (snapshot_pub_) {
          snapshot_.perception_status = *m;
          snapshot_freshness_.receive("perception_status", wall_seconds());
        }
      });
    line_sub_ = create_subscription<LineObservation>(
      "/perception/line", 10, [this](LineObservation::ConstSharedPtr m) {
        finish_confirmed_ = finish_confirmed_ ||
        finish_.update(m->finish_marker_candidate, traveled_distance_);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr) {imu_wall_ = SteadyClock::now();});
    command_srv_ = create_service<MissionCommand>(
      "/mission/command",
      [this](
        const MissionCommand::Request::SharedPtr req,
        MissionCommand::Response::SharedPtr res) {on_command(*req, *res);});

    if (declare_parameter("gcs_snapshot_enabled", false)) {enable_snapshot();}
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {on_timer();});

    RCLCPP_INFO(
      get_logger(), "mission_manager up (run_id=%s, startup_delay=%.1fs)",
      run_id_.c_str(), get_parameter("startup_delay_sec").as_double());
  }

private:
  std::string make_run_id() const
  {
    const std::time_t t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "run_%Y%m%d_%H%M%S", std::localtime(&t));
    return std::string(buf) + "_" + std::to_string(start_.time_since_epoch().count());
  }

  bool fresh(const std::optional<SteadyClock::time_point> & t) const
  {
    return t && std::chrono::duration<double>(SteadyClock::now() - *t).count() <=
           sensor_timeout_sec_;
  }

  void on_odom(const nav_msgs::msg::Odometry & m)
  {
    odom_wall_ = SteadyClock::now();
    if (snapshot_pub_) {
      if(localization_history_.empty()) {snapshot_.robot_pose = m.pose.pose;}
      snapshot_freshness_.receive("pose", wall_seconds());
      odom_history_.push_back(m);
      if (odom_history_.size() > 100) {odom_history_.pop_front();}
    }
    const auto & p = m.pose.pose.position;
    if (last_xy_) {
      traveled_distance_ += std::hypot(p.x - last_xy_->first, p.y - last_xy_->second);
    }
    last_xy_ = std::make_pair(p.x, p.y);
    robot_stopped_ = std::abs(m.twist.twist.linear.x) < stop_speed_epsilon_ &&
      std::abs(m.twist.twist.angular.z) < stop_speed_epsilon_;
  }

  void on_command(const MissionCommand::Request & req, MissionCommand::Response & res)
  {
    const auto reply = command_policy_.handle(
      {req.request_id, req.run_id, req.command}, run_id_, machine_.state(),
      get_parameter("auto_start").as_bool(), start_requested_ || kill_requested_);
    res.accepted = reply.accepted;
    res.reason = reply.reason;
    res.state = reply.state;
    res.run_id = reply.run_id;
    if (reply.execute) {
      if (req.command == MissionCommand::Request::KILL) {kill_requested_ = true;}
      if (req.command == MissionCommand::Request::START) {start_requested_ = true;}
    }
  }

  static double wall_seconds()
  {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  }

  void enable_snapshot()
  {
    const double hz = declare_parameter("gcs_snapshot_rate_hz", 2.0);
    snapshot_timeout_ = declare_parameter("gcs_source_timeout_sec", 1.0);
    if (hz <= 0.0) {throw std::invalid_argument("gcs_snapshot_rate_hz must be positive");}
    snapshot_pub_ = create_publisher<GcsSnapshot>("/mission/gcs_snapshot", 10);
    localization_sub_ = create_subscription<line_follower_interfaces::msg::LocalizationEstimate>(
      "/localization/estimate", 10,
      [this](line_follower_interfaces::msg::LocalizationEstimate::ConstSharedPtr m) {
        if(!m->healthy || m->header.frame_id != "odom") {return;}
        snapshot_.robot_pose = m->pose;localization_history_.push_back(*m);
        if(localization_history_.size() > 100) {localization_history_.pop_front();}
      });
    graph_sub_ = create_subscription<line_follower_interfaces::msg::ObservedGraph>(
      "/path/observed_graph", 10,
      [this](line_follower_interfaces::msg::ObservedGraph::ConstSharedPtr m) {
        snapshot_.observed_graph = *m;
        snapshot_freshness_.receive("observed_graph", wall_seconds());
      });
    local_sub_ = create_subscription<line_follower_interfaces::msg::LocalPath>(
      "/path/local", 10, [this](line_follower_interfaces::msg::LocalPath::ConstSharedPtr m) {
        snapshot_.path_valid = false;
        if (odom_history_.empty()) {return;}
        const auto stamp = rclcpp::Time(m->header.stamp);
        const auto nearest = std::min_element(odom_history_.begin(), odom_history_.end(),
        [&stamp](const auto & a, const auto & b) {
          return std::abs((rclcpp::Time(a.header.stamp) - stamp).seconds()) <
                 std::abs((rclcpp::Time(b.header.stamp) - stamp).seconds());
          });
        if (std::abs((rclcpp::Time(nearest->header.stamp) - stamp).seconds()) > 0.15) {return;}
        if (m->header.frame_id != "odom" && m->header.frame_id != "base_footprint" &&
        m->header.frame_id != "base_link") {return;}
        snapshot_freshness_.receive("selected_path", wall_seconds());
        auto basis = nearest->pose.pose;
        if(!localization_history_.empty()) {
          const auto sample = std::min_element(localization_history_.begin(),
            localization_history_.end(),
          [&](const auto & a, const auto & b) {
            return std::abs(rclcpp::Time(a.header.stamp).nanoseconds() - stamp.nanoseconds()) <
                   std::abs(rclcpp::Time(b.header.stamp).nanoseconds() - stamp.nanoseconds());
            });
          if(std::abs(rclcpp::Time(sample->header.stamp).nanoseconds() - stamp.nanoseconds()) <
          100000000) {basis = sample->pose;}
        }
        snapshot_.selected_path = path_in_odom(*m, basis);
        snapshot_.path_valid = !m->points.empty();
      });
    candidates_sub_ = create_subscription<line_follower_interfaces::msg::PathCandidates>(
      "/path/candidates", 10,
      [this](line_follower_interfaces::msg::PathCandidates::ConstSharedPtr m) {
        snapshot_.candidates = *m;
        snapshot_freshness_.receive("candidates", wall_seconds());
      });
    path_status_sub_ = create_subscription<line_follower_interfaces::msg::PathStatus>(
      "/path/status", 10, [this](line_follower_interfaces::msg::PathStatus::ConstSharedPtr m) {
        if (snapshot_.path_status.mode.value != m->mode.value) {
          snapshot_.mode_events.push_back(*m);
          if (snapshot_.mode_events.size() > 32) {
            snapshot_.mode_events.erase(snapshot_.mode_events.begin());
          }
        }
        snapshot_.path_status = *m;
        snapshot_freshness_.receive("path_status", wall_seconds());
      });
    safety_sub_ = create_subscription<line_follower_interfaces::msg::SafetyStatus>(
      "/safety/status", 10, [this](line_follower_interfaces::msg::SafetyStatus::ConstSharedPtr m) {
        if (snapshot_.safety_status.stop_reason != m->stop_reason) {
          snapshot_.safety_events.push_back(*m);
          if (snapshot_.safety_events.size() > 32) {
            snapshot_.safety_events.erase(snapshot_.safety_events.begin());
          }
        }
        snapshot_.safety_status = *m;
        snapshot_freshness_.receive("safety_status", wall_seconds());
      });
    snapshot_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this]() {
          if (!get_parameter("gcs_snapshot_enabled").as_bool()) {return;}
          auto out = snapshot_;
          out.stale_sources.clear();
          for (const auto & source : {"pose", "selected_path", "observed_graph", "candidates",
            "path_status", "safety_status", "perception_status"})
          {
            if (!snapshot_freshness_.valid(source, wall_seconds(), snapshot_timeout_)) {
              out.stale_sources.push_back(source);
            }
          }
          out.header.stamp = now();
          out.header.frame_id = "odom";
          out.pose_valid = fresh(odom_wall_);
          out.path_valid = out.path_valid && out.path_status.valid &&
          now() <= rclcpp::Time(out.selected_path.valid_until);
          snapshot_pub_->publish(out);
          snapshot_.mission_events.clear();
          snapshot_.mode_events.clear();
          snapshot_.safety_events.clear();
    });
  }

  void on_timer()
  {
    const double elapsed = std::chrono::duration<double>(SteadyClock::now() - start_).count();
    const bool ready = fresh(perception_wall_) && fresh(odom_wall_) && fresh(imu_wall_);

    MissionInputs in;
    in.elapsed_sec = elapsed;
    in.sensors_ready = ready;
    in.sensors_lost = !ready;
    in.finish_confirmed = finish_confirmed_;
    in.robot_stopped = robot_stopped_;
    in.start_requested = start_requested_;
    in.kill_requested = kill_requested_;

    const uint8_t before = machine_.state();
    machine_.step(in);
    start_requested_ = false;
    kill_requested_ = false;
    if (machine_.state() != before) {
      RCLCPP_INFO(
        get_logger(), "mission state %u -> %u (%s)", before, machine_.state(),
        machine_.reason().c_str());
    }

    MissionState msg;
    msg.header.stamp = now();
    msg.state = machine_.state();
    msg.drive_enabled = machine_.drive_enabled();
    msg.reason = machine_.reason();
    msg.run_id = run_id_;
    msg.elapsed_since_start = static_cast<float>(elapsed);
    state_pub_->publish(msg);
    if (snapshot_pub_) {
      snapshot_.mission = msg;
      if (before != machine_.state()) {
        snapshot_.mission_events.push_back(msg);
        if (snapshot_.mission_events.size() > 32) {
          snapshot_.mission_events.erase(snapshot_.mission_events.begin());
        }
      }
    }
  }

  CommandPolicy command_policy_;
  GcsSnapshot snapshot_;
  Freshness snapshot_freshness_;
  double snapshot_timeout_{1.0};
  rclcpp::Subscription<line_follower_interfaces::msg::LocalizationEstimate>::SharedPtr
    localization_sub_;
  std::deque<line_follower_interfaces::msg::LocalizationEstimate> localization_history_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  rclcpp::Publisher<GcsSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Subscription<line_follower_interfaces::msg::ObservedGraph>::SharedPtr graph_sub_;
  rclcpp::Subscription<line_follower_interfaces::msg::LocalPath>::SharedPtr local_sub_;
  rclcpp::Subscription<line_follower_interfaces::msg::PathCandidates>::SharedPtr candidates_sub_;
  rclcpp::Subscription<line_follower_interfaces::msg::PathStatus>::SharedPtr path_status_sub_;
  rclcpp::Subscription<line_follower_interfaces::msg::SafetyStatus>::SharedPtr safety_sub_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;
  SteadyClock::time_point start_;
  MissionStateMachine machine_;
  FinishJudge finish_;
  std::string run_id_;
  double sensor_timeout_sec_{1.0};
  double stop_speed_epsilon_{0.01};

  std::optional<SteadyClock::time_point> perception_wall_, odom_wall_, imu_wall_;
  std::optional<std::pair<double, double>> last_xy_;
  double traveled_distance_{0.0};
  bool robot_stopped_{true};
  bool finish_confirmed_{false};
  bool start_requested_{false};
  bool kill_requested_{false};

  rclcpp::Publisher<MissionState>::SharedPtr state_pub_;
  rclcpp::Subscription<PerceptionStatus>::SharedPtr perception_sub_;
  rclcpp::Subscription<LineObservation>::SharedPtr line_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<MissionCommand>::SharedPtr command_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace line_follower_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<line_follower_mission::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
