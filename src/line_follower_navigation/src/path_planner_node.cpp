#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <deque>
#include <set>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <limits>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "line_follower_interfaces/msg/drive_mode.hpp"
#include "line_follower_interfaces/msg/line_observation.hpp"
#include "line_follower_interfaces/msg/localization_estimate.hpp"
#include "line_follower_interfaces/msg/observed_graph.hpp"
#include "line_follower_interfaces/msg/mission_state.hpp"
#include "line_follower_navigation/candidate_feature_builder.hpp"
#include "line_follower_navigation/local_path_mapper.hpp"
#include "line_follower_navigation/local_path_memory.hpp"
#include "line_follower_navigation/path_evaluator.hpp"
#include "line_follower_navigation/pose_estimator.hpp"
#include "line_follower_navigation/trajectory_planner.hpp"
#include "line_follower_navigation/memory_policy.hpp"
#include "line_follower_navigation/selected_path_memory.hpp"

namespace line_follower_navigation
{

using line_follower_interfaces::msg::DriveMode;
using line_follower_interfaces::msg::LineObservation;
using line_follower_interfaces::msg::LocalizationEstimate;
using line_follower_interfaces::msg::ObservedGraph;

// Single C++ node hosting LocalPathMapper / LocalPathMemory / PoseEstimator /
// CandidateFeatureBuilder / evaluators / TrajectoryPlanner (design §7.1).
class PathPlannerNode : public rclcpp::Node
{
public:
  explicit PathPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("path_planner", options)
  {
    PlannerParams p;
    p.base_frame = declare_parameter("base_frame", p.base_frame);
    p.lookahead = declare_parameter("lookahead", p.lookahead);
    p.max_speed = declare_parameter("max_speed", p.max_speed);
    p.min_observed_length = declare_parameter("min_observed_length", p.min_observed_length);
    p.path_validity_sec = declare_parameter("path_validity_sec", p.path_validity_sec);
    const auto odom_frame = declare_parameter<std::string>("odom_frame", "odom");
    odom_frame_ = odom_frame;
    pose_ = PoseEstimator(600, odom_frame);
    pose_.use_imu_yaw(declare_parameter("imu_yaw_enabled", false));
    observation_timeout_sec_ = declare_parameter("observation_timeout_sec", 0.5);
    odom_timeout_sec_ = declare_parameter("odom_timeout_sec", 0.5);
    const bool use_mlp = declare_parameter("use_mlp", false);
    const auto mlp_model_path = declare_parameter<std::string>("mlp_model_path", "");

    planner_ = std::make_unique<TrajectoryPlanner>(p);
    mapper_ = std::make_unique<LocalPathMapper>(odom_frame);
    memory_ = std::make_unique<LocalPathMemory>(declare_parameter("memory_retention_sec", 10.0),
      declare_parameter("memory_max_points", 1200), declare_parameter("memory_max_segments", 32));
    image_enabled_=declare_parameter("image_tracking_enabled",true);
    memory_speed_limit_ = declare_parameter("memory_speed_limit", .06);
    if (use_mlp) {
      mlp_ = std::make_unique<MlpPathEvaluator>(mlp_model_path);
      if (!mlp_->available()) {
        RCLCPP_WARN(get_logger(), "MLP evaluator unavailable, using rule-based evaluation");
      }
    }

    local_path_pub_ = create_publisher<LocalPath>("/path/local", 10);
    status_pub_ = create_publisher<PathStatus>("/path/status", 10);
    candidates_pub_ = create_publisher<PathCandidates>("/path/candidates", 10);
    memory_pub_ = create_publisher<ObservedGraph>("/path/memory", 10);
    graph_pub_ = create_publisher<ObservedGraph>("/path/observed_graph", 10);
    localization_pub_ = create_publisher<LocalizationEstimate>("/localization/estimate", 10);

    // Default mutually-exclusive group serializes all planner state. The MT container
    // runs this group independently of the detector's heavy image callback group.
    line_sub_ = create_subscription<LineObservation>(
      "/perception/line", 10,
      [this, life = lifetime_](LineObservation::ConstSharedPtr msg,
      const rclcpp::MessageInfo & info) {
        std::lock_guard<std::mutex> lock(life->mutex);if(life->closing) {return;}
        if (!reported_transport_) {
          RCLCPP_INFO(get_logger(), "line callback intra_process=%s",
            info.get_rmw_message_info().from_intra_process ? "true" : "false");
          reported_transport_ = true;
        }
        pending_.push_back(msg);
        while(pending_.size() > 3) {pending_.pop_front();}
        process_pending();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this, life = lifetime_](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(life->mutex);if(life->closing) {return;}
        const auto odom_stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if(last_odom_stamp_ && odom_stamp < *last_odom_stamp_) {
          clear_memory(rclcpp::Time(msg->header.stamp), "odometry time reset");
        }
        last_odom_stamp_ = odom_stamp;
        pose_.add_odom(*msg);
        if(auto current = pose_.latest()) {memory_->update(now(), *current);selected_path_.update(now(),*current);}
        process_pending();
        localization_pub_->publish(pose_.estimate(now(), odom_timeout_sec_));
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      [this, life = lifetime_](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(life->mutex);
        if(life->closing) {return;} pose_.add_imu(*msg);process_pending();
                                                                                                                    });

    mission_sub_ =
      create_subscription<line_follower_interfaces::msg::MissionState>("/mission/state", 10,
        [this, life = lifetime_](line_follower_interfaces::msg::MissionState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(life->mutex);if(life->closing) {return;}
          const bool changed = memory_->set_run_id(msg->run_id);
          const bool recovering = (mission_state_ == msg->FAULT || mission_state_ == msg->E_STOP ||
          mission_state_ == msg->STOPPED) &&
          msg->state < msg->FINISHED;
          if(changed ||
          (msg->state != mission_state_ && (msg->state >= msg->FINISHED || recovering)))
          {
            clear_memory(now(), "mission memory reset");
          }
          mission_state_ = msg->state;
      });

    reset_service_ = create_service<std_srvs::srv::Trigger>("/path/reset_memory",
        [this, life = lifetime_](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(life->mutex);if(life->closing) {return;}
          clear_memory(now(), "local memory reset");response->success = true;
          response->message = "local memory cleared";
      });

    // Wall timer so the watchdog never waits on /clock or new input (design §5.2).
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this, life = lifetime_]() {
          std::lock_guard<std::mutex> lock(life->mutex);if(!life->closing) {
            on_watchdog();
          }
                                                                                                                                                                              });

    RCLCPP_INFO(get_logger(), "path_planner up (evaluator=%s)", mlp_ ? "mlp+rule" : "rule");
  }

  ~PathPlannerNode() override
  {
    std::lock_guard<std::mutex> lock(lifetime_->mutex);lifetime_->closing = true;
  }

private:
  struct Lifetime {std::mutex mutex;bool closing{false};};
  std::shared_ptr<Lifetime> lifetime_{std::make_shared<Lifetime>()};

  void clear_memory(const rclcpp::Time & stamp, const std::string & reason)
  {
    memory_->reset();selected_path_.reset();image_start_.reset();look_session_.reset();policy_.reset();tentative_.clear();pending_.clear();
    blind_search_.reset();recovery_bias_=1.0;stopping_since_.reset();image_lost_since_.reset();
    last_observation_wall_.reset();
    reset_stamp_ = stamp.nanoseconds();
    publish_memory(stamp);
    auto graph = std::make_unique<ObservedGraph>();graph->header.stamp = stamp;
    graph->header.frame_id = odom_frame_;
    graph_pub_->publish(std::move(graph));publish_stop(reason);
  }

  void process_pending()
  {
    if(!pose_.healthy(now(), odom_timeout_sec_)) {return;}
    const auto basis = pose_.estimate(now(), odom_timeout_sec_);
    while(!pending_.empty()) {
      const auto msg = pending_.front();
      const auto stamp = rclcpp::Time(msg->header.stamp);
      if(stamp.nanoseconds() < reset_stamp_) {pending_.pop_front();continue;}
      if(stamp.nanoseconds() > rclcpp::Time(basis.header.stamp).nanoseconds()) {break;}
      pending_.pop_front();
      const auto capture = pose_.pose_at(stamp, observation_timeout_sec_);
      if(!capture ||
        (now().nanoseconds() - stamp.nanoseconds()) * 1e-9 > observation_timeout_sec_)
      {
        publish_stop("capture pose outside interpolation history or stale");continue;
      }
      on_observation(*msg, *capture, basis);
    }
  }

  void publish_memory(const rclcpp::Time & stamp)
  {
    auto graph = std::make_unique<ObservedGraph>();graph->header.stamp = stamp;
    graph->header.frame_id = odom_frame_;
    graph->segments = memory_->segments();
    for(auto & s:graph->segments) {
      for(auto & p:s.points) {
        p.source = PathPoint::SOURCE_MEMORY_UPDATED;
      }
    }
    memory_pub_->publish(std::move(graph));
  }

  // Last-resort bounded rotation search (round 3 recovery): tried only in place of a
  // permanent stop, only with fresh pose, and only while BlindSearchSession has not
  // already exhausted its bounded legs for this stall. Publishes a valid /path/status
  // with an honest ROTATE_IN_PLACE detail so motion_controller/safety_guard see a
  // legitimate (never forward-translating) command; see design §9.2 on path validity.
  // Returns true when this cycle was handled (caller must not also publish_stop).
  bool attempt_blind_recovery(
    const rclcpp::Time & t, const Pose2D & current, const LocalizationEstimate & basis,
    const ObservedGraph & graph, const std::vector<PathPoint> & known,
    const std::string & lost_reason)
  {
    if(!pose_.healthy(t, odom_timeout_sec_)) {return false;}  // never rotate blind without fresh pose
    const auto turn = blind_search_.update(t.seconds(), current.yaw, recovery_bias_, true);
    if(!turn) {
      graph_pub_->publish(std::make_unique<ObservedGraph>(graph));publish_memory(basis.header.stamp);
      publish_stop("bounded rotation recovery exhausted; no line reacquired (" + lost_reason + ")");
      return true;
    }
    auto path = planner_->stop_path(t);path.header.stamp = basis.header.stamp;
    path.points = known;path.mode.value = DriveMode::ROTATE_IN_PLACE;path.target_heading = *turn;
    PathStatus status;status.header.stamp = t;status.valid = true;status.mode = path.mode;
    status.verified_remaining_distance = MemoryPolicy::observed_length(known);
    for(const auto & p : known) {
      status.uncertainty = std::max(status.uncertainty, p.uncertainty);
      if(rclcpp::Time(p.valid_until) < rclcpp::Time(path.valid_until)) {path.valid_until = p.valid_until;}
    }
    status.detail = "bounded recovery rotation (leg " + std::to_string(blind_search_.leg()) +
      "/3, attempt " + std::to_string(blind_search_.attempts()) + "): searching for line after " +
      lost_reason;
    graph_pub_->publish(std::make_unique<ObservedGraph>(graph));publish_memory(basis.header.stamp);
    local_path_pub_->publish(path);status_pub_->publish(status);
    return true;
  }

  void publish_image_path(
    const LineObservation & obs, const LocalizationEstimate & basis,
    const Pose2D & current, const rclcpp::Time & t, const ObservedGraph & graph,
    bool direct, const std::string & reason, const std::vector<PathPoint> & known)
  {
    if(!image_start_) {image_start_=t.seconds();image_origin_=current;image_distance_=0.;image_last_=current;}
    image_distance_+=std::hypot(current.x-image_last_.x,current.y-image_last_.y);image_last_=current;
    const bool lost = !obs.image_tracking_valid;
    if(lost || t.seconds()-*image_start_>50. || image_distance_>2.2) {
      // Debounce on a dedicated "continuously lost since" clock (distinct from
      // image_start_, which only marks entry into image-fallback mode and can
      // already be old by the time a single noisy frame flips image_tracking_valid
      // false). A single bad frame mid-curve must not immediately spin the robot;
      // only genuinely sustained loss (>= 1 s continuously lost) escalates to the
      // bounded blind search. A motion_controller rotation episode, once started,
      // is not cancelled by the next good frame (design: bounded, not jittery), so
      // this debounce also protects against spurious one-frame rotation diversions.
      if(!image_lost_since_) {image_lost_since_ = t.seconds();}
      const bool sustained = lost && (t.seconds()-*image_lost_since_ >= 1.0);
      if(sustained && attempt_blind_recovery(t,current,basis,graph,known,
        "image line lost/ambiguous or image tracking budget exhausted"))
      {
        return;
      }
      publish_stop("image line lost/ambiguous or image tracking budget exhausted");return;
    }
    image_lost_since_.reset();
    blind_search_.reset();
    auto path=planner_->stop_path(t);path.header.stamp=basis.header.stamp;
    path.mode.value=direct?DriveMode::REACQUIRING:DriveMode::IMAGE_TRACKING;
    path.image_tracking=true;path.image_offset=obs.image_offset;path.image_heading=obs.image_heading;
    path.speed_limit=(std::abs(obs.image_offset)>.15 || std::abs(obs.image_heading)>.1)?.035:.06;
    PathStatus status;status.header.stamp=t;status.valid=true;status.mode=path.mode;
    status.detail=direct?"verified image steering while metric memory is re-evaluated: "+reason:"bounded single-line image steering; no confirmed 3D geometry";
    graph_pub_->publish(std::make_unique<ObservedGraph>(graph));publish_memory(basis.header.stamp);
    local_path_pub_->publish(path);status_pub_->publish(status);
  }

  void on_observation(
    const LineObservation & obs, const Pose2D & capture,
    const LocalizationEstimate & basis, bool received = true)
  {
    if(received) {last_observation_wall_ = std::chrono::steady_clock::now();}
    const auto t = now();
    auto graph = mapper_->map(obs, capture);
    const auto current = pose_.latest();if(!current) {publish_stop("odometry unavailable");return;}
    if(!pose_.healthy(t,odom_timeout_sec_)) {publish_stop("localization stale; no metric or image driving");return;}
    selected_path_.refresh(memory_->project_to_base(*current),*current);
    const bool direct=obs.projection_valid && obs.line_found && !graph.segments.empty();
    const auto selected=selected_path_.path_to_base(*current,t);
    const bool usable_graph=memory_->path_to_base(*current,t).size()>1;
    const bool usable_memory=usable_graph || selected.size()>1;
    std::vector<LineSegment> selected_reference;
    if(!usable_graph && selected.size()>1) {
      LineSegment segment;segment.id=0;segment.points=selected;
      const double c=std::cos(current->yaw),s=std::sin(current->yaw);
      for(auto & p:segment.points) {const double x=p.x,y=p.y;p.x=current->x+c*x-s*y;p.y=current->y+s*x+c*y;}
      selected_reference.push_back(std::move(segment));
    }
    const auto & reference = usable_graph?memory_->segments():(usable_memory?selected_reference:tentative_);
    const bool consistent = reference.empty() || MemoryPolicy::observations_agree(graph.segments,
        reference);
    auto preview = *memory_;
    if(direct && consistent) {preview.store(graph.segments);}
    auto preview_chain = MemoryPolicy::without_short_backtracking(
      MemoryPolicy::verified_prefix(preview.path_to_base(*current,t),t.nanoseconds()));
    const auto ambiguous=[](const auto & choices) {
      return std::any_of(choices.begin(),choices.end(),[](const auto & c){return c.feasible;}) &&
        std::none_of(choices.begin(),choices.end(),[](const auto & c){return c.selected;});
    };
    if(!ambiguous(preview.decisions()) && selected_path_.prefer(selected,preview_chain))preview_chain=selected;
    const double remaining=MemoryPolicy::observed_length(preview_chain);
    const double uncertainty=MemoryPolicy::control_uncertainty(preview_chain);
    MemoryInput input;input.direct=direct;input.agrees=consistent;
    // Detector already uses capture-time pitch/roll. Never substitute latest pitch here.
    input.pitch=direct?0.:.1;input.remaining=remaining;input.uncertainty=uncertainty;
    auto decision = policy_.update(input);
    if(!direct) tentative_.clear();
    else if(!usable_memory) tentative_=graph.segments;
    auto known=MemoryPolicy::without_short_backtracking(MemoryPolicy::verified_prefix(memory_->path_to_base(*current,t),t.nanoseconds()));
    if(!ambiguous(memory_->decisions()) && selected_path_.prefer(selected,known))known=selected;
    // Track which side the line was last plausibly seen on, so a later bounded
    // blind search (every geometry-grounded recovery failed) sweeps that side
    // first instead of an arbitrary default direction.
    if(obs.image_tracking_valid) {
      if(std::abs(obs.image_offset)>1e-3) {recovery_bias_=obs.image_offset>0?-1.0:1.0;}
      else if(std::abs(obs.image_heading)>1e-3) {recovery_bias_=obs.image_heading>0?1.0:-1.0;}
    }
    std::optional<double> hint;
    if(image_enabled_ && !obs.image_tracking_valid && (!decision.accept_observation || remaining<.18))
      hint=MemoryPolicy::observed_tangent_turn(known);
    if(hint) {recovery_bias_=*hint>=0?1.0:-1.0;}
    const auto look_turn=look_session_.update(t.seconds(),current->x,current->y,current->yaw,hint,image_enabled_ && known.size()>=2);
    if(look_session_.timed_out()) {
      if(attempt_blind_recovery(t,*current,basis,graph,known,"observed look turn timed out")) {return;}
      publish_stop("observed look turn timed out; stopped to re-evaluate");return;
    }
    if(look_turn) {
      blind_search_.reset();
      auto path=planner_->stop_path(t);path.header.stamp=basis.header.stamp;
      path.points=known;path.mode.value=DriveMode::ROTATE_IN_PLACE;path.target_heading=*look_turn;
      PathStatus status;status.header.stamp=t;status.valid=true;status.mode=path.mode;
      status.verified_remaining_distance=MemoryPolicy::observed_length(known);
      for(const auto & p:known) {
        status.uncertainty=std::max(status.uncertainty,p.uncertainty);
        if(rclcpp::Time(p.valid_until)<rclcpp::Time(path.valid_until)) path.valid_until=p.valid_until;
      }
      status.detail="stopped look turn toward observed memory tangent; no unobserved translation";
      graph_pub_->publish(std::make_unique<ObservedGraph>(graph));publish_memory(basis.header.stamp);
      local_path_pub_->publish(path);status_pub_->publish(status);return;
    }
    if(image_enabled_ && (!decision.accept_observation || remaining<.10) &&
      MemoryPolicy::needs_image_fallback(usable_memory,remaining,uncertainty,decision.mode==DriveMode::STOPPING && direct && !consistent)) {
      if(std::getenv("LF_PROJECTION_DEBUG")) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
        "image fallback direct=%d agrees=%d usable=%d preview=%zu remaining=%.3f U=%.3f known=%zu known_length=%.3f image=%d reason=%s",
        direct,consistent,usable_memory,preview_chain.size(),remaining,uncertainty,known.size(),MemoryPolicy::observed_length(known),obs.image_tracking_valid,decision.reason.c_str());
      publish_image_path(obs,basis,*current,t,graph,direct,decision.reason,known);return;
    }
    if(direct && memory_->segments().empty()) {tentative_ = graph.segments;}
    if(!decision.accept_observation) {graph.segments.clear(), graph.junctions.clear();}
    const auto ids = memory_->store(graph.segments);
    graph.segments.erase(std::remove_if(graph.segments.begin(), graph.segments.end(),
      [&](const auto & s){return !ids.count(s.id);}), graph.segments.end());
    for(auto & s:graph.segments) {
      s.connected_ids.erase(std::remove_if(s.connected_ids.begin(), s.connected_ids.end(),
        [&](auto id){return !ids.count(id);}), s.connected_ids.end());
    }
    graph.junctions.erase(std::remove_if(graph.junctions.begin(), graph.junctions.end(),
      [&](const auto & j){
        return std::any_of(j.segment_ids.begin(), j.segment_ids.end(), [&](auto id){
                 return !ids.count(id);
                                                                                                                            });
                                                                                                                               }),
        graph.junctions.end());
    for(auto & s:graph.segments) {
      s.id = ids.at(s.id);for(auto & p:s.points) {p.segment_id = s.id;}
      for(auto & id:s.connected_ids) {id = ids.at(id);}
    }
    for(auto & j:graph.junctions) {for(auto & id:j.segment_ids) {id = ids.at(id);}}
    auto chain = MemoryPolicy::verified_prefix(memory_->path_to_base(*current,t),t.nanoseconds());
    bool selected_memory=false;
    if(!ambiguous(memory_->decisions()) && selected_path_.prefer(selected,chain)) {chain=selected;selected_memory=true;}
    // Stored geometry is memory. A current raw observation replaces a matching
    // point exactly before it may be labelled OBSERVED (never label a fused point direct).
    const double c = std::cos(current->yaw), s = std::sin(current->yaw);
    for(auto & p:chain) {
      const double wx = current->x + c * p.x - s * p.y, wy = current->y + s * p.x + c * p.y;
      const PathPoint * fresh = nullptr;double best = .018;
      for(const auto & seg:graph.segments) {
        if(seg.id == p.segment_id) {
          for(const auto & q:seg.points) {
            const double d = std::hypot(q.x - wx, q.y - wy);if(d < best) {best = d;fresh = &q;}
          }
        }
      }
      if(fresh) {
        p = *fresh;const double dx = p.x - current->x, dy = p.y - current->y;
        p.x = c * dx + s * dy;p.y = -s * dx + c * dy;p.source = PathPoint::SOURCE_OBSERVED;
      }
    }
    // Proposal geometry can advance the reacquisition counter, but only the
    // already accepted prefix contributes to published validity and braking.
    chain=MemoryPolicy::without_short_backtracking(MemoryPolicy::verified_prefix(chain,t.nanoseconds()));
    const double confirmed_remaining=MemoryPolicy::observed_length(chain);
    const double confirmed_uncertainty=MemoryPolicy::control_uncertainty(chain);
    LineObservation local = obs;local.header.stamp = basis.header.stamp;
    local.projection_valid = true;
    local.segments.clear();local.junctions.clear();local.line_found = chain.size() > 1;
    if(local.line_found) {
      LineSegment segment;segment.id = chain.front().segment_id;segment.points = std::move(chain);
      segment.confidence = .9;
      local.segments.push_back(std::move(segment));
    }
    const auto features = builder_.build(local);
    auto result = planner_->plan(local, features, rule_.score(features), {}, t);
    bool memory = false;std::set<uint32_t> selected_ids;
    for(const auto & p:result.path.points) {
      memory |= p.source == PathPoint::SOURCE_MEMORY_UPDATED;selected_ids.insert(p.segment_id);
      const auto until = rclcpp::Time(p.valid_until).nanoseconds();
      if(until > 0 && until < rclcpp::Time(result.path.valid_until).nanoseconds()) {
        result.path.valid_until = p.valid_until;
      }
    }
    // Stop before the end of verified geometry; uncertainty and braking distance
    // can only reduce the candidate's speed, never add geometry.
    if(decision.mode == DriveMode::STOPPING || confirmed_remaining<=.055) {
      result.path = planner_->stop_path(t);result.status.valid = false;
      result.status.detail=decision.mode==DriveMode::STOPPING?decision.reason:"verified path exhausted; camera gap is not observed geometry";
    }
    if(result.path.mode.value==DriveMode::ROTATE_IN_PLACE && decision.mode!=DriveMode::STOPPING) decision.mode=DriveMode::ROTATE_IN_PLACE;
    if(result.path.mode.value==DriveMode::SLOWDOWN && decision.mode==DriveMode::VISION_TRACKING) decision.mode=DriveMode::SLOWDOWN;
    if(!result.status.valid && decision.mode != DriveMode::REACQUIRING) {
      decision.mode = DriveMode::STOPPING;
    }
    result.path.mode.value = decision.mode;result.status.mode = result.path.mode;
    result.path.speed_limit = std::min(result.path.speed_limit, float(decision.speed));
    const double verified_speed=std::min(.1,std::sqrt(2*.15*std::max(0.,confirmed_remaining-.055)))*
      std::max(0.,1-confirmed_uncertainty/.06);
    result.path.speed_limit=std::min(result.path.speed_limit,float(verified_speed));
    if(memory) {
      result.path.speed_limit = std::min(result.path.speed_limit, float(memory_speed_limit_));
    }
    result.status.uncertainty = confirmed_uncertainty;
    result.status.verified_remaining_distance = confirmed_remaining;
    if(!result.candidates.candidates.empty()) {
      result.candidates.candidates[0].segment_ids.assign(selected_ids.begin(), selected_ids.end());
      result.candidates.candidates[0].reason =
        "confirmed forward polyline; observed branch rules evaluated";
    }
    if(!memory_->decisions().empty()) {
      const auto retained_candidate=result.candidates.candidates.empty()?line_follower_interfaces::msg::PathCandidate{}:result.candidates.candidates.front();
      result.candidates.candidates.clear();
      for(const auto & option:memory_->decisions()) {
        line_follower_interfaces::msg::PathCandidate c;c.candidate_id=option.id;c.segment_ids={option.id};
        c.heading_change=option.heading;c.observed_length=option.length;c.rule_score=option.score;
        c.curvature=2*std::sin(option.heading)/std::max(.08,option.length);c.mlp_score=std::numeric_limits<float>::quiet_NaN();
        c.geometry_feasible=option.feasible;c.selected=option.selected && !selected_memory;c.reason=option.reason;
        if(option.selected && selected_memory)c.reason="partial candidate deferred; retaining the selected connected route";
        result.candidates.selection_locked |= c.selected;
        result.candidates.candidates.push_back(c);
        if(c.selected) decision.reason+="; "+option.reason;
      }
      if(selected_memory) {
        auto held=retained_candidate;held.candidate_id=std::numeric_limits<uint32_t>::max();
        held.segment_ids.assign(selected_ids.begin(),selected_ids.end());held.selected=result.status.valid;held.geometry_feasible=result.status.valid;
        held.reason=result.status.valid?"locked previously selected connected route; partial views cannot skip its observed arc":"previously selected route currently infeasible: "+result.status.detail;
        result.candidates.candidates.push_back(held);result.candidates.selection_locked=result.status.valid;
      }
      if(result.path.mode.value!=DriveMode::ROTATE_IN_PLACE) {
        result.path.speed_limit=std::min(result.path.speed_limit,.05f);
        if(result.path.mode.value==DriveMode::VISION_TRACKING) result.path.mode.value=DriveMode::SLOWDOWN;
        result.status.mode=result.path.mode;
      }
    }
    if(selected_memory)decision.reason+="; retained selected connected route through graph fragmentation";
    if(result.status.valid) result.status.detail = decision.reason;
    else if(result.status.detail.empty()) result.status.detail="no feasible continuous confirmed path";
    if(!result.status.valid && image_enabled_ && obs.image_tracking_valid) {
      publish_image_path(obs,basis,*current,t,graph,direct,
        "metric planner rejected: "+result.status.detail,known);return;
    }
    if(result.status.valid) {
      auto retained=result.path.points;
      const auto retention=rclcpp::Duration::from_seconds(get_parameter("memory_retention_sec").as_double());
      for(auto & p:retained) if(p.source==PathPoint::SOURCE_OBSERVED)
        p.valid_until=rclcpp::Time(p.capture_stamp)+retention;
      selected_path_.remember(retained,*current);
      if(decision.accept_observation) {image_start_.reset();blind_search_.reset();}
      stopping_since_.reset();
    } else {
      // Debounce: a brand-new mission or a momentary single-frame dip (not yet
      // enough confirmed length, a transient uncertainty tick) must not trigger a
      // rotation search. Only a STOPPING result that persists is a genuine stall;
      // "first stop (safety first), then perform a bounded re-acquisition."
      if(!stopping_since_) {stopping_since_ = t.seconds();}
      const bool sustained = t.seconds() - *stopping_since_ >= 1.0;
      if(sustained && attempt_blind_recovery(t,*current,basis,graph,known,result.status.detail)) {
        return;
      }
    }
    graph_pub_->publish(std::make_unique<ObservedGraph>(std::move(graph)));
    publish_memory(basis.header.stamp);
    local_path_pub_->publish(std::make_unique<LocalPath>(std::move(result.path)));
    status_pub_->publish(std::make_unique<PathStatus>(std::move(result.status)));
    candidates_pub_->publish(std::make_unique<PathCandidates>(std::move(result.candidates)));
  }

  void on_watchdog()
  {
    const auto age = last_observation_wall_ ?
      std::chrono::duration<double>(std::chrono::steady_clock::now() - *last_observation_wall_)
      .count() : 1e9;
    localization_pub_->publish(pose_.estimate(now(), odom_timeout_sec_));
    if(auto current = pose_.latest()) {memory_->update(now(), *current);selected_path_.update(now(),*current);}
    if(!pose_.healthy(now(), odom_timeout_sec_)) {
      MemoryInput input;input.healthy = false;policy_.update(input);
      publish_stop("odometry or IMU stale");return;
    }
    if (age > observation_timeout_sec_) {
      auto basis = pose_.estimate(now(), odom_timeout_sec_);LineObservation lost;
      lost.header = basis.header;lost.projection_valid = false;lost.line_found = false;
      on_observation(lost, *pose_.latest(), basis, false);
    }
  }

  void publish_stop(const std::string & reason)
  {
    const auto t = now();
    local_path_pub_->publish(planner_->stop_path(t));
    PathStatus status;
    status.header.stamp = t;
    status.valid = false;
    status.mode.value = DriveMode::STOPPING;
    status.detail = reason;
    status_pub_->publish(status);
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::deque<LineObservation::ConstSharedPtr> pending_;
  rclcpp::Publisher<ObservedGraph>::SharedPtr memory_pub_;
  rclcpp::Subscription<line_follower_interfaces::msg::MissionState>::SharedPtr mission_sub_;
  uint8_t mission_state_{255};
  int64_t reset_stamp_{0};
  std::optional<int64_t> last_odom_stamp_;
  std::string odom_frame_;
  double memory_speed_limit_{.06};
  bool reported_transport_{false};
  double observation_timeout_sec_{0.5};
  double odom_timeout_sec_{0.5};
  std::optional<std::chrono::steady_clock::time_point> last_observation_wall_;

  SelectedPathMemory selected_path_;
  bool image_enabled_{true};
  std::optional<double> image_start_;
  Pose2D image_origin_,image_last_;double image_distance_{0.};
  MemoryPolicy policy_;
  LookTurnSession look_session_;
  BlindSearchSession blind_search_;
  double recovery_bias_{1.0};
  std::optional<double> stopping_since_;
  std::optional<double> image_lost_since_;
  std::vector<LineSegment> tentative_;
  PoseEstimator pose_;
  CandidateFeatureBuilder builder_;
  RuleBasedEvaluator rule_;
  std::unique_ptr<MlpPathEvaluator> mlp_;
  std::unique_ptr<LocalPathMapper> mapper_;
  std::unique_ptr<LocalPathMemory> memory_;
  std::unique_ptr<TrajectoryPlanner> planner_;

  rclcpp::Publisher<LocalPath>::SharedPtr local_path_pub_;
  rclcpp::Publisher<PathStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<PathCandidates>::SharedPtr candidates_pub_;
  rclcpp::Publisher<ObservedGraph>::SharedPtr graph_pub_;
  rclcpp::Publisher<LocalizationEstimate>::SharedPtr localization_pub_;
  rclcpp::Subscription<LineObservation>::SharedPtr line_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace line_follower_navigation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(line_follower_navigation::PathPlannerNode)
