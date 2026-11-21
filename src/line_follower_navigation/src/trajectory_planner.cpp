#include "line_follower_navigation/trajectory_planner.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "line_follower_interfaces/msg/drive_mode.hpp"
#include "line_follower_navigation/memory_policy.hpp"

namespace line_follower_navigation
{

using line_follower_interfaces::msg::DriveMode;
using line_follower_interfaces::msg::PathCandidate;

std::vector<PathPoint> TrajectoryPlanner::confirmed_only(const std::vector<PathPoint> & points)
{
  std::vector<PathPoint> out;
  for (const auto & p : points) {
    if (p.source != PathPoint::SOURCE_UNKNOWN) {
      out.push_back(p);
    }
  }
  return out;
}

LocalPath TrajectoryPlanner::stop_path(const rclcpp::Time & now) const
{
  LocalPath path;
  path.header.stamp = now;
  path.header.frame_id = params_.base_frame;
  path.speed_limit = 0.0f;
  path.mode.value = DriveMode::STOPPING;
  path.valid_until = now + rclcpp::Duration::from_seconds(params_.path_validity_sec);
  return path;
}

PlanResult TrajectoryPlanner::plan(
  const line_follower_interfaces::msg::LineObservation & obs,
  const std::vector<CandidateFeatures> & features,
  const std::vector<double> & rule_scores,
  const std::vector<double> & mlp_scores,
  const rclcpp::Time & now) const
{
  PlanResult r;
  r.candidates.header.stamp = obs.header.stamp;
  r.candidates.header.frame_id = params_.base_frame;
  r.status.header.stamp = now;
  r.status.evaluator = mlp_scores.empty() ?
    PathStatus::EVALUATOR_RULE_BASED : PathStatus::EVALUATOR_MLP_ASSISTED;
  r.status.mlp_healthy = !mlp_scores.empty();

  // Geometry gate first, then rule scores (design §7.4-4/5).
  int best = -1;
  double best_score = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < features.size(); ++i) {
    PathCandidate c;
    c.candidate_id = features[i].candidate_id;
    c.segment_ids = features[i].segment_ids;
    c.observed_length = static_cast<float>(features[i].observed_length);
    c.heading_change = static_cast<float>(features[i].heading_change);
    c.curvature = static_cast<float>(features[i].curvature);
    c.rule_score = static_cast<float>(rule_scores.at(i));
    c.mlp_score = mlp_scores.empty() ?
      std::numeric_limits<float>::quiet_NaN() : static_cast<float>(mlp_scores.at(i));
    bool short_near_memory=false;
    if(!features[i].segment_ids.empty()) {
      const auto seg=std::find_if(obs.segments.begin(),obs.segments.end(),[&](const auto & s){return s.id==features[i].segment_ids.front();});
      if(seg!=obs.segments.end() && !seg->points.empty()) {
        const auto & first=seg->points.front();
        short_near_memory=first.source==PathPoint::SOURCE_MEMORY_UPDATED &&
          std::hypot(first.x,first.y)<.08 && first.x>=-.02 && features[i].observed_length>.055 &&
          MemoryPolicy::verified_prefix(seg->points,now.nanoseconds()).size()==seg->points.size();
      }
    }
    c.geometry_feasible = features[i].valid &&
      (features[i].observed_length>=params_.min_observed_length || short_near_memory);
    if (c.geometry_feasible && rule_scores[i] > best_score) {
      best_score = rule_scores[i];
      best = static_cast<int>(i);
    }
    r.candidates.candidates.push_back(c);
  }

  if (!obs.line_found || !obs.projection_valid || best < 0) {
    r.path = stop_path(now);
    r.status.valid = false;
    r.status.mode.value = DriveMode::STOPPING;
    r.status.detail = !obs.projection_valid ? "ground projection untrusted" :
      (obs.line_found ? "no feasible candidate" : "no line observed");
    return r;
  }

  auto & chosen = r.candidates.candidates[best];
  chosen.selected = true;
  chosen.reason = "highest feasible observed-polyline rule score";

  const uint32_t seg_id = features[best].segment_ids.front();
  for (const auto & seg : obs.segments) {
    if (seg.id == seg_id) {
      r.path.points = MemoryPolicy::without_short_backtracking(confirmed_only(seg.points));
      break;
    }
  }
  if (r.path.points.empty()) {
    r.path = stop_path(now);
    r.status.valid = false;
    r.status.mode.value = DriveMode::STOPPING;
    r.status.detail = "selected segment has no confirmed points";
    return r;
  }

  const PathPoint * target = &r.path.points.back();
  for (const auto & p : r.path.points) {
    if (std::hypot(p.x, p.y) >= params_.lookahead) {
      target = &p;
      break;
    }
  }

  r.path.header.stamp = obs.header.stamp;
  r.path.header.frame_id = params_.base_frame;
  r.path.target_point.x = target->x;
  r.path.target_point.y = target->y;
  r.path.target_heading = static_cast<float>(std::atan2(target->y, target->x));
  r.path.speed_limit = static_cast<float>(params_.max_speed);
  r.path.mode.value = DriveMode::VISION_TRACKING;
  r.path.valid_until = now + rclcpp::Duration::from_seconds(params_.path_validity_sec);

  double strongest=1.05;size_t vertex=0,after_vertex=0;
  for(size_t k=1;k+1<r.path.points.size();++k) {
    size_t i=k-1,j=k+1;
    while(i>0 && std::hypot(r.path.points[k].x-r.path.points[i].x,r.path.points[k].y-r.path.points[i].y)<.06) --i;
    while(j+1<r.path.points.size() && std::hypot(r.path.points[j].x-r.path.points[k].x,r.path.points[j].y-r.path.points[k].y)<.06) ++j;
    const auto & a=r.path.points[i];const auto & b=r.path.points[k];const auto & c=r.path.points[j];
    const double l1=std::hypot(b.x-a.x,b.y-a.y),l2=std::hypot(c.x-b.x,c.y-b.y);
    if(l1<.012 || l2<.04) continue;
    const double angle=std::abs(std::atan2((b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x),(b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y)));
    if(angle>strongest) {strongest=angle;vertex=k;after_vertex=j;}
  }
  if(vertex) {
    const auto & corner=r.path.points[vertex];const auto & outgoing=r.path.points[after_vertex];
    const double distance=std::hypot(corner.x,corner.y);
    if(distance<.45) {
      r.path.target_point.x=corner.x;r.path.target_point.y=corner.y;
      r.path.mode.value=DriveMode::SLOWDOWN;
      r.path.speed_limit=std::min(r.path.speed_limit,float(std::min(.05,std::sqrt(.3*std::max(0.,distance-.025)))));
      if(distance<.035) {
        r.path.mode.value=DriveMode::ROTATE_IN_PLACE;r.path.speed_limit=0.;
        r.path.target_heading=std::atan2(outgoing.y-corner.y,outgoing.x-corner.x);
      }
    }
  }
  const double curvature=2*std::abs(r.path.target_point.y)/std::max(.01,r.path.target_point.x*r.path.target_point.x+r.path.target_point.y*r.path.target_point.y);
  r.path.speed_limit=std::min(r.path.speed_limit,float(.45/std::max(.45,curvature)));

  r.status.valid = true;
  r.status.mode = r.path.mode;
  r.status.verified_remaining_distance = static_cast<float>(features[best].observed_length);
  return r;
}

}  // namespace line_follower_navigation
