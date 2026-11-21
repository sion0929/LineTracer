#include "line_follower_navigation/local_path_mapper.hpp"

#include <cmath>
#include <algorithm>
#include <set>

namespace line_follower_navigation
{

line_follower_interfaces::msg::ObservedGraph LocalPathMapper::map(
  const line_follower_interfaces::msg::LineObservation & obs,
  const Pose2D & pose_at_capture) const
{
  line_follower_interfaces::msg::ObservedGraph graph;
  graph.header.stamp = obs.header.stamp;
  graph.header.frame_id = odom_frame_;

  if (!obs.projection_valid) {return graph;}

  const double c = std::cos(pose_at_capture.yaw);
  const double s = std::sin(pose_at_capture.yaw);
  auto to_odom = [&](double bx, double by, double & ox, double & oy) {
      ox = pose_at_capture.x + c * bx - s * by;
      oy = pose_at_capture.y + s * bx + c * by;
    };

  std::set<uint32_t> ids;
  for (const auto & seg:obs.segments) {
    if (seg.points.size() < 2 || std::any_of(seg.points.begin(), seg.points.end(),
      [](const auto & p) {
        return p.source != line_follower_interfaces::msg::PathPoint::SOURCE_OBSERVED ||
               !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.confidence) ||
               !std::isfinite(p.uncertainty);
      })) {continue;}
    graph.segments.push_back(seg);ids.insert(seg.id);
  }
  for (auto & seg : graph.segments) {
    for (auto & p : seg.points) {
      to_odom(p.x, p.y, p.x, p.y);
      p.capture_stamp = obs.header.stamp;
      p.valid_until = rclcpp::Time(obs.header.stamp) + rclcpp::Duration::from_seconds(.5);
    }
    seg.connected_ids.erase(std::remove_if(seg.connected_ids.begin(), seg.connected_ids.end(),
      [&](auto id){return !ids.count(id);}), seg.connected_ids.end());
  }
  for (auto j:obs.junctions) {
    j.segment_ids.erase(std::remove_if(j.segment_ids.begin(), j.segment_ids.end(),
      [&](auto id){return !ids.count(id);}), j.segment_ids.end());
    if (j.segment_ids.size() >= 2) {graph.junctions.push_back(j);}
  }
  for (auto & j : graph.junctions) {
    to_odom(j.x, j.y, j.x, j.y);
  }
  return graph;
}

}  // namespace line_follower_navigation
