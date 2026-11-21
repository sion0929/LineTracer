#pragma once
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/time.hpp>
#include "line_follower_interfaces/msg/line_segment.hpp"
#include "line_follower_interfaces/msg/path_point.hpp"
#include "line_follower_navigation/pose_estimator.hpp"

namespace line_follower_navigation
{
using line_follower_interfaces::msg::LineSegment;
using line_follower_interfaces::msg::PathPoint;

// Bounded odom geometry: only actual observations may extend the graph.
class LocalPathMemory
{
public:
  explicit LocalPathMemory(
    double retention_sec = 10., std::size_t max_points = 1200,
    std::size_t max_segments = 32)
  : retention_sec_(retention_sec), max_points_(max_points), max_segments_(max_segments) {}
  struct BranchDecision {uint32_t id;double length,heading,score;bool feasible,selected;std::string reason;};
  const std::vector<BranchDecision> & decisions() const {return decisions_;}
  void reset();
  bool set_run_id(const std::string & id);
  std::unordered_map<uint32_t, uint32_t> store(const std::vector<LineSegment> & odom_segments);
  void update(const rclcpp::Time & now, const Pose2D & current);
  void prune(const rclcpp::Time & now);
  const std::vector<LineSegment> & segments() const {return segments_;}
  std::vector<PathPoint> project_to_base(const Pose2D & current) const;
  // Nearest forward chain follows recorded endpoint connections.
  // Branches use continuity/feasibility rules and retain the selected leg.
  std::vector<PathPoint> path_to_base(const Pose2D & current, const rclcpp::Time & capture) const;

private:
  void bound();
  mutable std::vector<BranchDecision> decisions_;
  mutable std::unordered_map<uint32_t,uint32_t> locks_;
  double retention_sec_;
  std::size_t max_points_, max_segments_;
  uint32_t next_id_{1};
  std::string run_id_;
  std::optional<int64_t> updated_;
  std::optional<Pose2D> current_;
  std::vector<LineSegment> segments_;
};
}  // namespace line_follower_navigation
