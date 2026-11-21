#pragma once

#include <string>
#include <vector>

#include <rclcpp/time.hpp>

#include "line_follower_interfaces/msg/line_observation.hpp"
#include "line_follower_interfaces/msg/local_path.hpp"
#include "line_follower_interfaces/msg/path_candidates.hpp"
#include "line_follower_interfaces/msg/path_status.hpp"
#include "line_follower_navigation/candidate_feature_builder.hpp"

namespace line_follower_navigation
{

using line_follower_interfaces::msg::LocalPath;
using line_follower_interfaces::msg::PathCandidates;
using line_follower_interfaces::msg::PathPoint;
using line_follower_interfaces::msg::PathStatus;

struct PlannerParams
{
  std::string base_frame{"base_footprint"};
  double lookahead{0.3};             // [m]   TODO: measure (design §14.3)
  double max_speed{0.1};             // [m/s] TODO: measure (design §14.3)
  double min_observed_length{0.1};   // [m]
  double path_validity_sec{0.3};
};

struct PlanResult
{
  LocalPath path;
  PathStatus status;
  PathCandidates candidates;
};

// Geometry check, candidate selection, target point / speed limit / drive mode (design §7.1, §7.6).
class TrajectoryPlanner
{
public:
  explicit TrajectoryPlanner(PlannerParams params)
  : params_(std::move(params)) {}

  // Plan on the selected connected polyline; preserve observed vertices and
  // request a stopped rotation at a sharp corner. The node applies validity limits.
  PlanResult plan(
    const line_follower_interfaces::msg::LineObservation & obs,
    const std::vector<CandidateFeatures> & features,
    const std::vector<double> & rule_scores,
    const std::vector<double> & mlp_scores,   // empty when MLP is disabled/unavailable
    const rclcpp::Time & now) const;

  // Path that commands a stop (no observation, stale input, unsafe).
  LocalPath stop_path(const rclcpp::Time & now) const;

  // Invariant (design §5.2): SOURCE_UNKNOWN points are never sent to the controller.
  static std::vector<PathPoint> confirmed_only(const std::vector<PathPoint> & points);

private:
  PlannerParams params_;
};

}  // namespace line_follower_navigation
