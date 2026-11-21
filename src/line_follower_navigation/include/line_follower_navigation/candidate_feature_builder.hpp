#pragma once

#include <cstdint>
#include <vector>

#include "line_follower_interfaces/msg/line_observation.hpp"

namespace line_follower_navigation
{

// Per-candidate features shared by the rule-based and MLP evaluators (design §7.5).
struct CandidateFeatures
{
  uint32_t candidate_id{0};
  std::vector<uint32_t> segment_ids;
  double observed_length{0.0};   // [m]
  double heading_change{0.0};    // [rad]
  double curvature{0.0};         // [1/m]
  double quality{0.0};
  bool valid{false};
};

class CandidateFeatureBuilder
{
public:
  // DUMMY: one candidate per segment. TODO(D-08): branch legs from junctions, memory agreement,
  // projection quality, masking of missing features.
  std::vector<CandidateFeatures> build(
    const line_follower_interfaces::msg::LineObservation & obs) const;
};

}  // namespace line_follower_navigation
