#include "line_follower_navigation/candidate_feature_builder.hpp"

#include <cmath>

namespace line_follower_navigation
{

std::vector<CandidateFeatures> CandidateFeatureBuilder::build(
  const line_follower_interfaces::msg::LineObservation & obs) const
{
  std::vector<CandidateFeatures> out;
  if (!obs.line_found) {
    return out;
  }
  uint32_t next_id = 0;
  for (const auto & seg : obs.segments) {
    CandidateFeatures f;
    f.candidate_id = next_id++;
    f.segment_ids = {seg.id};
    f.quality = seg.confidence;
    if (seg.points.size() >= 2) {
      for (std::size_t i = 1; i < seg.points.size(); ++i) {
        f.observed_length += std::hypot(
          seg.points[i].x - seg.points[i - 1].x, seg.points[i].y - seg.points[i - 1].y);
      }
      const auto & a = seg.points.front();
      const auto & b = seg.points.back();
      f.heading_change = std::atan2(b.y - a.y, b.x - a.x);
      f.valid = true;
    }
    out.push_back(f);
  }
  return out;
}

}  // namespace line_follower_navigation
