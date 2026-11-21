#include "line_follower_navigation/path_evaluator.hpp"

#include <cmath>
#include <limits>

namespace line_follower_navigation
{

std::vector<double> RuleBasedEvaluator::score(
  const std::vector<CandidateFeatures> & candidates) const
{
  std::vector<double> scores;
  scores.reserve(candidates.size());
  for (const auto & c : candidates) {
    scores.push_back(
      c.valid ? c.observed_length - 0.5 * std::abs(c.heading_change) :
      -std::numeric_limits<double>::infinity());
  }
  return scores;
}

std::vector<double> MlpPathEvaluator::score(
  const std::vector<CandidateFeatures> & candidates) const
{
  return std::vector<double>(candidates.size(), std::numeric_limits<double>::quiet_NaN());
}

}  // namespace line_follower_navigation
