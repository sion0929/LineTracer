#pragma once

#include <string>
#include <vector>

#include "line_follower_navigation/candidate_feature_builder.hpp"

namespace line_follower_navigation
{

// Relative ranking score per candidate (higher is better). Not a probability.
class PathEvaluator
{
public:
  virtual ~PathEvaluator() = default;
  virtual bool available() const = 0;
  virtual std::vector<double> score(const std::vector<CandidateFeatures> & candidates) const = 0;
};

// Always-available fallback.
// DUMMY: longer and straighter is better. TODO(D-08): continuity, curvature, expected traverse time.
class RuleBasedEvaluator : public PathEvaluator
{
public:
  bool available() const override {return true;}
  std::vector<double> score(const std::vector<CandidateFeatures> & candidates) const override;
};

// features -> 64 -> 32 -> score via ONNX Runtime CPU.
// STUB: never available, so path_planner uses RuleBasedEvaluator. TODO(D-15).
class MlpPathEvaluator : public PathEvaluator
{
public:
  explicit MlpPathEvaluator(std::string model_path)
  : model_path_(std::move(model_path)) {}
  bool available() const override {return false;}
  std::vector<double> score(const std::vector<CandidateFeatures> & candidates) const override;

private:
  std::string model_path_;
};

}  // namespace line_follower_navigation
