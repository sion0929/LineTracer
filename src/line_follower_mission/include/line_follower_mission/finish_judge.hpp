#pragma once

namespace line_follower_mission
{

// Finish decision (design §9.3): ignore the start T, require marker + center crossing.
// DUMMY: never confirms. TODO(D-10): multi-observation of the finish T, odom-linked crossing check.
class FinishJudge
{
public:
  explicit FinishJudge(double min_travel_before_arming)
  : min_travel_(min_travel_before_arming) {}

  // Start-T suppression: candidates are ignored until the robot has traveled far enough.
  bool armed(double traveled_distance) const {return traveled_distance >= min_travel_;}
  bool update(bool marker_candidate, double traveled_distance);

private:
  double min_travel_;
};

}  // namespace line_follower_mission
