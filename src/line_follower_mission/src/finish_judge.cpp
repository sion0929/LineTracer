#include "line_follower_mission/finish_judge.hpp"

namespace line_follower_mission
{

bool FinishJudge::update(bool marker_candidate, double traveled_distance)
{
  if (!marker_candidate || !armed(traveled_distance)) {
    return false;
  }
  return false;  // DUMMY: a single detection never ends the mission (design §9.3)
}

}  // namespace line_follower_mission
