#pragma once

#include <string>

#include "line_follower_interfaces/msg/line_observation.hpp"
#include "line_follower_interfaces/msg/observed_graph.hpp"
#include "line_follower_navigation/pose_estimator.hpp"

namespace line_follower_navigation
{

// Builds the odom-frame graph of segments the camera actually observed (design §7.1, §7.2).
class LocalPathMapper
{
public:
  explicit LocalPathMapper(std::string odom_frame = "odom")
  : odom_frame_(std::move(odom_frame)) {}

  // TODO(D-06): merge with previous graph, keep connectivity across frames.
  line_follower_interfaces::msg::ObservedGraph map(
    const line_follower_interfaces::msg::LineObservation & obs,
    const Pose2D & pose_at_capture) const;

private:
  std::string odom_frame_;
};

}  // namespace line_follower_navigation
