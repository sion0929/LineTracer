#include <gtest/gtest.h>

#include <cmath>

#include "line_follower_control/pure_pursuit.hpp"
#include "line_follower_control/rotate_pid.hpp"
#include "line_follower_control/safety_policy.hpp"

using namespace line_follower_control;

namespace
{
SafetyInputs all_ok(double linear = 0.1, double angular = 0.2)
{
  SafetyInputs in;
  in.drive_enabled = true;
  in.mission_age = 0.0;
  in.candidate_age = 0.0;
  in.candidate_expired = false;
  in.candidate_linear = linear;
  in.candidate_angular = angular;
  in.path_valid = true;
  in.path_status_age = 0.0;
  in.perception_healthy = true;
  in.perception_age = 0.0;
  in.odom_age = 0.0;
  return in;
}

void expect_stop(const SafetyInputs & in, const char * reason)
{
  const auto d = evaluate_safety(in, SafetyLimits{}, 0.1, 0.1, 0.02);
  EXPECT_FALSE(d.approved) << reason;
  EXPECT_DOUBLE_EQ(d.linear, 0.0) << reason;
  EXPECT_DOUBLE_EQ(d.angular, 0.0) << reason;
  EXPECT_EQ(d.reason, reason);
}
}  // namespace

TEST(SafetyPolicy, DefaultInputsStop)  // nothing received yet -> zero (T-03)
{
  const auto d = evaluate_safety(SafetyInputs{}, SafetyLimits{}, 0.0, 0.0, 0.02);
  EXPECT_FALSE(d.approved);
  EXPECT_DOUBLE_EQ(d.linear, 0.0);
}

TEST(SafetyPolicy, EachGateStops)  // T-12
{
  auto in = all_ok();
  in.drive_enabled = false;
  expect_stop(in, "drive not enabled");

  in = all_ok();
  in.mission_age = 10.0;
  expect_stop(in, "mission state stale");

  in = all_ok();
  in.odom_age = 10.0;
  expect_stop(in, "odometry stale");

  in = all_ok();
  in.perception_age = 10.0;
  expect_stop(in, "perception stale");

  in = all_ok();
  in.perception_healthy = false;
  expect_stop(in, "perception unhealthy");

  in = all_ok();
  in.path_valid = false;
  expect_stop(in, "path invalid");

  in = all_ok();
  in.candidate_expired = true;
  expect_stop(in, "command candidate stale");

  in = all_ok();
  in.candidate_age = 10.0;
  expect_stop(in, "command candidate stale");

  in = all_ok(std::nan(""), 0.0);
  expect_stop(in, "command candidate not finite");
}

TEST(SafetyPolicy, ApprovedCommandIsClampedAndRateLimited)
{
  SafetyLimits lim;
  const auto d = evaluate_safety(all_ok(5.0, -9.0), lim, 0.0, 0.0, 0.02);
  EXPECT_TRUE(d.approved);
  EXPECT_NEAR(d.linear, lim.max_linear_accel * 0.02, 1e-12);
  EXPECT_NEAR(d.angular, -lim.max_angular_accel * 0.02, 1e-12);

  const auto steady = evaluate_safety(all_ok(5.0, -9.0), lim, lim.max_linear, -lim.max_angular,
      0.02);
  EXPECT_DOUBLE_EQ(steady.linear, lim.max_linear);
  EXPECT_DOUBLE_EQ(steady.angular, -lim.max_angular);
}

TEST(PurePursuit, TurnsTowardTarget)
{
  EXPECT_GT(pure_pursuit_angular(0.3, 0.1, 0.1, 1.5), 0.0);
  EXPECT_LT(pure_pursuit_angular(0.3, -0.1, 0.1, 1.5), 0.0);
  EXPECT_DOUBLE_EQ(pure_pursuit_angular(0.3, 0.0, 0.1, 1.5), 0.0);
  EXPECT_DOUBLE_EQ(pure_pursuit_angular(0.01, 0.5, 10.0, 1.5), 1.5);
}

TEST(RotatePid, SaturatesAndResets)
{
  RotatePid pid(10.0, 0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.02), 1.0);
  EXPECT_DOUBLE_EQ(pid.update(-1.0, 0.02), -1.0);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0), 0.0);
}

#include "line_follower_control/pure_pursuit.hpp"
TEST(ImagePursuit, CorrectSteeringSignsAndBoundedOutput) {
  EXPECT_LT(line_follower_control::image_pursuit_angular(.3,0,.06,.4),0.);
  EXPECT_GT(line_follower_control::image_pursuit_angular(-.3,0,.06,.4),0.);
  EXPECT_GT(line_follower_control::image_pursuit_angular(0,.3,.06,.4),0.);
  EXPECT_DOUBLE_EQ(line_follower_control::image_pursuit_angular(.3,0,0.,.4),0.);
  EXPECT_LE(std::abs(line_follower_control::image_pursuit_angular(1.,1.,.06,.4)),.4);
}

TEST(RotationProgress, RequiresForwardTravelRatherThanLateralOrBackwardMotion) {
  EXPECT_TRUE(line_follower_control::forward_progress(.04,0.,0.));
  EXPECT_FALSE(line_follower_control::forward_progress(-.04,0.,0.));
  EXPECT_FALSE(line_follower_control::forward_progress(0.,.04,0.));
  EXPECT_TRUE(line_follower_control::forward_progress(0.,.04,1.57079632679));
}
