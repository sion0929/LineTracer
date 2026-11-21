#include <gtest/gtest.h>
#include <cmath>
#include "line_follower_mission/snapshot.hpp"

TEST(Snapshot, TransformPreservesSourceAndTimes)
{
  line_follower_interfaces::msg::LocalPath path;
  path.header.frame_id = "base_footprint";
  path.points.resize(2);
  path.points[0].x = 1.;
  path.points[0].source = 0;
  path.points[1].source = 1;
  path.points[1].capture_stamp.sec = 7;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 2.; pose.position.y = 3.;
  pose.orientation.z = std::sin(M_PI/4); pose.orientation.w = std::cos(M_PI/4);
  const auto result = line_follower_mission::path_in_odom(path, pose);
  EXPECT_EQ(result.header.frame_id, "odom");
  EXPECT_NEAR(result.points[0].x, 2., 1e-6);
  EXPECT_NEAR(result.points[0].y, 4., 1e-6);
  EXPECT_EQ(result.points[1].source, 1);
  EXPECT_EQ(result.points[1].capture_stamp.sec, 7);
  path.header.frame_id = "odom";
  EXPECT_EQ(line_follower_mission::path_in_odom(path, pose).points[0].x, 1.);
}

TEST(Snapshot, HeadingIsWrapped)
{
  line_follower_interfaces::msg::LocalPath path;
  path.target_heading = 3.;
  geometry_msgs::msg::Pose pose;
  pose.orientation.z = std::sin(.5); pose.orientation.w = std::cos(.5);
  const auto out = line_follower_mission::path_in_odom(path, pose);
  EXPECT_NEAR(out.target_heading, 4.-2*M_PI, 1e-6);
  path.header.frame_id = "odom";
  path.target_heading = 4.;
  EXPECT_NEAR(line_follower_mission::path_in_odom(path, pose).target_heading, 4.-2*M_PI, 1e-6);
}

#include "line_follower_mission/freshness.hpp"
TEST(Snapshot, MissingAndStoppedSourcesBecomeStaleByWallTime)
{
  line_follower_mission::Freshness tracker;
  EXPECT_FALSE(tracker.valid("graph", 10., 1.));
  tracker.receive("graph", 10.);
  EXPECT_TRUE(tracker.valid("graph", 10.5, 1.));
  EXPECT_FALSE(tracker.valid("graph", 11.1, 1.));
  EXPECT_FALSE(tracker.valid("candidates", 10.5, 1.));
}
