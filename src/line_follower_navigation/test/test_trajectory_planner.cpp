#include <gtest/gtest.h>

#include <cmath>

#include "line_follower_interfaces/msg/drive_mode.hpp"
#include "line_follower_navigation/candidate_feature_builder.hpp"
#include "line_follower_navigation/local_path_memory.hpp"
#include "line_follower_navigation/local_path_mapper.hpp"
#include "line_follower_navigation/path_evaluator.hpp"
#include "line_follower_navigation/trajectory_planner.hpp"

using line_follower_interfaces::msg::DriveMode;
using line_follower_interfaces::msg::LineObservation;
using line_follower_interfaces::msg::LineSegment;
using namespace line_follower_navigation;

namespace
{
LineObservation straight_line(uint8_t source = PathPoint::SOURCE_OBSERVED)
{
  LineObservation obs;
  obs.line_found = true;
  obs.projection_valid = true;
  LineSegment seg;
  seg.id = 7;
  for (int i = 0; i <= 10; ++i) {
    PathPoint p;
    p.x = 0.1 + 0.05 * i;
    p.source = source;
    seg.points.push_back(p);
  }
  obs.segments.push_back(seg);
  return obs;
}

PlanResult run(const LineObservation & obs)
{
  TrajectoryPlanner planner(PlannerParams{});
  CandidateFeatureBuilder builder;
  RuleBasedEvaluator rule;
  const auto f = builder.build(obs);
  return planner.plan(obs, f, rule.score(f), {}, rclcpp::Time(0, 0));
}
}  // namespace

TEST(TrajectoryPlanner, ConfirmedOnlyDropsUnknown)
{
  std::vector<PathPoint> pts(3);
  pts[1].source = PathPoint::SOURCE_UNKNOWN;
  EXPECT_EQ(TrajectoryPlanner::confirmed_only(pts).size(), 2u);
}

TEST(TrajectoryPlanner, NoLineMeansStop)
{
  const auto r = run(LineObservation{});
  EXPECT_FALSE(r.status.valid);
  EXPECT_EQ(r.path.mode.value, DriveMode::STOPPING);
  EXPECT_FLOAT_EQ(r.path.speed_limit, 0.0f);
}

TEST(TrajectoryPlanner, UnknownOnlySegmentMeansStop)
{
  const auto r = run(straight_line(PathPoint::SOURCE_UNKNOWN));
  EXPECT_FALSE(r.status.valid);
  EXPECT_TRUE(r.path.points.empty());
}

TEST(TrajectoryPlanner, StraightLineIsTrackedAhead)
{
  const auto r = run(straight_line());
  ASSERT_TRUE(r.status.valid);
  EXPECT_EQ(r.path.mode.value, DriveMode::VISION_TRACKING);
  EXPECT_GT(r.path.target_point.x, 0.0);
  EXPECT_EQ(r.status.evaluator, PathStatus::EVALUATOR_RULE_BASED);
  ASSERT_EQ(r.candidates.candidates.size(), 1u);
  EXPECT_TRUE(r.candidates.candidates[0].selected);
  EXPECT_TRUE(std::isnan(r.candidates.candidates[0].mlp_score));
}

TEST(LocalPathMemory, ProjectsStoredPointsAsMemoryUpdated)
{
  LocalPathMemory memory(3.0);
  memory.store(straight_line().segments);
  const auto pts = memory.project_to_base(Pose2D{0.1, 0.0, 0.0});
  ASSERT_FALSE(pts.empty());
  EXPECT_EQ(pts.front().source, PathPoint::SOURCE_MEMORY_UPDATED);
  EXPECT_NEAR(pts.front().x, 0.0, 1e-9);
}

TEST(TrajectoryPlanner, UntrustedProjectionNeverBecomesDrivePath)
{
  auto obs = straight_line();
  obs.projection_valid = false;
  const auto r = run(obs);
  EXPECT_FALSE(r.status.valid);
  EXPECT_TRUE(r.path.points.empty());
  EXPECT_EQ(r.path.mode.value, DriveMode::STOPPING);
}

TEST(LocalPathMapper, UntrustedProjectionNeverEntersObservedMemory)
{
  auto obs = straight_line();
  obs.projection_valid = false;
  LocalPathMapper mapper("odom");
  EXPECT_TRUE(mapper.map(obs, Pose2D{}).segments.empty());
}

TEST(TrajectoryPlanner, SharpObservedCornerStopsThenRotatesToOutgoing) {
  auto o=straight_line();auto & pts=o.segments[0].points;pts.clear();
  for(int k=0;k<=3;++k) {PathPoint p;p.x=k*.01;pts.push_back(p);}
  for(int k=1;k<=30;++k) {PathPoint p;p.x=.03;p.y=k*.01;pts.push_back(p);}
  auto r=run(o);EXPECT_EQ(r.path.mode.value,DriveMode::ROTATE_IN_PLACE);EXPECT_FLOAT_EQ(r.path.speed_limit,0.);
  EXPECT_NEAR(r.path.target_heading,M_PI/2,.1);
}

TEST(TrajectoryPlanner, SmoothTightArcIsNotASharpCorner) {
  auto o=straight_line();auto & pts=o.segments[0].points;pts.clear();
  for(int k=0;k<=80;++k) {double a=k*M_PI/160.;PathPoint p;p.x=.11*std::sin(a);p.y=.11*(1-std::cos(a));pts.push_back(p);}
  for(int k=1;k<=20;++k) {PathPoint p;p.x=.11;p.y=.11+k*.01;pts.push_back(p);}
  auto r=run(o);ASSERT_TRUE(r.status.valid);EXPECT_EQ(r.path.mode.value,DriveMode::VISION_TRACKING);
  EXPECT_GT(r.path.target_point.y,.04);
}

TEST(TrajectoryPlanner, ShortVerifiedNearMemoryRemainsFeasibleBeyondBrakingReserve) {
  auto o=straight_line(PathPoint::SOURCE_MEMORY_UPDATED);auto & pts=o.segments[0].points;pts.clear();
  for(double x:{.01,.03,.06,.09}) {PathPoint p;p.x=x;p.source=PathPoint::SOURCE_MEMORY_UPDATED;p.uncertainty=.02;p.valid_until.sec=10;pts.push_back(p);}
  auto r=run(o);EXPECT_TRUE(r.status.valid);EXPECT_GT(r.path.speed_limit,0.);
  for(auto & p:pts) p.x+=.32;
  EXPECT_FALSE(run(o).status.valid);
}

TEST(TrajectoryPlanner, TinyBacktrackingSpurDoesNotRotateOnObservedCurve) {
  auto o=straight_line();auto & pts=o.segments[0].points;pts.clear();
  for(auto xy : {std::pair<double,double>{.008,-.005},{.025,.001},{.033,-.009},{.032,.011}}) {
    PathPoint p;p.x=xy.first;p.y=xy.second;p.segment_id=2;pts.push_back(p);
  }
  for(int k=0;k<30;++k) {PathPoint p;p.x=.040+k*.005;p.y=.023+k*.013;p.segment_id=2;pts.push_back(p);}
  auto r=run(o);ASSERT_TRUE(r.status.valid);EXPECT_NE(r.path.mode.value,DriveMode::ROTATE_IN_PLACE);
  EXPECT_GT(r.path.speed_limit,0.);
}
