#include <gtest/gtest.h>
#include "line_follower_navigation/memory_policy.hpp"
using namespace line_follower_navigation;
using line_follower_interfaces::msg::DriveMode;
TEST(MemoryPolicy, LossReacquisitionAndMismatch) {
  MemoryPolicy p; MemoryInput i; i.direct = true; i.remaining = .8; i.uncertainty = .008;
  EXPECT_EQ(p.update(i).mode, DriveMode::VISION_TRACKING);
  i.direct = false; EXPECT_EQ(p.update(i).mode, DriveMode::MEMORY_TRACKING);
  i.direct = true; i.agrees = true;
  EXPECT_EQ(p.update(i).mode, DriveMode::REACQUIRING);
  EXPECT_EQ(p.update(i).mode, DriveMode::REACQUIRING);
  EXPECT_EQ(p.update(i).mode, DriveMode::VISION_TRACKING);
  i.agrees = false; EXPECT_EQ(p.update(i).mode, DriveMode::REACQUIRING);
  for(int k = 0; k < 20; ++k) {
    p.update(i);
}
  EXPECT_EQ(p.update(i).mode, DriveMode::STOPPING);
}
TEST(MemoryPolicy, SpeedMonotoneAndStopsWithPositiveMargin) {
  for(bool vary_uncertainty : {false, true}) {
    MemoryPolicy p; MemoryInput i; i.remaining = .8; i.uncertainty = .008;
    double previous = 1.; bool slow = false;
    for(int k = 0; k < 100; ++k) {
      if(vary_uncertainty) {i.uncertainty = .008 + k * .0007;} else {i.remaining = .8 - k * .008;}
      auto r = p.update(i); EXPECT_LE(r.speed, previous + 1e-9);previous = r.speed;
      slow |= r.mode == DriveMode::SLOWDOWN;
    }
    EXPECT_TRUE(slow);EXPECT_EQ(p.update(i).mode, DriveMode::STOPPING);
  }
}
TEST(MemoryPolicy, UntrustedPitchAndExpiryNeverConfirm) {
  MemoryPolicy p; MemoryInput i; i.direct = true;i.remaining = .8;i.uncertainty = .008;
  p.update(i); i.pitch = .14;
  EXPECT_EQ(p.update(i).mode, DriveMode::MEMORY_TRACKING);
  i.remaining = 0.; EXPECT_EQ(p.update(i).mode, DriveMode::STOPPING);
  i.remaining = .8;i.uncertainty = .08;i.pitch = 0.;
  EXPECT_EQ(p.update(i).mode, DriveMode::STOPPING);
  i.uncertainty = .008;i.healthy = false;
  EXPECT_EQ(p.update(i).mode, DriveMode::STOPPING);
}

TEST(MemoryPolicy, GeometryHeadingAndConnectivityMustAgree) {
  using line_follower_interfaces::msg::LineSegment;
  using line_follower_interfaces::msg::PathPoint;
  LineSegment a;a.id = 1;LineSegment b;b.id = 2;
  for(int k = 0; k < 10; ++k) {
    PathPoint p;p.x = .3 + k * .01;p.y = 0.;a.points.push_back(p);p.x = .39;p.y = k * .01;
    b.points.push_back(p);
                                                                                                                       }
  std::vector<LineSegment> old = {a, b}, fresh = old;
  EXPECT_TRUE(MemoryPolicy::observations_agree(fresh, old));
  fresh[0].connected_ids = {2};fresh[1].connected_ids = {1};
  EXPECT_TRUE(MemoryPolicy::observations_agree(fresh, old));
  old = fresh;EXPECT_TRUE(MemoryPolicy::observations_agree(fresh, old));
  for(auto & seg:fresh) {
    for(auto & p:seg.points) {
      p.y += .12;
    }
}
  EXPECT_FALSE(MemoryPolicy::observations_agree(fresh, old));
  fresh = {a};for(auto & p:fresh[0].points) {
    double x = p.x;p.x = .3;p.y = x - .3;
                                                                        }
  EXPECT_FALSE(MemoryPolicy::observations_agree(fresh, {a}));
}

TEST(MemoryPolicy, PublishedPrefixNeverIncludesUncheckedOrExpiredGeometry) {
  using line_follower_interfaces::msg::PathPoint;
  std::vector<PathPoint> points(5);
  for(size_t k=0;k<points.size();++k) {
    points[k].x=.1+k*.01;points[k].source=PathPoint::SOURCE_MEMORY_UPDATED;
    points[k].uncertainty=.008;points[k].valid_until.sec=10;
  }
  points[2].uncertainty=.065;
  EXPECT_EQ(MemoryPolicy::verified_prefix(points,1000000000).size(),2u);
  points[1].source=PathPoint::SOURCE_UNKNOWN;
  EXPECT_EQ(MemoryPolicy::verified_prefix(points,1000000000).size(),1u);
  points[0].valid_until.sec=0;
  EXPECT_TRUE(MemoryPolicy::verified_prefix(points,1000000000).empty());
}

TEST(MemoryPolicy, NewlyObservedConnectionMustBePhysicallyLocal) {
  using line_follower_interfaces::msg::LineSegment;
  using line_follower_interfaces::msg::PathPoint;
  LineSegment a;a.id=1;LineSegment b;b.id=2;
  for(int k=0;k<10;++k) {PathPoint p;p.x=.3+k*.01;a.points.push_back(p);p.x=.6+k*.01;b.points.push_back(p);}
  a.connected_ids={2};b.connected_ids={1};
  EXPECT_FALSE(MemoryPolicy::observations_agree({a,b},{a,b}));
}

TEST(MemoryPolicy, ImageFallbackDoesNotDiscardUsableCurveMemory) {
  EXPECT_FALSE(MemoryPolicy::needs_image_fallback(true,.5,.045));
  EXPECT_FALSE(MemoryPolicy::needs_image_fallback(true,.2,.055));
  EXPECT_FALSE(MemoryPolicy::needs_image_fallback(true,.09,.02));
  EXPECT_TRUE(MemoryPolicy::needs_image_fallback(true,.054,.02));
  EXPECT_TRUE(MemoryPolicy::needs_image_fallback(true,.5,.06));
  EXPECT_TRUE(MemoryPolicy::needs_image_fallback(false,.5,.02));
}

TEST(MemoryPolicy, CameraGapIsNotVerifiedPathLength) {
  using line_follower_interfaces::msg::PathPoint;
  std::vector<PathPoint> points(2);points[0].x=.316;points[1].x=.356;
  EXPECT_NEAR(MemoryPolicy::observed_length(points),.04,1e-6);
  EXPECT_DOUBLE_EQ(MemoryPolicy::observed_length({}),0.);
  EXPECT_DOUBLE_EQ(MemoryPolicy::observed_length({points[0]}),0.);
}

TEST(MemoryPolicy, ShortObservedCurveCanRestoreViewByRotatingWithoutTranslation) {
  using line_follower_interfaces::msg::PathPoint;
  std::vector<PathPoint> points;
  for(auto xy : {std::pair<double,double>{.017,.019},{.037,.007},{.059,-.008},{.09,-.028},{.075,-.020}}) {
    PathPoint p;p.x=xy.first;p.y=xy.second;p.source=PathPoint::SOURCE_MEMORY_UPDATED;p.uncertainty=.03;points.push_back(p);
  }
  auto turn=MemoryPolicy::observed_tangent_turn(points);ASSERT_TRUE(turn);EXPECT_LT(*turn,-.15);EXPECT_GT(*turn,-1.2);
  for(auto & p:points) p.y=0.;EXPECT_FALSE(MemoryPolicy::observed_tangent_turn(points));
  points.clear();for(auto xy : {std::pair<double,double>{.02,0.},{.05,0.},{.08,0.},{.08,.04}}) {
    PathPoint p;p.x=xy.first;p.y=xy.second;p.source=PathPoint::SOURCE_MEMORY_UPDATED;p.uncertainty=.03;points.push_back(p);
  }
  EXPECT_FALSE(MemoryPolicy::observed_tangent_turn(points));
  points[0].source=PathPoint::SOURCE_UNKNOWN;EXPECT_FALSE(MemoryPolicy::observed_tangent_turn(points));
}

TEST(MemoryPolicy, RejectedMetricMemoryMayUseVerifiedImageRatherThanRemainLatched) {
  EXPECT_TRUE(MemoryPolicy::needs_image_fallback(true,.5,.03,true));
  EXPECT_FALSE(MemoryPolicy::needs_image_fallback(true,.5,.03,false));
}
TEST(LookTurnSession, FinishesOneObservedTargetThenRequiresMeasuredProgress) {
  LookTurnSession s;
  ASSERT_TRUE(s.update(0.,0.,0.,0.,.6,true));
  auto pending=s.update(.5,0.,0.,.3,std::nullopt,true);ASSERT_TRUE(pending);EXPECT_NEAR(*pending,.3,1e-8);
  EXPECT_FALSE(s.update(1.,0.,0.,.55,.6,true));
  EXPECT_FALSE(s.update(1.1,0.,0.,0.,.6,true));
  EXPECT_TRUE(s.update(2.,.06,0.,0.,.6,true));
  EXPECT_FALSE(s.update(9.,.06,0.,0.,.6,true));EXPECT_TRUE(s.timed_out());
  EXPECT_FALSE(s.update(10.,.06,0.,0.,.6,true));
  EXPECT_TRUE(s.update(10.5,.06,0.,0.,-.6,true));
}

TEST(MemoryPolicy, DistantUncertaintyDoesNotEraseVerifiedNearControlPrefix) {
  using line_follower_interfaces::msg::PathPoint;
  std::vector<PathPoint> points;
  for(int k=0;k<80;++k) {PathPoint p;p.x=.01+k*.01;p.source=PathPoint::SOURCE_MEMORY_UPDATED;p.uncertainty=k<40?.01:.059;p.valid_until.sec=20;points.push_back(p);}
  EXPECT_NEAR(MemoryPolicy::control_uncertainty(points),.01,1e-6);
  points[50].uncertainty=.061;auto prefix=MemoryPolicy::verified_prefix(points,1000000000);
  EXPECT_EQ(prefix.size(),50u);EXPECT_NEAR(MemoryPolicy::control_uncertainty(prefix),.01,1e-6);
  points.erase(points.begin(),points.begin()+40);
  EXPECT_GT(MemoryPolicy::control_uncertainty(points),.05);
}

TEST(BlindSearchSession, SweepsBiasSideThenBackThenOppositeSideThenExhausts) {
  BlindSearchSession s;
  // Leg 1: toward the bias side (+1 => left), bounded to ~60 deg (kept well short
  // of the suggested ~120 deg since an uncommitted rotate episode cannot be
  // cancelled early even after the line is reacquired mid-turn).
  auto leg1 = s.update(0., 0., 1., true);
  ASSERT_TRUE(leg1);
  EXPECT_NEAR(*leg1, 1.0472, 1e-3);
  EXPECT_EQ(s.leg(), 1);
  // Mid-leg: still turning, same goal, never exhausted or translating.
  auto mid = s.update(1., 0.5, 1., true);
  ASSERT_TRUE(mid);
  EXPECT_FALSE(s.exhausted());
  // Reaches leg-1 goal without reacquiring: advances to leg 2 (back to start heading).
  auto leg2 = s.update(2., 1.0472, 1., true);
  ASSERT_TRUE(leg2);
  EXPECT_EQ(s.leg(), 2);
  EXPECT_NEAR(*leg2, -1.0472, 1e-3);
  // Reaches start heading again: advances to leg 3 (opposite side).
  auto leg3 = s.update(3., 0., 1., true);
  ASSERT_TRUE(leg3);
  EXPECT_EQ(s.leg(), 3);
  EXPECT_NEAR(*leg3, -1.0472, 1e-3);
  // Reaches leg-3 goal without reacquiring anywhere: bounded search gives up.
  auto done = s.update(4., -1.0472, 1., true);
  EXPECT_FALSE(done);
  EXPECT_TRUE(s.exhausted());
  // Once exhausted it never resumes on its own, only via reset().
  EXPECT_FALSE(s.update(5., -1.0472, 1., true));
  s.reset();
  EXPECT_FALSE(s.exhausted());
  EXPECT_TRUE(s.update(5., 0., 1., true));
}

TEST(BlindSearchSession, StopsAsSoonAsTriggerClearsAndNeverExceedsOverallBudget) {
  BlindSearchSession s;
  ASSERT_TRUE(s.update(0., 0., -1., true));
  EXPECT_EQ(s.leg(), 1);
  // Line reacquired (trigger false): cancel immediately, no latched state.
  EXPECT_FALSE(s.update(0.2, -0.5, -1., false));
  EXPECT_FALSE(s.exhausted());
  // A fresh trigger afterwards starts a brand new bounded search.
  ASSERT_TRUE(s.update(0.3, 0., -1., true));
  EXPECT_EQ(s.leg(), 1);
  EXPECT_EQ(s.attempts(), 1);
  // An overall-budget timeout (stuck mid-leg) also exhausts the search.
  BlindSearchSession slow;
  ASSERT_TRUE(slow.update(100., 0., 1., true));
  auto still = slow.update(131., 0.5, 1., true);  // > 30 s total budget, never reached a goal
  EXPECT_FALSE(still);
  EXPECT_TRUE(slow.exhausted());
}

TEST(MemoryPolicy, TwoCoherentObservedPointsMayOrientCameraWithoutDriving) {
  using line_follower_interfaces::msg::PathPoint;
  PathPoint a,b;a.x=.01;a.y=.01;b.x=.034;b.y=-.005;
  a.source=b.source=PathPoint::SOURCE_MEMORY_UPDATED;a.segment_id=b.segment_id=8;a.uncertainty=b.uncertainty=.04;
  auto turn=MemoryPolicy::observed_tangent_turn({a,b});ASSERT_TRUE(turn);EXPECT_LT(*turn,-.15);EXPECT_GT(*turn,-1.2);
  b.source=PathPoint::SOURCE_UNKNOWN;EXPECT_FALSE(MemoryPolicy::observed_tangent_turn({a,b}));
  b.source=PathPoint::SOURCE_MEMORY_UPDATED;b.segment_id=9;EXPECT_FALSE(MemoryPolicy::observed_tangent_turn({a,b}));
}