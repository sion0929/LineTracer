#include <gtest/gtest.h>
#include <cmath>
#include "line_follower_navigation/pose_estimator.hpp"
#include "line_follower_navigation/local_path_mapper.hpp"
#include "line_follower_navigation/local_path_memory.hpp"
using namespace line_follower_navigation;
using line_follower_interfaces::msg::LineObservation;
namespace
{
nav_msgs::msg::Odometry odom(double t, double x, double y, double yaw)
{
  nav_msgs::msg::Odometry m; m.header.stamp = rclcpp::Time(int64_t(t * 1e9));
  m.header.frame_id = "odom";m.pose.pose.position.x = x;m.pose.pose.position.y = y;
  m.pose.pose.orientation.z = std::sin(yaw / 2);m.pose.pose.orientation.w = std::cos(yaw / 2);
  return m;
}
LineSegment segment(int id, double t, double x0 = .32, double x1 = 1., double y = 0.)
{
  LineSegment s;s.id = id;s.confidence = .9;
  for(double x = x0; x <= x1 + .0001; x += .01) {
    PathPoint p;p.x = x;p.y = y;p.confidence = .9;p.uncertainty = .005;
    p.source = PathPoint::SOURCE_OBSERVED;p.segment_id = id;
    p.capture_stamp = rclcpp::Time(int64_t(t * 1e9));s.points.push_back(p);
  }
  return s;
}
size_t count(const LocalPathMemory & m)
{
  size_t n = 0;for(const auto & s:m.segments()) {
    n += s.points.size();
  }
  return n;
}
}
TEST(CapturePose, InterpolatesPositionAndShortestRotationRejectsOutsideAndOld) {
 PoseEstimator p;p.add_odom(odom(1., 0, 0, 170 * M_PI / 180));
    p.add_odom(odom(2., 2, 4, -170 * M_PI / 180));
 auto mid = p.pose_at(rclcpp::Time(1, 500000000), 2.);ASSERT_TRUE(mid);
 EXPECT_NEAR(mid->x, 1, 1e-8);EXPECT_NEAR(mid->y, 2, 1e-8);
    EXPECT_NEAR(std::abs(mid->yaw), M_PI, 1e-8);
 EXPECT_FALSE(p.pose_at(rclcpp::Time(0, 900000000), 2.));
    EXPECT_FALSE(p.pose_at(rclcpp::Time(2, 100000000), 2.));
 EXPECT_FALSE(p.pose_at(rclcpp::Time(1, 0), .5));
}
TEST(CapturePose, ImuAndOdomMustBothBeFresh) {
 PoseEstimator p;p.add_odom(odom(1, 0, 0, 0));EXPECT_FALSE(p.healthy(rclcpp::Time(1, 0), .5));
 sensor_msgs::msg::Imu imu;imu.header.stamp = rclcpp::Time(1, 0);imu.orientation.w = 1;
    p.add_imu(imu);
 EXPECT_TRUE(p.healthy(rclcpp::Time(1, 100000000), .5));p.add_odom(odom(2, 0, 0, 0));
 EXPECT_FALSE(p.healthy(rclcpp::Time(2, 0), .5));
}
TEST(Mapper, KnownRotationAndOnlyDirectObservedGeometry) {
 LineObservation o;o.header.stamp = rclcpp::Time(1, 0);o.projection_valid = true;
    o.line_found = true;
 o.segments = {segment(1, 1, .3, .4), segment(2, 1)};o.segments[0].connected_ids = {2};
 o.segments[1].points[0].source = PathPoint::SOURCE_UNKNOWN;
 LocalPathMapper mapper("odom");auto g = mapper.map(o, Pose2D{1, 2, M_PI / 2});
 ASSERT_EQ(g.segments.size(), 1u);EXPECT_EQ(g.header.frame_id, "odom");
 EXPECT_NEAR(g.segments[0].points[0].x, 1., 1e-8);EXPECT_NEAR(g.segments[0].points[0].y, 2.3, 1e-8);
 EXPECT_TRUE(g.segments[0].connected_ids.empty());
    EXPECT_EQ(g.segments[0].points[0].capture_stamp, o.header.stamp);
}
TEST(Memory, RepeatedViewsDeduplicateAndStayBounded) {
 LocalPathMemory m(10., 150, 8);m.update(rclcpp::Time(1, 0), Pose2D{});
 for(int k = 0; k < 500; ++k) {
    double t = 1 + k * .002;m.update(rclcpp::Time(int64_t(t * 1e9)), Pose2D{});
    m.store({segment(k, t, .32, 1., (k % 3 - 1) * .002)});
                                             }
 EXPECT_EQ(m.segments().size(), 1u);EXPECT_LE(count(m), 100u);
}
TEST(Memory, AgeDistanceUncertaintyExpiryAndPassedGeometry) {
 LocalPathMemory m(2.);m.update(rclcpp::Time(1, 0), Pose2D{});m.store({segment(1, 1)});
 m.update(rclcpp::Time(2, 0), Pose2D{.1, 0, 0});ASSERT_FALSE(m.segments().empty());
 EXPECT_GT(m.segments()[0].points[0].uncertainty, .005);
    EXPECT_GT(m.segments()[0].points[0].valid_until.sec, 1);
 m.update(rclcpp::Time(3, 100000000), Pose2D{.1, 0, 0});EXPECT_TRUE(m.segments().empty());
 m.store({segment(1, 4)});m.update(rclcpp::Time(4, 0), Pose2D{3, 0, 0});
    EXPECT_TRUE(m.segments().empty());
}
TEST(Memory, MissionAndFaultResetAndCapacity) {
 LocalPathMemory m(10., 40, 4);m.set_run_id("a");m.store({segment(1, 1)});m.set_run_id("a");
    EXPECT_GT(count(m), 0u);
 m.set_run_id("b");EXPECT_EQ(count(m), 0u);
 for(int k = 0; k < 30; ++k) {
    m.store({segment(k, 1, .32 + k * .15, .4 + k * .15, k * .2)});
}
 EXPECT_LE(count(m), 40u);EXPECT_LE(m.segments().size(), 4u);m.reset();EXPECT_EQ(count(m), 0u);
}
TEST(Memory, BlindZoneAndCornerUseOnlyRetainedObservationsInCurrentBase) {
 LocalPathMemory m(10.);m.update(rclcpp::Time(1, 0), Pose2D{});
 auto a = segment(1, 1, .1, .6);auto b = segment(2, 1, 0., .25);
 for(auto & p:b.points) {
    p.y = p.x;p.x = .6;
                                       }
                                        a.connected_ids = {2};b.connected_ids = {1};m.store({a, b});
 m.update(rclcpp::Time(2, 0), Pose2D{.38, 0, 0});
 auto path = m.path_to_base(Pose2D{.38, 0, 0}, rclcpp::Time(2, 0));
 bool vertex = false, outgoing = false;for(const auto & p:path) {
    EXPECT_EQ(p.source, PathPoint::SOURCE_MEMORY_UPDATED);
    EXPECT_EQ(rclcpp::Time(p.capture_stamp).nanoseconds(), 1000000000LL);
    vertex |= std::hypot(p.x - .22, p.y) < .015;outgoing |= std::abs(p.x - .22) < .015 && p.y > .15;
                                                                                                                                                    }
 EXPECT_TRUE(vertex);EXPECT_TRUE(outgoing);
 auto corner = std::min_element(path.begin(), path.end(), [](const auto & a, const auto & b){
        return std::hypot(a.x - .22, a.y) < std::hypot(b.x - .22, b.y);
                                                                                                                                                            });
 const auto k = size_t(corner - path.begin());ASSERT_GE(k, 2u);ASSERT_LT(k + 2, path.size());
 const double ax = path[k].x - path[k - 2].x, ay = path[k].y - path[k - 2].y;
 const double bx = path[k + 2].x - path[k].x, by = path[k + 2].y - path[k].y;
 EXPECT_NEAR((ax * bx + ay * by) / (std::hypot(ax, ay) * std::hypot(bx, by)), 0., .01);
 auto unknown = segment(3, 2);unknown.points[2].source = PathPoint::SOURCE_UNKNOWN;
 auto before = count(m);m.store({unknown});EXPECT_EQ(count(m), before);
}

TEST(Memory, MovingObservationWindowDoesNotAdvectOldGeometry)
{
 LocalPathMemory m(10.);m.update(rclcpp::Time(1, 0), Pose2D{});m.store({segment(1, 1)});
 for(int k = 1; k <= 100; ++k) {
    const double x = k * .002, t = 1. + k * .03;
    m.update(rclcpp::Time(int64_t(t * 1e9)), Pose2D{x, 0, 0});
    m.store({segment(k, t, .32 + x, 1. + x)});
 }
 ASSERT_FALSE(m.segments().empty());
 double nearest = 10.;for(const auto & s:m.segments()) {
    for(const auto & p:s.points) {
      nearest = std::min(nearest, p.x);
    }
}
 EXPECT_LT(nearest, .34);
 auto path = m.path_to_base(Pose2D{.2, 0, 0}, rclcpp::Time(4, 0));ASSERT_FALSE(path.empty());
 EXPECT_LT(path.front().x, .15);
}

TEST(Memory, ReobservedCornerHasOneSharedVertex)
{
 LocalPathMemory m(10.);
 for(int k = 0; k < 80; ++k) {
    double t = 1. + k * .03, dx = .004 * std::sin(k), dy = .003 * std::cos(k);
    m.update(rclcpp::Time(int64_t(t * 1e9)), Pose2D{});
    auto a = segment(1, t, .1, .6);auto b = segment(2, t, 0., .25);
    for(auto & p:a.points) {
      p.x += dx;p.y += dy;
    }
    for(auto & p:b.points) {
      p.y = p.x + dy;p.x = .6 + dx;
    }
    a.connected_ids = {2};b.connected_ids = {1};m.store({a, b});
 }
 ASSERT_EQ(m.segments().size(), 2u);
 const auto & a = m.segments()[0].points;const auto & b = m.segments()[1].points;
 double closest = 1.;for(auto i:{a.front(), a.back()}) {
    for(auto j:{b.front(), b.back()}) {
      closest = std::min(closest, std::hypot(i.x - j.x, i.y - j.y));
    }
}
 EXPECT_LT(closest, 1e-8);
}

#include <thread>
#include <chrono>
TEST(CapturePose, FrozenRosClockDoesNotHideStaleSensors)
{
 PoseEstimator p;p.add_odom(odom(1., 0, 0, 0));
 sensor_msgs::msg::Imu imu;imu.header.stamp = rclcpp::Time(1, 0);imu.orientation.w = 1.;
    p.add_imu(imu);
 EXPECT_TRUE(p.healthy(rclcpp::Time(1, 0), .1));
 std::this_thread::sleep_for(std::chrono::milliseconds(150));
 EXPECT_FALSE(p.healthy(rclcpp::Time(1, 0), .1));
}

TEST(Memory, RejectsUnobservedLongEdges) {
  LocalPathMemory m; auto s = segment(1, 1); s.points = {s.points.front(), s.points.back()};
  m.store({s}); EXPECT_TRUE(m.segments().empty());
}
TEST(Memory, ParallelRepeatDoesNotDoubleBackInLocalPath) {
  LocalPathMemory m; m.store({segment(1, 1, .1, .8)});
  auto s = segment(2, 2, .1, .8, .02);m.store({s});
  auto path = m.path_to_base(Pose2D{}, rclcpp::Time(2, 0));
  ASSERT_GT(path.size(), 2u);
  for(size_t k = 1; k < path.size(); ++k) {
    EXPECT_GE(path[k].x, path[k - 1].x - .002);
    EXPECT_LT(std::hypot(path[k].x - path[k - 1].x, path[k].y - path[k - 1].y), .08);
  }
  EXPECT_EQ(m.segments().size(), 1u);
}
TEST(CapturePose, GyroHeadingOptionIntegratesAndResetAnchors) {
  PoseEstimator p; p.use_imu_yaw(true);p.add_odom(odom(1, 0, 0, .2));
  for(int k = 0; k <= 100; ++k) {
    sensor_msgs::msg::Imu m;m.orientation.w = 1;
    m.header.stamp = rclcpp::Time(int64_t((1 + k * .01) * 1e9));
    m.angular_velocity.z = .5;p.add_imu(m);
  }
  p.add_odom(odom(2, .1, 0, .6)); ASSERT_TRUE(p.latest());EXPECT_NEAR(p.latest()->yaw, .7, .006);
  p.reset();p.add_odom(odom(3, 0, 0, 1.));EXPECT_NEAR(p.latest()->yaw, 1., 1e-8);
}

TEST(Memory, ConnectedParallelDuplicateDoesNotRetraceConfirmedLine) {
  LocalPathMemory m;auto a = segment(1, 1, .1, .8);auto b = segment(2, 1, .1, .8, .02);
  a.connected_ids = {2};b.connected_ids = {1};m.store({a, b});
  auto path = m.path_to_base(Pose2D{}, rclcpp::Time(1, 0));
  ASSERT_GT(path.size(), 2u);EXPECT_GT(path.back().x, .75);
  for(size_t k = 1; k < path.size(); ++k) {
    EXPECT_GE(path[k].x, path[k - 1].x - .002);
}
}

TEST(Memory, RecordedIdsNeverConnectFarApartEndpoints) {
  LocalPathMemory m;auto a=segment(1,1,.1,.2);auto b=segment(2,1,.6,.7);
  a.connected_ids={2};b.connected_ids={1};m.store({a,b});
  ASSERT_EQ(m.segments().size(),2u);
  for(const auto & s:m.segments()) EXPECT_TRUE(s.connected_ids.empty());
}

TEST(Memory, CrossingChoosesAndLocksStraightContinuity) {
  LocalPathMemory m;auto a=segment(1,1,.1,.6),b=segment(2,1,.6,1.0),c=segment(3,1,0.,.3),d=segment(4,1,0.,.3);
  for(auto & p:c.points) {p.y=p.x;p.x=.6;}for(auto & p:d.points) {p.y=-p.x;p.x=.6;}
  a.connected_ids={2,3,4};b.connected_ids={1,3,4};c.connected_ids={1,2,4};d.connected_ids={1,2,3};m.store({a,b,c,d});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));ASSERT_FALSE(path.empty());EXPECT_GT(path.back().x,.95);EXPECT_NEAR(path.back().y,0.,.01);
  EXPECT_EQ(m.decisions().size(),3u);
}
TEST(Memory, RotatedRobotChoosesOutgoingForwardLeg) {
  LocalPathMemory m;auto a=segment(1,1,.1,.6),b=segment(2,1,0.,.4);
  for(auto & p:b.points) {p.y=p.x;p.x=.6;}a.connected_ids={2};b.connected_ids={1};m.store({a,b});
  auto path=m.path_to_base(Pose2D{.58,0,M_PI/2},rclcpp::Time(1,0));ASSERT_FALSE(path.empty());EXPECT_GT(path.back().x,.35);
}

TEST(Memory, CurveRootUsesLocalTangentNotWholeSegmentChord) {
  LocalPathMemory m;auto s=segment(1,1,.1,.3);
  auto p=s.points.back();
  for(double a=.1;a<M_PI/2;a+=.1) {p.x=.3+.11*std::sin(a);p.y=.11*(1-std::cos(a));s.points.push_back(p);}
  p.x=.41;p.y=.11;s.points.push_back(p);
  for(double y=.12;y<.8;y+=.01) {p.y=y;s.points.push_back(p);}
  m.store({s});auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));
  ASSERT_GT(path.size(),20u);EXPECT_LT(path.front().x,.15);EXPECT_GT(path.back().y,.7);
}

TEST(Memory, ShortDisconnectedFragmentDoesNotHideContinuousForwardLine) {
  LocalPathMemory m;
  m.store({segment(1,1,.001,.041),segment(2,1,.025,.7,.005)});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));
  ASSERT_GT(path.size(),10u);EXPECT_GT(path.back().x,.6);
}

TEST(Memory, CapacityPreservesNearBlindZoneBeforeNewDistantFragments) {
  LocalPathMemory m(20.,1200,2);m.update(rclcpp::Time(1,0),Pose2D{});
  m.store({segment(1,1,.1,.4)});
  m.store({segment(2,2,1.,1.3,.2)});
  m.store({segment(3,3,1.5,1.8,-.2)});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(3,0));
  ASSERT_FALSE(path.empty());EXPECT_LT(path.front().x,.2);EXPECT_EQ(m.segments().size(),2u);
}

TEST(Memory, SharedJunctionEndpointIsNotDuplicateOutgoingGeometry) {
  LocalPathMemory m;auto a=segment(1,1,.01,.15);LineSegment b;b.id=2;b.confidence=.9;
  for(int k=0;k<4;++k) {auto p=a.points.back();p.x=.15;p.y=k*.05;b.points.push_back(p);}
  a.connected_ids={2};b.connected_ids={1};m.store({a,b});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));
  ASSERT_FALSE(path.empty());EXPECT_GT(path.back().y,.1);
}

TEST(Memory, SparseStaleTailDoesNotDeleteStillConnectedObservedCornerLeg) {
  LocalPathMemory m;auto a=segment(1,1,.1,.4);LineSegment b;b.id=2;b.confidence=.9;
  for(double y:{0.,.05,.10,.13,.209}) {auto p=a.points.back();p.x=.4;p.y=y;b.points.push_back(p);}
  a.connected_ids={2};b.connected_ids={1};m.store({a,b});
  auto fresh=b;fresh.points.back().y=.12;fresh.points.erase(fresh.points.end()-2);
  m.store({fresh});
  ASSERT_EQ(m.segments().size(),2u);
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));ASSERT_FALSE(path.empty());EXPECT_GT(path.back().y,.10);
  for(const auto & s:m.segments()) for(size_t k=1;k<s.points.size();++k)
    EXPECT_LE(std::hypot(s.points[k].x-s.points[k-1].x,s.points[k].y-s.points[k-1].y),.080001);
}

TEST(Memory, ReobservedSegmentMayNotMoveKnownJunctionIntoItsInterior) {
  LocalPathMemory m;auto a=segment(1,1,.1,.6),b=segment(2,1,0.,.3);
  for(auto & p:b.points) {p.y=p.x;p.x=.6;}a.connected_ids={2};b.connected_ids={1};m.store({a,b});
  auto fresh=segment(2,2,-.1,.3);for(auto & p:fresh.points) {p.y=p.x;p.x=.6;}
  m.store({fresh});auto path=m.path_to_base(Pose2D{},rclcpp::Time(2,0));
  ASSERT_FALSE(path.empty());EXPECT_GT(path.back().y,.25);
  ASSERT_EQ(m.segments().size(),2u);EXPECT_FALSE(m.segments()[0].connected_ids.empty());
}

TEST(Memory, OverlappingCopiesOfOneObservedLegAreNotAnAmbiguousJunction) {
  LocalPathMemory m;auto a=segment(1,1,.1,.6);std::vector<LineSegment> input{a};
  for(int id=2;id<=4;++id) {
    auto b=segment(id,1,0.,id==3?.53:.5);for(auto & p:b.points) {p.y=p.x;p.x=.6+(id-3)*.004;}
    b.connected_ids={1};input.push_back(b);input[0].connected_ids.push_back(id);
  }
  m.store(input);auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));ASSERT_FALSE(path.empty());EXPECT_GT(path.back().y,.3);
  ASSERT_EQ(m.decisions().size(),3u);EXPECT_EQ(std::count_if(m.decisions().begin(),m.decisions().end(),[](const auto & c){return c.selected;}),1);
}
TEST(Memory, SimilarInitialHeadingsWithDivergentGeometryRemainAmbiguous) {
  LocalPathMemory m;auto a=segment(1,1,.1,.6),b=segment(2,1,0.,.5),c=b;c.id=3;
  for(auto & p:b.points) {p.y=p.x;p.x=.6;}for(auto & p:c.points) {p.y=p.x;p.x=.6+.15*p.y;}
  a.connected_ids={2,3};b.connected_ids={1};c.connected_ids={1};m.store({a,b,c});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));ASSERT_FALSE(path.empty());EXPECT_LT(path.back().y,.03);
  EXPECT_EQ(std::count_if(m.decisions().begin(),m.decisions().end(),[](const auto & q){return q.selected;}),0);
}

TEST(Memory, InvalidNearRootCannotHideFreshVerifiedForwardGeometry) {
  LocalPathMemory m(20.);auto a=segment(1,1,.01,.3),b=segment(2,2,.32,1.);
  for(auto & p:a.points) p.uncertainty=.061;
  m.store({a,b});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(2,0));ASSERT_FALSE(path.empty());
  EXPECT_LT(path.front().uncertainty,.06);EXPECT_GT(path.front().x,.3);
}
TEST(Memory, UnusableNearPrefixCannotHideDriveableFreshRoot) {
  LocalPathMemory m(20.);auto a=segment(1,1,.01,.3),b=segment(2,2,.32,1.);
  for(size_t k=3;k<a.points.size();++k) a.points[k].uncertainty=.065;
  m.store({a,b});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(2,0));ASSERT_FALSE(path.empty());EXPECT_GT(path.front().x,.3);
}
TEST(Memory, ExpiredRootCannotHideFreshGeometry) {
  LocalPathMemory m(5.);m.store({segment(1,1,.01,.3),segment(2,9,.32,1.)});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(10,0));ASSERT_FALSE(path.empty());EXPECT_GT(path.front().x,.3);
}

TEST(Memory, NewlyObservedStraightLegCorrectsPrematureTurnLock) {
  LocalPathMemory m(20.);auto a=segment(1,1,.1,.6),left=segment(2,1,0.,.4),back=segment(3,1,0.,.4);
  for(auto & p:left.points) {p.y=p.x;p.x=.6;}
  for(auto & p:back.points) {p.y=-.909*p.x;p.x=.6-.416*p.x;}
  a.connected_ids={2,3};left.connected_ids={1,3};back.connected_ids={1,2};m.store({a,left,back});
  auto first=m.path_to_base(Pose2D{},rclcpp::Time(1,0));ASSERT_FALSE(first.empty());EXPECT_GT(first.back().y,.3);
  auto straight=segment(4,2,.6,1.2);a.connected_ids={2,3,4};left.connected_ids={1,3,4};straight.connected_ids={1,2,3};
  m.store({a,left,straight});auto path=m.path_to_base(Pose2D{.2,0,0},rclcpp::Time(2,0));
  ASSERT_FALSE(path.empty());EXPECT_GT(path.back().x,.9);EXPECT_NEAR(path.back().y,0.,.02);
  ASSERT_FALSE(m.decisions().empty());EXPECT_TRUE(std::any_of(m.decisions().begin(),m.decisions().end(),[](const auto & d){return d.selected && std::abs(d.heading)<.3;}));
}
TEST(Memory, ShortNearPrefixDoesNotJumpToUnrelatedParallelLine) {
  LocalPathMemory m(20.);auto a=segment(1,1,.01,.3),b=segment(2,2,.32,1.,.15);
  for(size_t k=3;k<a.points.size();++k) a.points[k].uncertainty=.065;
  m.store({a,b});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(2,0));ASSERT_FALSE(path.empty());EXPECT_NEAR(path.front().y,0.,.01);
}

TEST(Memory, RepeatedStaticCurveKeepsItsObservedOrderAndSampling) {
  LocalPathMemory m(30.);LineSegment curve;curve.id=1;curve.confidence=.9;
  for(int k=0;k<=24;++k) {
    const double a=k*M_PI/48;PathPoint p;p.x=.35+.11*std::sin(a);p.y=-.11*(1-std::cos(a));
    p.source=PathPoint::SOURCE_OBSERVED;p.confidence=.9;p.uncertainty=.008;curve.points.push_back(p);
  }
  for(int frame=0;frame<200;++frame) {
    auto observed=curve;const double t=1+frame*.03;
    for(auto & p:observed.points) {p.capture_stamp=rclcpp::Time(int64_t(t*1e9));p.x+=.004*std::sin(frame*.2);p.y+=.004*std::cos(frame*.2);}
    m.store({observed});
  }
  ASSERT_EQ(m.segments().size(),1u);const auto & points=m.segments().front().points;
  EXPECT_GE(points.size(),18u);
  for(size_t k=1;k<points.size();++k) {
    EXPECT_GE(points[k].x,points[k-1].x-1e-5);EXPECT_LE(points[k].y,points[k-1].y+1e-5);
    EXPECT_LT(std::hypot(points[k].x-points[k-1].x,points[k].y-points[k-1].y),.02);
  }
  EXPECT_NEAR(points.front().x,curve.points.front().x,.005);EXPECT_NEAR(points.back().y,curve.points.back().y,.005);
}

TEST(Memory, ShortVerifiedNearCurveRemainsAvailableForStoppedOrientation) {
  LocalPathMemory m(20.);auto curve=segment(1,1,.015,.045);
  for(auto & p:curve.points) p.y=-.6*(p.x-.015);
  m.store({curve});auto path=m.path_to_base(Pose2D{},rclcpp::Time(1,0));
  ASSERT_GE(path.size(),2u);EXPECT_LT(std::hypot(path.front().x,path.front().y),.08);
  EXPECT_LT(std::hypot(path.back().x-path.front().x,path.back().y-path.front().y),.055);
}

#include "line_follower_navigation/selected_path_memory.hpp"
TEST(SelectedPathMemory, FragmentedUpdatesCannotEraseStillValidSelectedCorner) {
  SelectedPathMemory m;auto route=segment(1,1,.01,.3).points;
  for(auto & p:route) p.valid_until=rclcpp::Time(30,0);
  for(double y=.01;y<=.3;y+=.01) {auto p=route.back();p.x=.3;p.y=y;route.push_back(p);}
  m.update(rclcpp::Time(1,0),Pose2D{});m.remember(route,Pose2D{});
  auto short_prefix=std::vector<PathPoint>(route.begin(),route.begin()+8);m.remember(short_prefix,Pose2D{});
  m.update(rclcpp::Time(2,0),Pose2D{.2,0,0});auto path=m.path_to_base(Pose2D{.2,0,0},rclcpp::Time(2,0));
  ASSERT_FALSE(path.empty());EXPECT_GT(path.back().y,.25);EXPECT_NEAR(path.back().x,.1,.001);
  EXPECT_TRUE(m.prefer(path,short_prefix));m.reset();EXPECT_TRUE(m.path_to_base(Pose2D{},rclcpp::Time(2,0)).empty());
}
TEST(SelectedPathMemory, ContradictionExpiryUnknownAndUnobservedGapNeverBecomeRoute) {
  SelectedPathMemory m;auto route=segment(1,1,.01,.5).points;for(auto & p:route)p.valid_until=rclcpp::Time(3,0);
  m.remember(route,Pose2D{});auto branch=route;for(auto & p:branch)p.y=p.x;
  EXPECT_FALSE(m.prefer(route,branch));EXPECT_TRUE(m.path_to_base(Pose2D{},rclcpp::Time(4,0)).empty());
  m.reset();route[4].source=PathPoint::SOURCE_UNKNOWN;m.remember(route,Pose2D{});EXPECT_TRUE(m.path_to_base(Pose2D{},rclcpp::Time(1,0)).empty());
  route[4].source=PathPoint::SOURCE_OBSERVED;route[4].x+=1.;m.remember(route,Pose2D{});EXPECT_TRUE(m.path_to_base(Pose2D{},rclcpp::Time(1,0)).empty());
}
TEST(SelectedPathMemory, UncertaintyGrowsAndReobservedDirectPointsRefreshWithoutMovingCorners) {
  SelectedPathMemory m;auto route=segment(1,1,.01,.5).points;for(auto & p:route)p.valid_until=rclcpp::Time(30,0);
  m.update(rclcpp::Time(1,0),Pose2D{});m.remember(route,Pose2D{});m.update(rclcpp::Time(2,0),Pose2D{.1,0,.1});
  auto aged=m.path_to_base(Pose2D{},rclcpp::Time(2,0));ASSERT_FALSE(aged.empty());EXPECT_GT(aged.front().uncertainty,.005);
  auto fresh=std::vector<PathPoint>(route.begin()+20,route.end());for(auto & p:fresh){p.capture_stamp=rclcpp::Time(2,0);p.valid_until=rclcpp::Time(31,0);p.y=.004;}
  m.remember(fresh,Pose2D{});auto refreshed=m.path_to_base(Pose2D{},rclcpp::Time(2,0));ASSERT_FALSE(refreshed.empty());
  for(const auto & p:refreshed){EXPECT_NEAR(p.y,0.,1e-9);EXPECT_EQ(p.source,PathPoint::SOURCE_MEMORY_UPDATED);}
  m.update(rclcpp::Time(40,0),Pose2D{});EXPECT_TRUE(m.path_to_base(Pose2D{},rclcpp::Time(40,0)).empty());
}
TEST(SelectedPathMemory, ProgressCannotJumpToNearbyReturnLegAndStorageIsBounded) {
  SelectedPathMemory m;auto route=segment(1,1,.01,.3).points;
  for(double y=.01;y<=.06;y+=.01){auto p=route.back();p.x=.3;p.y=y;route.push_back(p);}
  for(double x=.29;x>=.01;x-=.01){auto p=route.back();p.x=x;p.y=.06;route.push_back(p);}
  for(auto & p:route)p.valid_until=rclcpp::Time(30,0);
  m.update(rclcpp::Time(1,0),Pose2D{});m.remember(route,Pose2D{});m.update(rclcpp::Time(2,0),Pose2D{.05,.05,0});
  auto path=m.path_to_base(Pose2D{.05,.05,0},rclcpp::Time(2,0));ASSERT_FALSE(path.empty());EXPECT_LT(path.front().y,-.04);
  m.reset();route=segment(1,1,.01,4.).points;for(auto & p:route)p.valid_until=rclcpp::Time(30,0);
  m.remember(route,Pose2D{});EXPECT_LE(m.path_to_base(Pose2D{},rclcpp::Time(1,0)).size(),256u);
}
TEST(SelectedPathMemory, NewerMemoryEvidenceRefreshesOnceWithoutRenewingItself) {
  SelectedPathMemory m;auto route=segment(1,1,.01,.4).points;for(auto & p:route)p.valid_until=rclcpp::Time(30,0);
  m.update(rclcpp::Time(1,0),Pose2D{});m.remember(route,Pose2D{});m.update(rclcpp::Time(10,0),Pose2D{});
  auto fresh=route;for(auto & p:fresh){p.source=PathPoint::SOURCE_MEMORY_UPDATED;p.capture_stamp=rclcpp::Time(9,0);p.valid_until=rclcpp::Time(39,0);}
  m.refresh(fresh,Pose2D{});auto first=m.path_to_base(Pose2D{},rclcpp::Time(10,0));ASSERT_FALSE(first.empty());EXPECT_LT(first.front().uncertainty,.01);
  m.update(rclcpp::Time(11,0),Pose2D{});auto aged=m.path_to_base(Pose2D{},rclcpp::Time(11,0));
  for(auto & p:fresh)p.uncertainty=0.;
  m.refresh(fresh,Pose2D{});auto same=m.path_to_base(Pose2D{},rclcpp::Time(11,0));
  ASSERT_FALSE(same.empty());EXPECT_FLOAT_EQ(same.front().uncertainty,aged.front().uncertainty);
}
TEST(SelectedPathMemory, InteriorExpiryDoesNotConnectToAFreshTail) {
  SelectedPathMemory m;auto route=segment(1,1,.01,.5).points;for(auto & p:route)p.valid_until=rclcpp::Time(30,0);
  route[5].valid_until=rclcpp::Time(1,0);m.remember(route,Pose2D{});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(2,0));EXPECT_LE(path.size(),5u);
}
TEST(SelectedPathMemory, FreshSelectedBranchReplacesProvisionalTurnRatherThanKeepingItsTail) {
  SelectedPathMemory m;auto old=segment(1,1,.01,.3).points;
  for(double y=.01;y<=.3;y+=.01){auto p=old.back();p.x=.3;p.y=y;old.push_back(p);}
  for(auto & p:old)p.valid_until=rclcpp::Time(30,0);
  m.remember(old,Pose2D{});auto straight=segment(2,2,.01,.8).points;for(auto & p:straight)p.valid_until=rclcpp::Time(31,0);
  EXPECT_FALSE(m.prefer(m.path_to_base(Pose2D{},rclcpp::Time(2,0)),straight));m.remember(straight,Pose2D{});
  auto path=m.path_to_base(Pose2D{},rclcpp::Time(2,0));ASSERT_FALSE(path.empty());EXPECT_GT(path.back().x,.75);EXPECT_NEAR(path.back().y,0.,1e-9);
}
TEST(SelectedPathMemory, APartialShortcutCannotSkipAnObservedLoopEvenWithANewTail) {
  auto polyline=[](std::vector<std::pair<double,double>> corners) {
    std::vector<PathPoint> result;
    for(size_t i=1;i<corners.size();++i) {
      const auto a=corners[i-1],b=corners[i];const int n=int(std::ceil(std::hypot(b.first-a.first,b.second-a.second)/.01));
      for(int k=0;k<n;++k){PathPoint p;p.x=a.first+(b.first-a.first)*k/n;p.y=a.second+(b.second-a.second)*k/n;p.source=PathPoint::SOURCE_OBSERVED;p.uncertainty=.008;p.capture_stamp=rclcpp::Time(1,0);p.valid_until=rclcpp::Time(30,0);result.push_back(p);}
    }
    auto p=result.back();p.x=corners.back().first;p.y=corners.back().second;result.push_back(p);return result;
  };
  auto observed=polyline({{.01,0},{.3,0},{.3,.2},{.1,.2},{.1,.04},{-.2,.04}});
  auto shortcut=polyline({{.01,0},{.07,0},{.1,.04},{-.4,.04}});
  SelectedPathMemory m;m.remember(observed,Pose2D{});
  auto saved=m.path_to_base(Pose2D{},rclcpp::Time(1,0));EXPECT_TRUE(m.prefer(saved,shortcut));
  m.remember(shortcut,Pose2D{});auto retained=m.path_to_base(Pose2D{},rclcpp::Time(1,0));
  EXPECT_TRUE(std::any_of(retained.begin(),retained.end(),[](const auto & p){return p.y>.19;}));
  m.update(rclcpp::Time(31,0),Pose2D{});
  for(auto & p:shortcut){p.capture_stamp=rclcpp::Time(31,0);p.valid_until=rclcpp::Time(61,0);}
  m.remember(shortcut,Pose2D{});auto fresh=m.path_to_base(Pose2D{},rclcpp::Time(31,0));
  ASSERT_FALSE(fresh.empty());EXPECT_LT(fresh.back().x,-.35);
}
