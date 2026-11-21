#include <gtest/gtest.h>

#include "line_follower_interfaces/msg/path_point.hpp"
#include "line_follower_perception/line_detector_backend.hpp"

using line_follower_interfaces::msg::PathPoint;
using namespace line_follower_perception;

TEST(MakeBackend, FastScnnUnavailableFallsBackToOpenCv)
{
  bool fallback = false;
  auto backend = make_backend("fast_scnn", "", fallback);
  EXPECT_TRUE(fallback);
  EXPECT_EQ(backend->name(), "opencv_baseline");
}

TEST(MakeBackend, OpenCvRequestedIsNotFallback)
{
  bool fallback = true;
  auto backend = make_backend("opencv_baseline", "", fallback);
  EXPECT_FALSE(fallback);
  EXPECT_TRUE(backend->ready());
}

TEST(OpenCvBaseline, EmptyImageReportsNoLine)
{
  OpenCvBaselineDetector det;
  LineObservation obs;
  EXPECT_FALSE(det.detect(cv::Mat(), obs));
  EXPECT_FALSE(obs.line_found);
}


#include <opencv2/imgproc.hpp>
#include <cmath>

namespace
{
cv::Point pixel(double x, double y)
{
  const double f = 320.0 / std::tan(62.2 * M_PI / 360.0);
  return {cvRound(319.5 - f * y / (x - .076)), cvRound(239.5 + f * .103 / (x - .076))};
}
cv::Mat canvas() {return cv::Mat(480, 640, CV_8UC3, cv::Scalar(205, 205, 205));}
void stroke(cv::Mat & image, std::initializer_list<cv::Point> pts)
{
  auto prev = pts.begin();
  for (auto it = prev + 1; it != pts.end(); ++it) {
    cv::line(image, *prev, *it, cv::Scalar(0, 0, 0), 8);
    prev = it;
  }
}
LineObservation detect(const cv::Mat & image)
{
  OpenCvBaselineDetector detector;
  LineObservation obs;
  EXPECT_TRUE(detector.detect(image, obs));
  return obs;
}
}

TEST(OpenCvBaseline, WhiteImageHasNoLine)
{
  const auto obs = detect(canvas());
  EXPECT_FALSE(obs.line_found);
  EXPECT_TRUE(obs.segments.empty());
}

TEST(OpenCvBaseline, StraightObservedAndProjected)
{
  auto image = canvas();
  const std::vector<cv::Point> polygon = {pixel(.32, -.01), pixel(1.1, -.01), pixel(1.1, .01),
    pixel(.32, .01)};
  cv::fillConvexPoly(image, polygon, cv::Scalar(0, 0, 0));
  const auto obs = detect(image);
  ASSERT_TRUE(obs.line_found);
  ASSERT_EQ(obs.segments.size(), 1u);
  EXPECT_TRUE(obs.projection_valid);
  EXPECT_LT(obs.observed_range_min, .35);
  EXPECT_GT(obs.observed_range_max, 1.0);
  for (const auto & point : obs.segments[0].points) {
    EXPECT_EQ(point.source, PathPoint::SOURCE_OBSERVED);
    EXPECT_NEAR(point.y, 0., .012);
  }
}

TEST(OpenCvBaseline, KnownPixelProjection)
{
  OpenCvBaselineDetector detector;
  const auto p = detector.project_pixel(319.5, 399.5);
  const double f = 320.0 / std::tan(62.2 * M_PI / 360.0);
  EXPECT_NEAR(p.x, .076 + f * .103 / 160., 1e-9);
  EXPECT_NEAR(p.y, 0., 1e-9);
  EXPECT_GT(detector.project_pixel(239.5, 399.5).y, 0.);
}

TEST(OpenCvBaseline, LeftAndRightCornersStaySharp)
{
  for (double sign : {-1., 1.}) {
    auto image = canvas();
    stroke(image, {pixel(.32, 0), pixel(.60, 0), pixel(.60, sign * .22)});
    const auto obs = detect(image);
    ASSERT_TRUE(obs.line_found);
    EXPECT_GE(obs.segments.size(), 2u);
    ASSERT_FALSE(obs.junctions.empty());
    bool corner = false;
    for (const auto & j : obs.junctions) {
      corner |= j.segment_ids.size() == 2 && std::hypot(j.x - .60, j.y) < .03;
    }
    EXPECT_TRUE(corner);
  }
}

TEST(OpenCvBaseline, CrossingHasFourLegsNotOneBentLine)
{
  auto image = canvas();
  stroke(image, {pixel(.32, 0), pixel(1.1, 0)});
  stroke(image, {pixel(.60, -.24), pixel(.60, .24)});
  const auto obs = detect(image);
  EXPECT_GE(obs.segments.size(), 4u);
  bool crossing = false;
  for (const auto & j : obs.junctions) {
    crossing |= j.segment_ids.size() == 4;
                                                                            }
  EXPECT_TRUE(crossing);
  for (const auto & s : obs.segments) {
    EXPECT_FALSE(s.connected_ids.empty());
                                                                             }
}

TEST(OpenCvBaseline, TMarkIsOnlyCandidate)
{
  auto image = canvas();
  stroke(image, {pixel(.32, 0), pixel(.60, 0)});
  stroke(image, {pixel(.60, -.16), pixel(.60, .16)});
  const auto obs = detect(image);
  EXPECT_GE(obs.segments.size(), 3u);
  EXPECT_TRUE(obs.finish_marker_candidate);
}

TEST(OpenCvBaseline, ArcAndImageEdgeSurvive)
{
  auto image = canvas();
  std::vector<cv::Point> pts;
  for (double a = 0.; a < 1.1; a += .02) {
    pts.push_back(pixel(.35 + .4 * std::sin(a), .4 * (1 - std::cos(a))));
                                                                                                 }
  cv::polylines(image, pts, false, cv::Scalar(0, 0, 0), 8);
  auto obs = detect(image);
  EXPECT_TRUE(obs.line_found);
  EXPECT_FALSE(obs.segments.empty());
  image = canvas();
  cv::line(image, {2, 479}, {100, 310}, cv::Scalar(0, 0, 0), 5);
  obs = detect(image);
  EXPECT_TRUE(obs.line_found);
}

TEST(OpenCvBaseline, SlopeKeepsDiagnosticPointsButInvalidatesProjection)
{
  auto image = canvas();
  stroke(image, {pixel(.32, 0), pixel(1.1, 0)});
  OpenCvBaselineDetector detector;
  detector.set_attitude(.12, 0.);
  LineObservation obs;
  ASSERT_TRUE(detector.detect(image, obs));
  EXPECT_TRUE(obs.line_found);
  EXPECT_FALSE(obs.projection_valid);
  EXPECT_FALSE(obs.segments.empty());
  EXPECT_LT(obs.quality, .5);
}

TEST(OpenCvBaseline, SmallCapturePitchCorrectsProjection)
{
  OpenCvBaselineDetector detector;
  const auto level = detector.project_pixel(319.5, 399.5);
  detector.set_attitude(-.01, 0.);
  EXPECT_GT(detector.project_pixel(319.5, 399.5).x, level.x);
}

TEST(OpenCvBaseline, FaintDistantTransverseLineSurvives)
{
  auto image = canvas();
  cv::line(image, pixel(1.0, -.40), pixel(1.0, .40), cv::Scalar(135, 135, 135), 1);
  const auto obs = detect(image);
  EXPECT_TRUE(obs.line_found);
  EXPECT_GT(obs.observed_range_max, .9);
}

TEST(OpenCvBaseline, MissingAttitudeInvalidatesProjection)
{
  auto image = canvas();
  stroke(image, {pixel(.32, 0), pixel(1.1, 0)});
  OpenCvBaselineDetector detector;
  detector.set_attitude(0., 0., false);
  LineObservation obs;
  ASSERT_TRUE(detector.detect(image, obs));
  EXPECT_TRUE(obs.line_found);
  EXPECT_FALSE(obs.projection_valid);
}

TEST(OpenCvBaseline, WidthInconsistentWithFlatPlaneIsUntrusted) {
  OpenCvBaselineDetector detector; auto image = canvas();
  // A uniformly thin ground footprint mimics a lower plane seen from a hilltop.
  cv::line(image, pixel(.4, 0), pixel(1.1, 0), cv::Scalar(0, 0, 0), 2);
  LineObservation obs;detector.detect(image, obs);
  ASSERT_TRUE(obs.line_found);EXPECT_FALSE(obs.projection_valid);
}

TEST(OpenCvBaseline, ProjectionNeedsStableRecoveryAfterSlope) {
  OpenCvBaselineDetector d;auto image = canvas();
  const std::vector<cv::Point> polygon = {pixel(.32, -.01), pixel(1.1, -.01), pixel(1.1, .01),
    pixel(.32, .01)};
  cv::fillConvexPoly(image, polygon, cv::Scalar(0, 0, 0));
  LineObservation o;d.set_attitude(.14, 0);d.detect(image, o);EXPECT_FALSE(o.projection_valid);
  d.set_attitude(0, 0);
  for(int k = 0; k < 4; ++k) {
    d.detect(image, o);EXPECT_FALSE(o.projection_valid);
                                                                           }
  d.detect(image, o);EXPECT_TRUE(o.projection_valid);
}

TEST(OpenCvBaseline, ImageSpaceTracksClearSlopeWithoutConfirmingGeometry) {
  auto image=canvas();cv::line(image,{320,300},{320,479},cv::Scalar(0,0,0),18);
  OpenCvBaselineDetector d;d.set_attitude(.14,0);LineObservation o;
  ASSERT_TRUE(d.detect(image,o));EXPECT_FALSE(o.projection_valid);
  EXPECT_TRUE(o.image_tracking_valid);EXPECT_NEAR(o.image_offset,0.,.03);
  image=canvas();d.detect(image,o);EXPECT_FALSE(o.image_tracking_valid);
  cv::line(image,{200,300},{200,479},cv::Scalar(0,0,0),18);
  cv::line(image,{440,300},{440,479},cv::Scalar(0,0,0),18);
  d.detect(image,o);EXPECT_FALSE(o.image_tracking_valid);
}

TEST(OpenCvBaseline, ImageSpaceIgnoresPeripheralUnrelatedStroke) {
  auto image=canvas();cv::line(image,{320,300},{320,479},cv::Scalar(0,0,0),18);
  cv::line(image,{80,300},{80,365},cv::Scalar(0,0,0),5);
  OpenCvBaselineDetector d;LineObservation o;d.detect(image,o);EXPECT_TRUE(o.image_tracking_valid);
}

TEST(OpenCvBaseline, ImageSpaceAcceptsContinuousLowerLineBelowRaisedHorizon) {
  auto image=canvas();cv::line(image,{320,375},{320,479},cv::Scalar(0,0,0),18);
  OpenCvBaselineDetector d;LineObservation o;d.detect(image,o);EXPECT_TRUE(o.image_tracking_valid);
  image=canvas();cv::line(image,{320,300},{320,425},cv::Scalar(0,0,0),18);
  d.detect(image,o);EXPECT_FALSE(o.image_tracking_valid);
}

#include "line_follower_perception/terrain_trust.hpp"
TEST(TerrainTrust, HeightChangeKeepsLevelHilltopUntrustedUntilDescent) {
  TerrainTrust t;t.update(0.,0.,0.);EXPECT_TRUE(t.trusted());
  t.update(-std::atan2(.06,.44),.445,.06);EXPECT_FALSE(t.trusted());
  for(int k=0;k<100;++k) t.update(0.,.004,.06);
  EXPECT_FALSE(t.trusted());EXPECT_GT(t.height(),.04);
  t.update(std::atan2(.06,.44),.445,.06);t.update(0.,0.,0.);
  EXPECT_TRUE(t.trusted());EXPECT_NEAR(t.height(),0.,.01);
}
TEST(TerrainTrust, RestPitchBiasAndFlatTravelDoNotInventHeight) {
  TerrainTrust t;for(int k=0;k<100;++k) t.update(-.005,0.,0.);
  for(int k=0;k<1000;++k) t.update(-.005,.01,.06);
  EXPECT_TRUE(t.trusted());EXPECT_NEAR(t.height(),0.,.001);
}

TEST(OpenCvBaseline, FlatPhysicalStrokeSurvivesOffsetAndSmallHeading) {
  for(double offset : {-.03,-.02,-.01,0.,.01,.02,.03}) for(double slope : {-.04,0.,.04}) {
    auto image=canvas();std::vector<cv::Point> polygon;
    for(auto xy : {std::pair<double,double>{.30,-.01},{1.1,-.01},{1.1,.01},{.30,.01}}) polygon.push_back(pixel(xy.first,offset+slope*(xy.first-.3)+xy.second));
    cv::fillConvexPoly(image,polygon,cv::Scalar(0,0,0));auto o=detect(image);
    EXPECT_TRUE(o.projection_valid)<<offset<<" "<<slope;
  }
}

TEST(OpenCvBaseline, GreyTerrainShadowIsNotABlackPath) {
  auto image=canvas();std::vector<cv::Point> shadow={{0,280},{500,310},{0,345}};
  cv::fillConvexPoly(image,shadow,cv::Scalar(80,80,80));
  std::vector<cv::Point> line={pixel(.32,-.01),pixel(1.1,-.01),pixel(1.1,.01),pixel(.32,.01)};
  cv::fillConvexPoly(image,line,cv::Scalar(0,0,0));auto o=detect(image);
  EXPECT_TRUE(o.projection_valid);ASSERT_TRUE(o.line_found);
  for(const auto & seg:o.segments) for(const auto & p:seg.points) EXPECT_NEAR(p.y,0.,.02);
}

TEST(OpenCvBaseline, TrueWidthDiagonalFlatStrokeStaysProjectable) {
  for(double angle : {-.785398,-.5,.5,.785398}) for(double offset : {-.02,0.,.02}) {
    auto image=canvas();const cv::Point2d a(.30,offset),b(.95,offset+.65*std::tan(angle));
    const cv::Point2d normal(-std::sin(angle)*.01,std::cos(angle)*.01);
    std::vector<cv::Point> polygon;
    for(auto q:{a-normal,b-normal,b+normal,a+normal}) polygon.push_back(pixel(q.x,q.y));
    cv::fillConvexPoly(image,polygon,cv::Scalar(0,0,0));auto o=detect(image);
    ASSERT_TRUE(o.line_found);EXPECT_TRUE(o.projection_valid)<<angle<<" "<<offset;
  }
}

TEST(OpenCvBaseline, DarkGreyDistantTransverseLineSurvives) {
  auto image=canvas();
  cv::line(image,pixel(1.0,-.40),pixel(1.0,.40),cv::Scalar(80,80,80),1);
  auto obs=detect(image);EXPECT_TRUE(obs.line_found);EXPECT_GT(obs.observed_range_max,.9);
}

TEST(OpenCvBaseline, ImageSpaceTracksSingleCurvedVisibleStroke) {
  auto image=canvas();std::vector<cv::Point> curve={{320,479},{315,465},{290,450},{250,438},{190,430}};
  cv::polylines(image,curve,false,cv::Scalar(0,0,0),12);
  OpenCvBaselineDetector d;d.set_attitude(.14,0);LineObservation o;d.detect(image,o);
  EXPECT_FALSE(o.projection_valid);EXPECT_TRUE(o.image_tracking_valid);
  EXPECT_LT(o.image_offset,-.05);EXPECT_GT(o.image_heading,.05);
}
TEST(OpenCvBaseline, ImageSpaceKeepsVisibleRightAngleAndRejectsAmbiguousBar) {
  auto image=canvas();std::vector<cv::Point> corner={{320,479},{320,450},{120,450}};
  cv::polylines(image,corner,false,cv::Scalar(0,0,0),12);
  OpenCvBaselineDetector d;LineObservation o;d.detect(image,o);EXPECT_TRUE(o.image_tracking_valid);EXPECT_GT(o.image_heading,.05);
  image=canvas();cv::line(image,{70,460},{570,460},cv::Scalar(0,0,0),12);
  d.detect(image,o);EXPECT_FALSE(o.image_tracking_valid);
}

TEST(OpenCvBaseline, ImageSpaceTrapezoidHasOneOutgoingStrokeDespiteBottomCapSpurs) {
  auto image=canvas();std::vector<cv::Point> polygon={{307,350},{327,350},{352,479},{280,479}};
  cv::fillConvexPoly(image,polygon,cv::Scalar(0,0,0));
  OpenCvBaselineDetector d;d.set_attitude(.1,0);LineObservation o;d.detect(image,o);
  EXPECT_TRUE(o.image_tracking_valid);EXPECT_NEAR(o.image_offset,0.,.1);
}

#include <opencv2/imgcodecs.hpp>
TEST(OpenCvBaseline, ContinuousGazeboCurveKeepsItsNearImageSteeringEvidence) {
  auto image=cv::imread(std::string(TEST_IMAGE_DIR)+"/curved_stroke_gazebo.png");
  ASSERT_FALSE(image.empty());OpenCvBaselineDetector d;LineObservation o;d.detect(image,o);
  EXPECT_TRUE(o.image_tracking_valid);EXPECT_TRUE(std::isfinite(o.image_offset));EXPECT_TRUE(std::isfinite(o.image_heading));
}
TEST(OpenCvBaseline, CurvedBottomContinuationIsNotASecondOutgoingBranch) {
  auto image=canvas();std::vector<cv::Point> curve={{460,479},{405,448},{340,425},{240,405},{170,370}};
  cv::polylines(image,curve,false,cv::Scalar(0,0,0),18);
  OpenCvBaselineDetector d;LineObservation o;d.detect(image,o);EXPECT_TRUE(o.image_tracking_valid);
}

TEST(OpenCvBaseline, ConnectedJunctionWithTwoBottomStrokesIsAmbiguous) {
  auto image=canvas();cv::line(image,{320,300},{320,430},cv::Scalar(0,0,0),12);
  cv::line(image,{320,430},{200,479},cv::Scalar(0,0,0),12);
  cv::line(image,{320,430},{440,479},cv::Scalar(0,0,0),12);
  OpenCvBaselineDetector d;LineObservation o;d.detect(image,o);EXPECT_FALSE(o.image_tracking_valid);
}
