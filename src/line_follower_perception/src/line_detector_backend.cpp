#include <cstdlib>
#include <iostream>
#include "line_follower_perception/line_detector_backend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <opencv2/imgproc.hpp>

#include "line_follower_interfaces/msg/path_point.hpp"
#include "line_follower_interfaces/msg/perception_status.hpp"

namespace line_follower_perception
{

using line_follower_interfaces::msg::LineSegment;
using line_follower_interfaces::msg::PathPoint;
using line_follower_interfaces::msg::PerceptionStatus;

uint8_t OpenCvBaselineDetector::id() const {return PerceptionStatus::BACKEND_OPENCV_BASELINE;}


namespace
{
// Zhang-Suen thinning preserves graph junctions; no smoothing across a corner or crossing.
cv::Mat thin(cv::Mat image)
{
  image /= 255;
  cv::Mat remove = cv::Mat::zeros(image.size(), CV_8U);
  bool changed = true;
  while (changed) {
    changed = false;
    for (int pass = 0; pass < 2; ++pass) {
      remove.setTo(0);
      for (int y = 1; y < image.rows - 1; ++y) {
        const auto * above = image.ptr<uint8_t>(y - 1);
        const auto * row = image.ptr<uint8_t>(y);
        const auto * below = image.ptr<uint8_t>(y + 1);
        auto * mark = remove.ptr<uint8_t>(y);
        for (int x = 1; x < image.cols - 1; ++x) {
          if (!row[x]) {continue;}
          const int p[] = {above[x], above[x + 1], row[x + 1], below[x + 1], below[x], below[x - 1],
            row[x - 1], above[x - 1]};
          int count = 0, transitions = 0;
          for (int k = 0; k < 8; ++k) {
            count += p[k]; transitions += !p[k] && p[(k + 1) % 8];
          }
          if (count < 2 || count > 6 || transitions != 1) {continue;}
          const bool protect = pass == 0 ? ((p[0] && p[2] && p[4]) || (p[2] && p[4] && p[6])) :
            ((p[0] && p[2] && p[6]) || (p[0] && p[4] && p[6]));
          if (!protect) {mark[x] = 1; changed = true;}
        }
      }
      image -= remove;
    }
  }
  return image;
}
}

cv::Point2d OpenCvBaselineDetector::project_pixel(double u, double v) const
{
  const double f = camera_.width / (2. * std::tan(camera_.hfov_deg * CV_PI / 360.));
  const double ry = -(u - (camera_.width - 1) * .5) / f;
  const double rz = -(v - (camera_.height_px - 1) * .5) / f;
  const double cr = std::cos(roll_), sr = std::sin(roll_), cp = std::cos(pitch_),
    sp = std::sin(pitch_);
  const double y = cr * ry - sr * rz, z = sr * ry + cr * rz;
  const double x = cp + sp * z, vertical = -sp + cp * z;
  const double scale = -camera_.height / vertical;
  return {camera_.x_offset + scale * x, scale *y};
}

bool OpenCvBaselineDetector::detect(const cv::Mat & bgr, LineObservation & out)
{
  out.segments.clear(); out.junctions.clear();
  out.image_tracking_valid=false;out.image_offset=out.image_heading=0.;
  out.line_found = false; out.finish_marker_candidate = false; out.quality = 0.f;
  out.observed_range_min = out.observed_range_max = 0.f;
  out.projection_valid = attitude_available_ && std::abs(pitch_) < camera_.slope_limit_rad &&
    std::abs(roll_) < camera_.slope_limit_rad;
  if (bgr.empty()) {return false;}
  if (bgr.type() != CV_8UC3 || bgr.cols != camera_.width || bgr.rows != camera_.height_px) {
    return false;
  }

  const double resolution = camera_.grid_resolution;
  constexpr double near = .28, half_width = .72;
  {
    // Reuse storage, update the mapping with capture-time attitude each frame.
    const cv::Size size(cvCeil(2 * half_width / resolution) + 2,
      cvCeil((camera_.range_max - near) / resolution) + 2);
    map_x_.create(size, CV_32F); map_y_.create(size, CV_32F);
    const double f = camera_.width / (2. * std::tan(camera_.hfov_deg * CV_PI / 360.));
    for (int row = 0; row < size.height; ++row) {
      for (int col = 0; col < size.width; ++col) {
        const double x = near + row * resolution, y = half_width - col * resolution;
        const double cp = std::cos(pitch_), sp = std::sin(pitch_);
        const double cr = std::cos(roll_), sr = std::sin(roll_);
        const double rx = cp * (x - camera_.x_offset) + sp * camera_.height;
        const double z = sp * (x - camera_.x_offset) - cp * camera_.height;
        const double ry = cr * y + sr * z, rz = -sr * y + cr * z;
        map_x_.at<float>(row, col) = static_cast<float>((camera_.width - 1) * .5 - f * ry / rx);
        map_y_.at<float>(row, col) = static_cast<float>((camera_.height_px - 1) * .5 - f * rz / rx);
      }
    }
  }
  cv::Mat gray, mask, ground;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  // The horizon is excluded before estimating illumination. A capped Otsu threshold
  // retains distant 2 cm strokes without interpreting a uniform light mat as a line.
  const int horizon = bgr.rows / 2;
  const double otsu = cv::threshold(gray.rowRange(horizon, bgr.rows), mask, 0, 255,
      cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  cv::threshold(gray, mask, std::min(50., otsu + 15.), 255, cv::THRESH_BINARY_INV);
  cv::Mat contrast, faint;
  cv::morphologyEx(gray, contrast, cv::MORPH_BLACKHAT,
    cv::getStructuringElement(cv::MORPH_RECT, {31, 31}));
  cv::threshold(contrast, faint, 24, 255, cv::THRESH_BINARY);
  // Recover thin anti-aliased grey strokes, but not broad grey terrain shadows.
  // Reject the whole broad component, rather than retaining its artificial edge.
  cv::Mat grey=(gray>50)&(gray<=110),grey_labels,grey_radius;
  const int grey_count=cv::connectedComponents(grey,grey_labels,8,CV_32S);
  cv::distanceTransform(grey,grey_radius,cv::DIST_L2,3);
  std::vector<bool> broad_grey(grey_count,false);
  for(int row=0;row<gray.rows;++row) for(int col=0;col<gray.cols;++col) {
    if(grey_radius.at<float>(row,col)>3.) broad_grey[grey_labels.at<int>(row,col)]=true;
  }
  for(int row=0;row<gray.rows;++row) for(int col=0;col<gray.cols;++col) {
    if(broad_grey[grey_labels.at<int>(row,col)]) faint.at<uint8_t>(row,col)=0;
  }
  cv::bitwise_or(mask, faint & (gray > 50) & (gray < 170), mask);
  mask.rowRange(0, horizon + 1).setTo(0);
  // Pixel-only steering evidence, independent of the ground projection. A single
  // narrow stroke must span the lower ROI; multiple runs / transverse bars reject it.
  cv::Mat tracking=cv::Mat::zeros(mask.size(),CV_8U),tracking_labels;
  const cv::Rect corridor(bgr.cols/10,bgr.rows*3/4,bgr.cols*8/10,bgr.rows/4);
  mask(corridor).copyTo(tracking(corridor));cv::connectedComponents(tracking,tracking_labels,8,CV_32S);
  std::set<int> seeds;
  for(int row=bgr.rows-32;row<bgr.rows-4;row+=4) for(int col=bgr.cols/10;col<bgr.cols*9/10;++col) {
    const int id=tracking_labels.at<int>(row,col);if(id) seeds.insert(id);
  }
  if(std::getenv("LF_IMAGE_DEBUG")) std::cerr<<"IMAGE seeds="<<seeds.size()<<std::endl;
  int bottom_runs=0;bool inside=false;
  for(int col=bgr.cols/10;col<bgr.cols*9/10;++col) {
    const bool black=seeds.size()==1 && tracking_labels.at<int>(bgr.rows-2,col)==*seeds.begin();
    if(black && !inside) ++bottom_runs;inside=black;
  }
  if(seeds.size()==1 && bottom_runs==1) {
    tracking=tracking_labels==*seeds.begin();cv::Mat small;
    cv::resize(tracking,small,{},.5,.5,cv::INTER_AREA);cv::threshold(small,small,32,255,cv::THRESH_BINARY);
    small.row(0).setTo(0);small.row(small.rows-1).setTo(0);
    small.col(0).setTo(0);small.col(small.cols-1).setTo(0);
    const auto skeleton=thin(small);const int width=skeleton.cols,height=skeleton.rows;
    auto neighbors=[&](int id) {
      std::vector<int> out;const int x=id%width,y=id/width;
      for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
        if((!dx&&!dy)||x+dx<0||x+dx>=width||y+dy<0||y+dy>=height) continue;
        if(!skeleton.at<uint8_t>(y+dy,x+dx)) continue;
        if(dx&&dy&&(skeleton.at<uint8_t>(y,x+dx)||skeleton.at<uint8_t>(y+dy,x))) continue;
        out.push_back((y+dy)*width+x+dx);
      }
      return out;
    };
    int root=-1;double closest=1e9;std::vector<int> endpoints;
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) if(skeleton.at<uint8_t>(y,x)) {
      const int id=y*width+x;if(neighbors(id).size()==1) endpoints.push_back(id);
      const double d=std::hypot(x-(width-1)*.5,y-(height-1.));
      if(y>=height-16 && d<closest) {closest=d;root=id;}
    }
    if(root>=0 && endpoints.size()>=2) {
      std::vector<int> parent(width*height,-1),queue{root},depth(width*height,0);parent[root]=root;
      for(size_t q=0;q<queue.size();++q) for(int next:neighbors(queue[q])) if(parent[next]<0) {
        parent[next]=queue[q];depth[next]=depth[queue[q]]+1;queue.push_back(next);
      }
      std::vector<int> outgoing;
      for(int endpoint:endpoints) if(parent[endpoint]>=0) {
        bool incoming_cap=depth[endpoint]<=12 ||
          (endpoint/width>=height-16 && std::abs(endpoint%width-root%width)<24);
        // A curved incoming stroke may move laterally by more than the old
        // cap radius. It is incoming only if it reaches the clipped bottom
        // entirely below the root; two bottom strokes were rejected above.
        if(!incoming_cap && endpoint/width>=height-4 && endpoint/width>root/width+4) {
          incoming_cap=true;
          for(int id=endpoint;id!=root;id=parent[id]) if(id/width<root/width-2) {incoming_cap=false;break;}
        }
        if(!incoming_cap) outgoing.push_back(endpoint);
      }
      if(std::getenv("LF_IMAGE_DEBUG")) {std::cerr<<"IMAGE root="<<root%width<<","<<root/width<<" outgoing="<<outgoing.size()<<" depths";for(int id:outgoing) std::cerr<<" "<<depth[id];std::cerr<<std::endl;}
      // A broad perspective stroke has short medial-axis spurs at the clipped
      // bottom cap. Only one direction may leave that local incoming cap.
      if(outgoing.size()==1 && depth[outgoing[0]]>=15) {
        const int end=outgoing[0];
        std::vector<int> path;for(int id=end;id!=root;id=parent[id]) path.push_back(id);
        path.push_back(root);std::reverse(path.begin(),path.end());
        const int target=path[std::min<size_t>(25,path.size()-1)];
        out.image_offset=(2.*(target%width)-319.5)/320.;
        out.image_heading=2.*((root%width)-(target%width))/320.;
        out.image_tracking_valid=true;
      }
    }
  }
  cv::remap(mask, ground, map_x_, map_y_, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  ground.row(0).setTo(0); ground.row(ground.rows - 1).setTo(0);
  ground.col(0).setTo(0); ground.col(ground.cols - 1).setTo(0);
  cv::Mat labels, stats, centroids;
  const int components = cv::connectedComponentsWithStats(ground, labels, stats, centroids, 8);
  for (int id = 1; id < components; ++id) {
    if (stats.at<int>(id, cv::CC_STAT_AREA) < 10) {ground.setTo(0, labels == id);}
  }
  cv::Mat skel = thin(ground.clone());
  std::vector<cv::Point> pixels;
  cv::findNonZero(skel, pixels);
  if (pixels.empty()) {return true;}
  const int width = skel.cols;
  const auto index = [width](cv::Point p) {return p.y * width + p.x;};
  const auto point = [width](int i) {return cv::Point(i % width, i / width);};
  const auto base = [&](cv::Point2d p) {
      return cv::Point2d(near + p.y * resolution, half_width - p.x * resolution);
    };
  std::vector<std::vector<int>> neighbors(skel.total());
  for (auto p : pixels) {
    auto & adj = neighbors[index(p)];
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((!dx && !dy) || !skel.at<uint8_t>(p.y + dy, p.x + dx)) {continue;}
        // Avoid the artificial triangular edges of 8-neighbor right-angle pixels.
        if (dx && dy && (skel.at<uint8_t>(p.y, p.x + dx) || skel.at<uint8_t>(p.y + dy, p.x))) {
          continue;
        }
        adj.push_back(index(p + cv::Point(dx, dy)));
      }
    }
  }
  std::vector<int> node(skel.total(), -1);
  std::vector<std::vector<int>> members;
  std::vector<cv::Point2d> centers;
  for (auto p : pixels) {
    int start = index(p);
    if (neighbors[start].size() == 2 || node[start] >= 0) {continue;}
    const int id = static_cast<int>(members.size());
    std::vector<int> queue{start}; node[start] = id;
    for (size_t k = 0; k < queue.size(); ++k) {
      for (int next : neighbors[queue[k]]) {
        if (neighbors[next].size() > 2 && node[next] < 0) {node[next] = id; queue.push_back(next);}
      }
    }
    cv::Point2d center;
    for (int v : queue) {
      center += cv::Point2d(point(v));
    }
    centers.push_back(center * (1. / queue.size())); members.push_back(std::move(queue));
  }
  // A closed observed loop has no endpoint. Break it once for a bounded trace.
  if (members.empty()) {
    node[index(pixels[0])] = 0; members.push_back({index(pixels[0])}); centers.push_back(pixels[0]);
  }
  struct Edge {std::vector<cv::Point2d> pts; int from, to;};
  std::vector<Edge> edges;
  std::set<std::pair<int, int>> visited;
  const auto visit = [&](int a, int b) {return visited.insert(std::minmax(a, b)).second;};
  for (size_t id = 0; id < members.size(); ++id) {
    for (int start : members[id]) {
      for (int next : neighbors[start]) {
        if (node[next] == static_cast<int>(id) || !visit(start, next)) {continue;}
        Edge edge{{base(centers[id])}, static_cast<int>(id), -1};
        int prev = start, current = next;
        for (size_t steps = 0; steps < skel.total(); ++steps) {
          if (node[current] >= 0) {
            edge.to = node[current]; edge.pts.push_back(base(centers[edge.to])); break;
          }
          edge.pts.push_back(base(point(current)));
          int following = -1;
          for (int candidate : neighbors[current]) {
            if (candidate != prev) {
              following = candidate; break;
            }
          }
          if (following < 0 || !visit(current, following)) {break;}
          prev = current; current = following;
        }
        double length = 0.;
        for (size_t i = 1; i < edge.pts.size(); ++i) {
          length += cv::norm(edge.pts[i] - edge.pts[i - 1]);
        }
        if (length >= .025) {edges.push_back(std::move(edge));}
      }
    }
  }
  // Split sharp direction changes explicitly; arcs retain their sampled curvature.
  struct Leg {std::vector<cv::Point2d> pts; int from, to;};
  std::vector<Leg> legs;
  for (const auto & edge : edges) {
    std::vector<cv::Point2f> input, simplified;
    for (auto p : edge.pts) {
      input.emplace_back(p);
    }
    cv::approxPolyDP(input, simplified, .012, false);
    // A thick right-angle stroke has a short medial-axis chamfer. Intersect
    // its two long straight legs instead of rounding the physical corner.
    for (size_t i = 1; i + 2 < simplified.size(); ) {
      const auto a = simplified[i] - simplified[i - 1];
      const auto bridge = simplified[i + 1] - simplified[i];
      const auto b = simplified[i + 2] - simplified[i + 1];
      const double cross = a.x * b.y - a.y * b.x;
      if (cv::norm(bridge) < .065 && cv::norm(a) > .07 && cv::norm(b) > .07 &&
        std::abs(a.dot(b) / (cv::norm(a) * cv::norm(b))) < .45 && std::abs(cross) > 1e-8)
      {
        const auto delta = simplified[i + 1] - simplified[i];
        const double t = (delta.x * b.y - delta.y * b.x) / cross;
        simplified[i] += a * t;
        simplified.erase(simplified.begin() + i + 1);
      } else {++i;}
    }
    size_t begin = 0; int from = edge.from;
    for (size_t i = 1; i + 1 < simplified.size(); ++i) {
      auto a = simplified[i] - simplified[i - 1], b = simplified[i + 1] - simplified[i];
      const double angle = std::acos(std::clamp(a.dot(b) / (cv::norm(a) * cv::norm(b)), -1., 1.));
      if (angle > CV_PI * .28) {
        const int corner = static_cast<int>(centers.size());
        const auto p = simplified[i];
        centers.emplace_back((half_width - p.y) / resolution, (p.x - near) / resolution);
        Leg leg{{}, from, corner};
        for (size_t k = begin; k <= i; ++k) {
          leg.pts.emplace_back(simplified[k]);
        }
        legs.push_back(std::move(leg)); begin = i; from = corner;
      }
    }
    Leg leg{{}, from, edge.to};
    for (size_t k = begin; k < simplified.size(); ++k) {
      leg.pts.emplace_back(simplified[k]);
    }
    if (leg.pts.size() > 1) {legs.push_back(std::move(leg));}
  }
  std::vector<std::vector<uint32_t>> incident(centers.size());
  // Flat-model footprint width provides an additional geometric sanity check.
  cv::Mat distance; cv::distanceTransform(ground, distance, cv::DIST_L2, 3);
  int broad = 0;std::vector<double> widths,close_widths;
  for (auto p : pixels) {
    broad += distance.at<float>(p) * resolution > .035;
    widths.push_back(distance.at<float>(p) * resolution);
    const auto ground_point=base(p);
    if(ground_point.x<.65 && std::abs(ground_point.y)<.25) close_widths.push_back(distance.at<float>(p)*resolution);
  }
  std::sort(widths.begin(), widths.end());
  // A level chassis does not imply a single ground plane (hilltop looking at
  // lower ground). A 2 cm stroke has a bounded BEV half-width. Quantiles reject
  // mixed-plane scale changes while tolerating rasterization and junction blobs.
  // Judge the dominant stroke, not a cropped unrelated distant component.
  // Terrain elevation / pitch separately reject a level hilltop or ramp.
  if(close_widths.size()>=8) {std::sort(close_widths.begin(),close_widths.end());widths=close_widths;}
  if(widths[widths.size()/2]<.0065 || widths[widths.size()/2]>.018 ||
    broad>static_cast<int>(pixels.size()*.35)) out.projection_valid=false;
  static int diagnostic_count=0;
  if(std::getenv("LF_PROJECTION_DEBUG") && ++diagnostic_count%30==0) std::cerr<<"WIDTH median="<<widths[widths.size()/2]<<" close="<<close_widths.size()<<" attitude="<<attitude_available_<<" pitch="<<pitch_<<" broad="<<broad<<" total="<<pixels.size()<<std::endl;
  // Isolated apparently-flat frames at a ramp transition cannot confirm a plane.
  if(!out.projection_valid) {
    projection_untrusted_ = true;recovery_frames_ = 0;
  } else if(projection_untrusted_) {
    if(++recovery_frames_ < 5) {out.projection_valid = false;} else {projection_untrusted_ = false;}
  }
  const float confidence = out.projection_valid ? .9f : .25f;
  for (auto & leg : legs) {
    LineSegment seg; seg.id = static_cast<uint32_t>(out.segments.size());
    seg.confidence = confidence;
    if (cv::norm(leg.pts.front()) > cv::norm(leg.pts.back())) {
      std::reverse(leg.pts.begin(), leg.pts.end());
    }
    // Resample only within each polyline edge; never cut across a corner.
    for (size_t i = 0; i + 1 < leg.pts.size(); ++i) {
      const int count = std::max(1, cvCeil(cv::norm(leg.pts[i + 1] - leg.pts[i]) / .015));
      for (int k = 0; k < count; ++k) {
        const auto p = leg.pts[i] + (leg.pts[i + 1] - leg.pts[i]) * (double(k) / count);
        PathPoint q; q.x = p.x; q.y = p.y; q.source = PathPoint::SOURCE_OBSERVED;
        q.segment_id = seg.id; q.confidence = confidence;
        q.uncertainty = out.projection_valid ? .008f : .08f;
        q.capture_stamp = out.header.stamp; seg.points.push_back(q);
      }
    }
    PathPoint q = seg.points.back(); q.x = leg.pts.back().x; q.y = leg.pts.back().y;
    seg.points.push_back(q);
    if (leg.from >= 0) {incident[leg.from].push_back(seg.id);}
    if (leg.to >= 0) {incident[leg.to].push_back(seg.id);}
    out.segments.push_back(std::move(seg));
  }
  for (size_t id = 0; id < incident.size(); ++id) {
    const auto & ids = incident[id];
    if (ids.size() < 2) {continue;}
    for (auto a : ids) {
      for (auto b : ids) {
        if (a != b) {
          out.segments[a].connected_ids.push_back(b);
        }
      }
    }
    // Two incident legs indicate a sharp corner only when their outward tangents differ.
    bool corner = ids.size() > 2;
    if (!corner) {
      const auto center = base(centers[id]);
      const auto tangent = [&](uint32_t seg) {
          const auto & pts = out.segments[seg].points;
          const cv::Point2d a(pts.front().x, pts.front().y), b(pts.back().x, pts.back().y);
          return cv::norm(a - center) > cv::norm(b - center) ? a - center : b - center;
        };
      auto a = tangent(ids[0]), b = tangent(ids[1]);
      corner = a.dot(b) / (cv::norm(a) * cv::norm(b)) > -.75;
    }
    if (corner) {
      line_follower_interfaces::msg::Junction j;
      auto p = base(centers[id]); j.x = p.x; j.y = p.y; j.segment_ids = ids;
      j.confidence = confidence;
      out.junctions.push_back(j);
      // A three-leg transverse bar is a marker candidate, never a mission decision.
      if (ids.size() == 3) {out.finish_marker_candidate = true;}
    }
  }
  out.line_found = !out.segments.empty();
  if (out.line_found) {
    out.observed_range_min = std::numeric_limits<float>::infinity();
    for (const auto & seg : out.segments) {
      for (const auto & p : seg.points) {
        const float range = static_cast<float>(std::hypot(p.x, p.y));
        out.observed_range_min = std::min(out.observed_range_min, range);
        out.observed_range_max = std::max(out.observed_range_max, range);
      }
    }
    out.quality = confidence;
  }
  return true;
}

uint8_t FastScnnDetector::id() const {return PerceptionStatus::BACKEND_FAST_SCNN;}

bool FastScnnDetector::detect(const cv::Mat &, LineObservation & out)
{
  out.line_found = false;
  return false;
}

std::unique_ptr<LineDetectorBackend> make_backend(
  const std::string & requested, const std::string & model_path, bool & fallback_active,
  const CameraGeometry & camera)
{
  fallback_active = false;
  if (requested == "fast_scnn") {
    auto backend = std::make_unique<FastScnnDetector>(model_path);
    if (backend->ready()) {
      return backend;
    }
    fallback_active = true;
  } else if (requested != "opencv_baseline") {
    fallback_active = true;
  }
  return std::make_unique<OpenCvBaselineDetector>(camera);
}

}  // namespace line_follower_perception
