#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
#include <rclcpp/time.hpp>
#include "line_follower_navigation/pose_estimator.hpp"
#include "line_follower_interfaces/msg/path_point.hpp"

namespace line_follower_navigation
{
// Preserve the order of an actually selected connected path when graph tracks
// fragment. This buffer never connects independent strokes or fills an expiry gap.
class SelectedPathMemory
{
  using Point=line_follower_interfaces::msg::PathPoint;
  static double distance(const Point & a,const Point & b) {return std::hypot(a.x-b.x,a.y-b.y);}
  static double length(const std::vector<Point> & p) {double d=0.;for(size_t i=1;i<p.size();++i)d+=distance(p[i-1],p[i]);return d;}
  struct Projection {double distance{1e9},along{0.};};
  static Projection project(const Point & p,const std::vector<Point> & line) {
    Projection best;double along=0.;
    for(size_t i=1;i<line.size();++i) {
      const auto & a=line[i-1];const auto & b=line[i];const double dx=b.x-a.x,dy=b.y-a.y,l=std::hypot(dx,dy);
      if(l<1e-8)continue;
      const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(l*l),0.,1.);
      const double d=std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
      if(d<best.distance)best={d,along+t*l};
      along+=l;
    }
    return best;
  }
  static bool skips_selected_arc(const std::vector<Point> & saved,const std::vector<Point> & current) {
    if(saved.size()<2 || current.empty())return false;
    double previous=0.;Point last=saved.front();
    for(const auto & p:current) {
      const auto q=project(p,saved);if(q.distance>.06)continue;
      const double advance=q.along-previous;
      if(advance<-.12 || (advance>.12 && advance>2.5*distance(last,p)+.05))return true;
      if(q.along>=previous-.02){previous=std::max(previous,q.along);last=p;}
    }
    return false;
  }
  static Point to_odom(Point p,const Pose2D & pose) {
    const double x=p.x,y=p.y;p.x=pose.x+std::cos(pose.yaw)*x-std::sin(pose.yaw)*y;
    p.y=pose.y+std::sin(pose.yaw)*x+std::cos(pose.yaw)*y;p.source=Point::SOURCE_MEMORY_UPDATED;return p;
  }
  std::vector<Point> points_;
  std::optional<Pose2D> previous_;
  std::optional<int64_t> updated_;
public:
  void reset() {points_.clear();previous_.reset();updated_.reset();}
  void update(const rclcpp::Time & now,const Pose2D & pose) {
    if(updated_ && now.nanoseconds()<*updated_)reset();
    const double dt=updated_?std::max(0.,(now.nanoseconds()-*updated_)*1e-9):0.;
    const double travel=previous_?std::hypot(pose.x-previous_->x,pose.y-previous_->y):0.;
    const double turn=previous_?std::abs(std::remainder(pose.yaw-previous_->yaw,2*M_PI)):0.;
    for(auto & p:points_)p.uncertainty+=float(dt*.0015+travel*.01+turn*.003);
    // Monotonic progress in a short arc window prevents jumping to a nearby
    // return leg or the later traversal of a crossing.
    if(!points_.empty()) {
      size_t best=0;double nearest=std::hypot(points_[0].x-pose.x,points_[0].y-pose.y),along=0.;
      for(size_t i=1;i<points_.size();++i) {
        along+=distance(points_[i-1],points_[i]);if(along>travel+.08)break;
        const double d=std::hypot(points_[i].x-pose.x,points_[i].y-pose.y);
        if(d<nearest){nearest=d;best=i;}
      }
      points_.erase(points_.begin(),points_.begin()+best);
    }
    previous_=pose;updated_=now.nanoseconds();
  }
  void refresh(const std::vector<Point> & base,const Pose2D & pose) {
    std::vector<Point> fresh;for(const auto & p:base)fresh.push_back(to_odom(p,pose));
    for(auto & old:points_) {
      const Point * observed=nullptr;double best=.018;
      for(size_t i=0;i<base.size();++i) if(base[i].source!=Point::SOURCE_UNKNOWN &&
        std::isfinite(base[i].uncertainty) && base[i].uncertainty>=0 && base[i].uncertainty<.06 &&
        (!updated_ || rclcpp::Time(base[i].valid_until).nanoseconds()>*updated_) &&
        rclcpp::Time(base[i].capture_stamp).nanoseconds()>rclcpp::Time(old.capture_stamp).nanoseconds()) {
        const double d=distance(old,fresh[i]);if(d<best){best=d;observed=&base[i];}
      }
      if(observed) {
        old.uncertainty=observed->uncertainty+float(best);old.confidence=observed->confidence;
        old.capture_stamp=observed->capture_stamp;old.valid_until=observed->valid_until;
      }
    }
  }
  void remember(const std::vector<Point> & base,const Pose2D & pose) {
    if(base.size()<2)return;
    for(size_t i=0;i<base.size();++i) if(base[i].source==Point::SOURCE_UNKNOWN ||
      !std::isfinite(base[i].x) || !std::isfinite(base[i].y) ||
      !std::isfinite(base[i].uncertainty) || base[i].uncertainty<0 ||
      (i && distance(base[i-1],base[i])>.08))return;
    std::vector<Point> fresh;for(const auto & p:base)fresh.push_back(to_odom(p,pose));
    refresh(base,pose);
    const auto invalid=std::find_if(points_.begin(),points_.end(),[&](const auto & p){
      return p.uncertainty>=.06 || (updated_ && rclcpp::Time(p.valid_until).nanoseconds()<=*updated_);
    });
    points_.erase(invalid,points_.end());
    // A partial view of a nearby return leg cannot replace the observed loop.
    if(skips_selected_arc(points_,fresh))return;
    size_t matched=0;double furthest=0.;bool diverges=false;
    for(size_t i=0;i<fresh.size();++i) {
      const auto p=project(fresh[i],points_);
      if(p.distance<.025){++matched;furthest=std::max(furthest,p.along);}
      else if(matched>=2 && p.distance>.04 && furthest<length(points_)-.05)diverges=true;
    }
    if(points_.empty() || matched<2 || diverges)points_=std::move(fresh);
    else {
      size_t join=0;double gap=.018;
      for(size_t i=0;i<fresh.size();++i) {const double d=distance(points_.back(),fresh[i]);if(d<gap){gap=d;join=i;}}
      if(gap<.018) for(size_t i=join+1;i<fresh.size();++i) {
        if(distance(points_.back(),fresh[i])>.08)break;
        if(distance(points_.back(),fresh[i])>.006)points_.push_back(fresh[i]);
      }
    }
    double along=0.;size_t end=std::min<size_t>(256,points_.size());
    for(size_t i=1;i<end;++i){along+=distance(points_[i-1],points_[i]);if(along>1.5){end=i;break;}}
    points_.resize(end);
  }
  std::vector<Point> path_to_base(const Pose2D & pose,const rclcpp::Time & now) const {
    std::vector<Point> result;
    for(auto p:points_) {
      if(p.source==Point::SOURCE_UNKNOWN || !std::isfinite(p.uncertainty) || p.uncertainty>=.06 ||
        rclcpp::Time(p.valid_until).nanoseconds()<=now.nanoseconds())break;
      const double x=p.x-pose.x,y=p.y-pose.y;
      p.x=std::cos(pose.yaw)*x+std::sin(pose.yaw)*y;p.y=-std::sin(pose.yaw)*x+std::cos(pose.yaw)*y;
      p.source=Point::SOURCE_MEMORY_UPDATED;result.push_back(p);
    }
    if(result.size()>=2) {
      const auto a=result[0],b=result[1];const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
      if(l2>1e-12) {
        const double t=std::clamp(-(a.x*dx+a.y*dy)/l2,0.,1.);
        result[0].x=a.x+t*dx;result[0].y=a.y+t*dy;
        result[0].uncertainty=std::max(a.uncertainty,b.uncertainty);
        if(rclcpp::Time(b.valid_until).nanoseconds()<rclcpp::Time(a.valid_until).nanoseconds())result[0].valid_until=b.valid_until;
        if(rclcpp::Time(b.capture_stamp).nanoseconds()<rclcpp::Time(a.capture_stamp).nanoseconds())result[0].capture_stamp=b.capture_stamp;
      }
    }
    if(!result.empty() && (result[0].x<-.02 || result[0].x>.45 || std::abs(result[0].y)>.20))return {};
    return result;
  }
  bool prefer(const std::vector<Point> & saved,const std::vector<Point> & current) const {
    if(skips_selected_arc(saved,current))return true;
    if(saved.size()<2 || length(saved)<=length(current)+.04)return false;
    for(const auto & p:current)if(project(p,saved).distance>.03)return false;
    return true;
  }
};
}  // namespace line_follower_navigation
