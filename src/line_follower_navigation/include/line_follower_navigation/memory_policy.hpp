#pragma once
#include <algorithm>
#include <optional>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include "line_follower_interfaces/msg/line_segment.hpp"
#include "line_follower_interfaces/msg/drive_mode.hpp"
namespace line_follower_navigation
{
struct MemoryInput
{
  bool direct{false}, agrees{true}, healthy{true};
  double pitch{0.}, remaining{0.}, uncertainty{0.};
};
struct MemoryDecision
{
  uint8_t mode; double speed; bool accept_observation; std::string reason;
};
// Validity policy, independent of ROS callbacks and candidate ranking (D-08).
class MemoryPolicy
{
public:
  static bool observations_agree(
    const std::vector<line_follower_interfaces::msg::LineSegment> & fresh,
    const std::vector<line_follower_interfaces::msg::LineSegment> & old)
  {
    size_t matched = 0;double error = 0.;std::unordered_map<uint32_t, uint32_t> ids;
    for(const auto & a:fresh) {
      std::unordered_map<uint32_t, size_t> votes;
      for(size_t k = 1; k < a.points.size(); ++k) {
        const auto & p = a.points[k];double best = 1e9;uint32_t best_id = 0;
        const double ax = p.x - a.points[k - 1].x, ay = p.y - a.points[k - 1].y;
        for(const auto & b:old) {
          for(size_t j = 1; j < b.points.size(); ++j) {
            const auto & q = b.points[j - 1];
            const double bx = b.points[j].x - q.x, by = b.points[j].y - q.y;
            const double norm = std::hypot(ax, ay) * std::hypot(bx, by);
            if(norm < 1e-10 || std::abs((ax * bx + ay * by) / norm) < .94) {continue;}
            const double f = std::clamp(((p.x - q.x) * bx + (p.y - q.y) * by) / (bx * bx + by * by),
                0., 1.);
            const double d = std::hypot(p.x - q.x - f * bx, p.y - q.y - f * by);
            if(d < best) {best = d;best_id = b.id;}
          }
        }
        if(best < .07) {++matched;error += best;++votes[best_id];}
      }
      if(!votes.empty()) {
        ids[a.id] = std::max_element(votes.begin(), votes.end(), [](const auto & a, const auto & b){
              return a.second < b.second;
                                                                                                                                                 })
          ->first;
      }
    }
    const auto junction=[](const auto & a,const auto & b) {
      std::array<double,3> best{0.,0.,1e9};
      if(a.points.empty() || b.points.empty()) return best;
      for(const auto & p:{a.points.front(),a.points.back()}) for(const auto & q:{b.points.front(),b.points.back()}) {
        const double d=std::hypot(p.x-q.x,p.y-q.y);
        if(d<best[2]) best={(p.x+q.x)*.5,(p.y+q.y)*.5,d};
      }
      return best;
    };
    for(const auto & a:fresh) for(auto neighbor:a.connected_ids) {
      const auto b=std::find_if(fresh.begin(),fresh.end(),[&](const auto & s){return s.id==neighbor;});
      if(b==fresh.end()) continue;
      const auto observed=junction(a,*b);
      if(observed[2]>.04) return false;
      if(!ids.count(a.id) || !ids.count(neighbor) || ids[a.id]==ids[neighbor]) continue;
      const auto old_a=std::find_if(old.begin(),old.end(),[&](const auto & s){return s.id==ids[a.id];});
      const auto old_b=std::find_if(old.begin(),old.end(),[&](const auto & s){return s.id==ids[neighbor];});
      // Absence in a partial view is UNKNOWN, not a confirmed prohibition.
      // An already observed junction must agree in position when re-observed.
      if(std::find(old_a->connected_ids.begin(),old_a->connected_ids.end(),ids[neighbor])!=old_a->connected_ids.end()) {
        const auto known=junction(*old_a,*old_b);
        if(known[2]>.04 || std::hypot(known[0]-observed[0],known[1]-observed[1])>.06) return false;
      }
    }
    return matched >= 3 && error / matched < .03;
  }

  static std::vector<line_follower_interfaces::msg::PathPoint> verified_prefix(
    const std::vector<line_follower_interfaces::msg::PathPoint> & points, int64_t now_ns)
  {
    std::vector<line_follower_interfaces::msg::PathPoint> result;
    for(const auto & p:points) {
      const int64_t until=int64_t(p.valid_until.sec)*1000000000+p.valid_until.nanosec;
      if(p.source==p.SOURCE_UNKNOWN || !std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(p.uncertainty) || p.uncertainty<0 || p.uncertainty>=.06 || until<=now_ns) break;
      result.push_back(p);
    }
    return result;
  }

  static double observed_length(const std::vector<line_follower_interfaces::msg::PathPoint> & points)
  {
    double length=0.;
    for(size_t k=1;k<points.size();++k) length+=std::hypot(points[k].x-points[k-1].x,points[k].y-points[k-1].y);
    return length;
  }
  static std::vector<line_follower_interfaces::msg::PathPoint> without_short_backtracking(
    std::vector<line_follower_interfaces::msg::PathPoint> points)
  {
    for(size_t k=1;k+1<points.size();) {
      const auto & a=points[k-1];const auto & b=points[k];const auto & c=points[k+1];
      const double ax=b.x-a.x,ay=b.y-a.y,bx=c.x-b.x,by=c.y-b.y;
      const double l1=std::hypot(ax,ay),l2=std::hypot(bx,by);
      if(a.segment_id==b.segment_id && b.segment_id==c.segment_id &&
        a.source!=a.SOURCE_UNKNOWN && b.source!=b.SOURCE_UNKNOWN && c.source!=c.SOURCE_UNKNOWN &&
        l1>.003 && l2>.003 && l1<.03 && l2<.03 && (ax*bx+ay*by)<-.5*l1*l2 &&
        std::hypot(c.x-a.x,c.y-a.y)<.04) {
        points.erase(points.begin()+k);if(k>1) --k;
      } else ++k;
    }
    return points;
  }
  static double control_uncertainty(const std::vector<line_follower_interfaces::msg::PathPoint> & points,double horizon=.30)
  {
    double uncertainty=0.,along=0.;
    for(size_t k=0;k<points.size();++k) {
      if(k) along+=std::hypot(points[k].x-points[k-1].x,points[k].y-points[k-1].y);
      if(along>horizon) break;
      if(!std::isfinite(points[k].uncertainty)) return INFINITY;
      uncertainty=std::max(uncertainty,double(points[k].uncertainty));
    }
    return uncertainty;
  }
  static std::optional<double> observed_tangent_turn(
    const std::vector<line_follower_interfaces::msg::PathPoint> & raw)
  {
    const auto points=without_short_backtracking(raw);
    if(points.size()<2 || observed_length(points)>.18 ||
      std::hypot(points.front().x,points.front().y)>.08) return std::nullopt;
    if(points.size()==2 && points[0].segment_id!=points[1].segment_id) return std::nullopt;
    size_t end=0;double farthest=0.;
    for(size_t k=0;k<points.size();++k) {
      const auto & p=points[k];
      if(p.source==p.SOURCE_UNKNOWN || !std::isfinite(p.x) || !std::isfinite(p.y) || p.uncertainty>=.06) return std::nullopt;
      const double range=std::hypot(p.x,p.y);if(range>farthest) {farthest=range;end=k;}
    }
    if(end<1) return std::nullopt;
    // Ignore a folded terminal duplicate; use the furthest observed extent.
    for(size_t k=1;k<end;++k) {
      const double ax=points[k].x-points[k-1].x,ay=points[k].y-points[k-1].y;
      const double bx=points[k+1].x-points[k].x,by=points[k+1].y-points[k].y;
      if(std::hypot(ax,ay)>.008 && std::hypot(bx,by)>.008 &&
        std::abs(std::atan2(ax*by-ay*bx,ax*bx+ay*by))>1.05) return std::nullopt;
    }
    size_t before=end-1;
    while(before>0 && std::hypot(points[end].x-points[before].x,points[end].y-points[before].y)<.025) --before;
    const double dx=points[end].x-points[before].x,dy=points[end].y-points[before].y;
    if(std::hypot(dx,dy)<.020) return std::nullopt;
    const double angle=std::atan2(dy,dx);
    if(std::abs(angle)<.15 || std::abs(angle)>1.2) return std::nullopt;
    return angle;
  }
  static bool needs_image_fallback(bool usable,double remaining,double uncertainty,bool metric_rejected=false)
  {
    return metric_rejected || !usable || remaining<=.055 || uncertainty>=.06;
  }
  void reset() {lost_ = false; checks_ = 0; mismatches_ = 0;}
  MemoryDecision update(const MemoryInput & i)
  {
    using M = line_follower_interfaces::msg::DriveMode;
    const bool direct = i.direct && std::abs(i.pitch) < .035;
    if(!i.healthy || !std::isfinite(i.remaining) || !std::isfinite(i.uncertainty) ||
      i.remaining <= .055 || i.uncertainty<0 || i.uncertainty >= .06)
    {
      lost_ = true;checks_ = 0;
      return {M::STOPPING, 0., false, "no sufficiently certain stoppable observed path"};
    }
    uint8_t mode = M::VISION_TRACKING;bool accept = direct;
    std::string reason = "trusted current observation";
    if(!direct) {
      lost_ = true;checks_ = 0;mode = M::MEMORY_TRACKING;accept = false;
      reason = "trusted memory; projection or near line lost";
    } else if(!i.agrees) {
      lost_ = true;checks_ = 0;++mismatches_;accept = false;mode = M::REACQUIRING;
      reason = "re-observation mismatch; memory retained";
      if(mismatches_ >= 15) {
        return MemoryDecision{M::STOPPING, 0., false, "unresolved re-observation mismatch"};
      }
    } else if(lost_) {
      ++checks_;mismatches_ = 0;accept = false;mode = M::REACQUIRING;
      reason = "checking position heading and connectivity";
      if(checks_ >= 3) {
        lost_ = false;checks_ = 0;accept = true;mode = M::VISION_TRACKING;
        reason = "consistent re-observation verified";
      }
    } else {mismatches_ = 0;}
    double speed = std::min(.1,
        std::sqrt(2 * .15 * std::max(0., i.remaining - .055))) * std::max(0.,1-i.uncertainty/.06);
    if(mode == M::MEMORY_TRACKING) {speed = std::min(speed, .06);}
    if(mode == M::REACQUIRING) {speed = std::min(speed, .03);}
    if(i.remaining < .25 || i.uncertainty > .025) {
      if(mode != M::REACQUIRING) {mode = M::SLOWDOWN;}
      speed = std::min(speed, .04);reason += "; verified distance/uncertainty requires slowdown";
    }
    return {mode, speed, accept, reason};
  }

private:
  bool lost_{false};int checks_{0}, mismatches_{0};
};
// One bounded orientation action per measured position, independent of Qt/ROS.
class LookTurnSession
{
public:
  void reset() {active_=false;origin_.reset();timed_out_=false;attempts_=0;ended_=0.;}
  bool timed_out() const {return timed_out_;}
  std::optional<double> update(double now,double x,double y,double yaw,std::optional<double> hint,bool valid)
  {
    timed_out_=false;
    if(!valid) {if(active_) origin_=std::array<double,2>{x,y};active_=false;return std::nullopt;}
    if(active_) {
      const double error=std::remainder(goal_-yaw,2*M_PI);
      if(std::abs(error)<.08 || now-start_>6. || now<start_) {
        timed_out_=now-start_>6.;active_=false;origin_=std::array<double,2>{x,y};ended_=now;return std::nullopt;
      }
      return error;
    }
    if(!hint) return std::nullopt;
    if(origin_ && std::hypot(x-(*origin_)[0],y-(*origin_)[1])<.05) {
      if(attempts_>=2 || now-ended_<.8 || std::abs(std::remainder(yaw+*hint-goal_,2*M_PI))<.35) return std::nullopt;
    } else attempts_=0;
    goal_=yaw+*hint;start_=now;active_=true;++attempts_;return *hint;
  }
private:
  bool active_{false},timed_out_{false};int attempts_{0};double goal_{0.},start_{0.},ended_{0.};
  std::optional<std::array<double,2>> origin_;
};

// Last-resort bounded blind rotation search (design: "bounded recovery" round 3).
// Used only when every geometry-grounded recovery (trusted memory tangent, image
// tracking) has already failed this cycle. Never translates; rotates in place,
// gyro/odometry-yaw measured, first toward the side the line was last seen, then
// sweeps back through the original heading to the opposite side, each leg well
// inside the controller's single rotation-episode budget. Stops and latches
// `exhausted()` once every bounded leg has been tried without reacquisition; only
// an explicit reset() (mission/run restart) may reuse it afterwards.
class BlindSearchSession
{
public:
  void reset() {active_ = false; leg_ = 0; attempts_ = 0; exhausted_ = false;}
  bool exhausted() const {return exhausted_;}
  int leg() const {return leg_;}
  int attempts() const {return attempts_;}

  // `trigger` must be true only when the caller is otherwise about to give up
  // permanently and the pose used for `yaw` is fresh. `bias` is a signed cue
  // (sign only matters) for which side the line was last seen/expected on.
  // Returns the relative target_heading to command this cycle (same base-frame
  // convention as LookTurnSession / path_planner `target_heading`), or
  // std::nullopt once the search is exhausted.
  std::optional<double> update(double now, double yaw, double bias, bool trigger)
  {
    if(exhausted_) {return std::nullopt;}
    if(!trigger) {reset();return std::nullopt;}
    const double side = bias >= 0. ? 1. : -1.;
    if(!active_) {
      start_yaw_ = yaw;total_start_ = now;leg_ = 1;leg_start_ = now;
      goal_ = start_yaw_ + side * kLegAngle;active_ = true;++attempts_;
      return std::remainder(goal_ - yaw, 2 * M_PI);
    }
    if(now - total_start_ > kTotalBudget) {exhausted_ = true;active_ = false;return std::nullopt;}
    const double error = std::remainder(goal_ - yaw, 2 * M_PI);
    if(std::abs(error) >= .08 && now - leg_start_ <= kLegBudget) {return error;}
    if(leg_ >= kLegs) {exhausted_ = true;active_ = false;return std::nullopt;}
    ++leg_;leg_start_ = now;++attempts_;
    goal_ = (leg_ == 2) ? start_yaw_ : start_yaw_ - side * kLegAngle;
    return std::remainder(goal_ - yaw, 2 * M_PI);
  }

private:
  // NOTE on magnitude: motion_controller's rotate episode, once started, commits to
  // the current leg's fixed goal for up to its own 8 s budget even if the line is
  // reacquired mid-turn (same pre-existing limitation as LookTurnSession, which has
  // no external "found it, cancel" signal either). A blind leg is an ungrounded
  // guess (unlike LookTurnSession's observed-tangent hint), so it is kept well
  // short of the suggested ~120 deg bound to limit how far off a committed-but-
  // moot rotation can carry the heading once vision actually recovers.
  static constexpr double kLegAngle = 1.0472;    // ~60 deg: inside one rotate episode
  static constexpr double kLegBudget = 4.0;      // s, under the controller's 8 s episode cap
  static constexpr double kTotalBudget = 20.0;   // s, overall bound across every leg
  static constexpr int kLegs = 3;                // +side*60 deg, back to start, -side*60 deg
  bool active_{false}, exhausted_{false};
  int leg_{0}, attempts_{0};
  double goal_{0.}, start_yaw_{0.}, leg_start_{0.}, total_start_{0.};
};

}
