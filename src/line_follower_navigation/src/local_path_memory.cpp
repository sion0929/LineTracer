#include "line_follower_navigation/local_path_memory.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace line_follower_navigation
{
namespace
{
double distance(const PathPoint & a, const PathPoint & b) {return std::hypot(a.x - b.x, a.y - b.y);}
struct Projection {double distance{1e9}, parameter{0.};};
Projection project(const PathPoint & p, const std::vector<PathPoint> & line)
{
  Projection result; double length = 0.;
  for(size_t i = 1; i < line.size(); ++i) {
    const auto & a = line[i - 1];const auto & b = line[i];
    const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    if(l2 < 1e-12) {continue;}
    const double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2;
    const double clipped = std::clamp(t, 0., 1.);
    const double d = std::hypot(p.x - a.x - clipped * dx, p.y - a.y - clipped * dy);
    if(d < result.distance) {
      result.distance = d;
      const double extended = (i == 1 && t < 0) || (i + 1 == line.size() && t > 1) ? t : clipped;
      result.parameter = length + extended * std::sqrt(l2);
    }
    length += std::sqrt(l2);
  }
  return result;
}
bool same_observed_leg(const LineSegment & a,const LineSegment & b)
{
  const auto length=[](const auto & s) {double d=0.;for(size_t k=1;k<s.points.size();++k) d+=distance(s.points[k-1],s.points[k]);return d;};
  const auto & shorter=length(a)<length(b)?a:b;const auto & longer=length(a)<length(b)?b:a;
  if(length(shorter)<.08) return false;
  size_t matched=0,sampled=0;const size_t stride=std::max<size_t>(1,shorter.points.size()/20);
  for(size_t k=0;k<shorter.points.size();k+=stride) {
    ++sampled;if(project(shorter.points[k],longer.points).distance<.015) ++matched;
  }
  return sampled>=3 && matched>=sampled*.80;
}
PathPoint to_base(PathPoint p, const Pose2D & pose)
{
  const double dx = p.x - pose.x, dy = p.y - pose.y, c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  p.x = c * dx + s * dy;p.y = -s * dx + c * dy;p.source = PathPoint::SOURCE_MEMORY_UPDATED;return p;
}
bool direct(const LineSegment & s)
{
  for(size_t k = 1; k < s.points.size(); ++k) {
    if(distance(s.points[k - 1], s.points[k]) > .08) {return false;}
  }
  return s.points.size() > 1 && std::all_of(s.points.begin(), s.points.end(), [](const auto & p) {
             return p.source == PathPoint::SOURCE_OBSERVED && std::isfinite(p.x) &&
                    std::isfinite(p.y) &&
                    std::isfinite(p.confidence) && std::isfinite(p.uncertainty);
 });
}
}

void LocalPathMemory::reset() {locks_.clear();decisions_.clear();segments_.clear();next_id_ = 1;updated_.reset();current_.reset();}
bool LocalPathMemory::set_run_id(const std::string & id)
{
  if(id.empty() || id == run_id_) {return false;}
  reset();run_id_ = id;return true;
}

std::unordered_map<uint32_t,
  uint32_t> LocalPathMemory::store(const std::vector<LineSegment> & input)
{
  std::unordered_map<uint32_t, uint32_t> ids;
  std::set<uint32_t> used;
  for(const auto & observed:input) {
    if(!direct(observed)) {continue;}
    size_t best = segments_.size();double best_cost = 1e9;
    for(size_t index = 0; index < segments_.size(); ++index) {
      const auto & old = segments_[index];if(used.count(old.id)) {continue;}
      size_t matched = 0;double total = 0.;
      for(size_t k = 0; k < observed.points.size(); ++k) {
        const auto & p = observed.points[k];const auto projection = project(p, old.points);
        if(projection.distance > .04) {continue;}
        const auto & a = observed.points[k ? k - 1 : k];
        const auto & b = observed.points[k + 1 < observed.points.size() ? k + 1 : k];
        double alignment = 0.;
        for(size_t j = 1; j < old.points.size(); ++j) {
          if(std::min(distance(p, old.points[j - 1]), distance(p, old.points[j])) > .06) {continue;}
          const double ax = b.x - a.x, ay = b.y - a.y, bx = old.points[j].x - old.points[j - 1].x,
            by = old.points[j].y - old.points[j - 1].y;
          const double norm = std::hypot(ax, ay) * std::hypot(bx, by);
          if(norm > 1e-9) {alignment = std::max(alignment, std::abs((ax * bx + ay * by) / norm));}
        }
        if(alignment > .85) {++matched;total += projection.distance;}
      }
      if(matched >= 2 && matched >= observed.points.size() * .25) {
        const double cost = total / matched;
        if(cost < best_cost) {best = index;best_cost = cost;}
      }
    }
    if(best == segments_.size()) {
      auto fresh = observed;fresh.id = next_id_++;fresh.connected_ids.clear();fresh.points.clear();
      segments_.push_back(std::move(fresh));
    }
    auto & track = segments_[best];ids[observed.id] = track.id;used.insert(track.id);
    bool pin_front=false,pin_back=false;PathPoint original_front,original_back;
    if(!track.points.empty()) {
      original_front=track.points.front();original_back=track.points.back();
      for(auto neighbor:track.connected_ids) {
        const auto other=std::find_if(segments_.begin(),segments_.end(),[&](const auto & s){return s.id==neighbor;});
        if(other==segments_.end() || other->points.empty()) continue;
        for(const auto & endpoint:{other->points.front(),other->points.back()}) {
          if(distance(original_front,endpoint)<.04) pin_front=true;
          if(distance(original_back,endpoint)<.04) pin_back=true;
        }
      }
    }
    std::vector<std::pair<double, PathPoint>> merged;
    double along = 0.;
    for(size_t k = 0; k < track.points.size(); ++k) {
      if(k) {
        along += distance(track.points[k - 1], track.points[k]);
      }
      merged.emplace_back(along, track.points[k]);
    }
    for(auto fresh:observed.points) {
      fresh.segment_id = track.id;fresh.source = PathPoint::SOURCE_OBSERVED;
      fresh.valid_until = rclcpp::Time(fresh.capture_stamp) +
        rclcpp::Duration::from_seconds(retention_sec_);
      if(updated_) {
        fresh.uncertainty += float(std::max(0.,
          (*updated_ - rclcpp::Time(fresh.capture_stamp).nanoseconds()) * 1e-9) * .004);
      }
      if(track.points.empty()) {merged.emplace_back(double(merged.size()) * .01, fresh);continue;}
      auto nearest = std::min_element(merged.begin(), merged.end(),
          [&](const auto & a, const auto & b){
            return distance(a.second, fresh) < distance(b.second, fresh);
                                                                                                           });
      if(nearest != merged.end() && distance(nearest->second, fresh) < .03) {
        const double x = nearest->second.x * .75 + fresh.x * .25,
          y = nearest->second.y * .75 + fresh.y * .25;
        nearest->second = fresh;nearest->second.x = x;nearest->second.y = y;
      } else {merged.emplace_back(project(fresh, track.points).parameter, fresh);}
    }
    std::stable_sort(merged.begin(), merged.end(), [](const auto & a, const auto & b){
        return a.first < b.first;
                                                                                                               });
    track.points.clear();
    for(const auto & item:merged) {
      // Preserve an observed junction boundary when a later feature spans
      // across it. The connected leg owns geometry on the other side.
      if((pin_front && item.first<0.) || (pin_back && item.first>along)) continue;
      if(track.points.empty() || distance(track.points.back(), item.second) > .006) {
        track.points.push_back(item.second);
      } else if(rclcpp::Time(item.second.capture_stamp).nanoseconds() >
        rclcpp::Time(track.points.back().capture_stamp).nanoseconds())
      {
        track.points.back() = item.second;
      }
    }
    if(!track.points.empty()) {
      if(pin_front) {track.points.front().x=original_front.x;track.points.front().y=original_front.y;}
      if(pin_back) {track.points.back().x=original_back.x;track.points.back().y=original_back.y;}
    }
    track.confidence = observed.confidence;
  }
 // Connections are inherited only from a detector observation, never proximity alone.
  for(const auto & observed:input) {
    if(ids.count(observed.id)) {
      auto it = std::find_if(segments_.begin(), segments_.end(), [&](const auto & s){
            return s.id == ids[observed.id];
                                                                                                                     });
      if(it == segments_.end()) {continue;}
      for(auto neighbor:observed.connected_ids) {
        if(ids.count(neighbor) && ids[neighbor] != it->id &&
          std::find(it->connected_ids.begin(), it->connected_ids.end(),
            ids[neighbor]) == it->connected_ids.end())
        {
          it->connected_ids.push_back(ids[neighbor]);
        }
      }
    }
  }
 // A detector-connected pair owns one shared endpoint. Independent averaging
 // of the two legs would turn a sharp corner into a small diagonal/chamfer.
  for(const auto & observed:input) {
    if(ids.count(observed.id)) {
      for(auto neighbor:observed.connected_ids) {
        if(!ids.count(neighbor) || neighbor <= observed.id) {continue;}
        auto other = std::find_if(input.begin(), input.end(), [&](const auto & s){
              return s.id == neighbor;
                                                                                                          });
        if(other == input.end() || other->points.empty()) {continue;}
        PathPoint anchor;double nearest = .02;
        for(const auto & a:{observed.points.front(), observed.points.back()}) {
          for(const auto & b:{other->points.front(), other->points.back()}) {
            if(distance(a, b) < nearest) {
              nearest = distance(a, b);anchor = a;anchor.x = (a.x + b.x) * .5;
              anchor.y = (a.y + b.y) * .5;
            }
          }
        }
        if(nearest >= .02) {continue;}
        auto a = std::find_if(segments_.begin(), segments_.end(), [&](const auto & s){
              return s.id == ids[observed.id];
                                                                                                                      });
        auto b = std::find_if(segments_.begin(), segments_.end(), [&](const auto & s){
              return s.id == ids[neighbor];
                                                                                                                   });
        if(a == segments_.end() || b == segments_.end() || a == b) {continue;}
        const bool af = distance(a->points.front(), anchor) < distance(a->points.back(), anchor);
        const bool bf = distance(b->points.front(), anchor) < distance(b->points.back(), anchor);
        auto & pa = af ? a->points.front() : a->points.back();
        auto & pb = bf ? b->points.front() : b->points.back();
        if(distance(pa, anchor) > .06 || distance(pb, anchor) > .06) {continue;}
        const double x = .4 * (pa.x + pb.x) + .2 * anchor.x, y = .4 * (pa.y + pb.y) + .2 * anchor.y;
        pa.x = pb.x = x;pa.y = pb.y = y;
        for(auto entry:{std::pair<LineSegment *, bool>{&*a, af}, {&*b, bf}}) {
          auto & pts = entry.first->points;
          if(entry.second) {
            while(pts.size() > 2 && distance(pts[0], pts[1]) < .012) {
              pts.erase(pts.begin() + 1);
            }
          } else {
            while(pts.size() > 2 && distance(pts.back(), pts[pts.size() - 2]) < .012) {
              pts.erase(pts.end() - 2);
            }
          }
        }
      }
    }
  }
  bound();return ids;
}

void LocalPathMemory::update(const rclcpp::Time & now, const Pose2D & current)
{
  if(updated_ && now.nanoseconds() < *updated_) {reset();}
  const double dt = updated_ ? std::max(0., (now.nanoseconds() - *updated_) * 1e-9) : 0.;
  const double travel = current_ ? std::hypot(current.x - current_->x,
      current.y - current_->y) : 0.;
  const double rotation = current_ ? std::abs(std::remainder(current.yaw - current_->yaw,
      2 * M_PI)) : 0.;
  for(auto & seg:segments_) {
    for(auto & p:seg.points) {
      p.uncertainty += float(dt * .0015 + travel * .01 + rotation * .003);
      p.confidence = std::max(0.f, p.confidence - float(dt * .005 + travel * .01));
    }
  }
  updated_ = now.nanoseconds();current_ = current;prune(now);
}

void LocalPathMemory::prune(const rclcpp::Time & now)
{
  for(auto & seg:segments_) {
    std::vector<bool> valid;
    for(const auto & p:seg.points) {
      valid.push_back(rclcpp::Time(p.valid_until).nanoseconds() > now.nanoseconds() &&
          p.uncertainty < .07 &&
        (!current_ || (to_base(p, *current_).x >= -.08 && std::hypot(p.x - current_->x,
          p.y - current_->y) < 2.)));
    }
    auto first = std::find(valid.begin(), valid.end(), true);
    auto last = std::find(valid.rbegin(), valid.rend(), true);
    if(first == valid.end()) {seg.points.clear();continue;}
    const size_t a = size_t(first - valid.begin()),
      b = valid.size() - size_t(last - valid.rbegin());
  // An interior expiry would otherwise create an unobserved shortcut. Drop it conservatively.
    if(std::find(valid.begin() + a, valid.begin() + b, false) != valid.begin() + b) {
      seg.points.clear();continue;
    }
    seg.points = std::vector<PathPoint>(seg.points.begin() + a, seg.points.begin() + b);
  }
  bound();
}

void LocalPathMemory::bound()
{
  // Updating one end of a sparse track may expose an unverified gap in an
  // old tail. Keep a contiguous observed run; never bridge the gap or discard
  // the whole still-useful corner solely because of that distant tail.
  for(auto & segment:segments_) {
    size_t best_begin=0,best_end=0,begin=0;double best_range=1e9;
    for(size_t end=1;end<=segment.points.size();++end) {
      if(end<segment.points.size() && distance(segment.points[end-1],segment.points[end])<=.08) continue;
      if(end-begin>=2) {
        double range=0.;
        if(current_) {range=1e9;for(size_t k=begin;k<end;++k) range=std::min(range,std::hypot(segment.points[k].x-current_->x,segment.points[k].y-current_->y));}
        if(range<best_range-.001 || (std::abs(range-best_range)<=.001 && end-begin>best_end-best_begin)) {
          best_begin=begin;best_end=end;best_range=range;
        }
      }
      begin=end;
    }
    segment.points=std::vector<PathPoint>(segment.points.begin()+best_begin,segment.points.begin()+best_end);
  }
  segments_.erase(std::remove_if(segments_.begin(),segments_.end(),[](const auto & s){return s.points.size()<2;}),segments_.end());
  auto total = [&](){
      size_t n = 0;for(const auto & s:segments_) {
        n += s.points.size();
      }
      return n;
    };
  while(segments_.size() > max_segments_ || total() > max_points_) {
    // Blind-zone geometry is older by construction. Fresh far fragments must
    // not evict the currently useful near-field path simply due to its age.
    const auto range=[&](const auto & segment) {
      double nearest=1e9;
      if(!current_) return 0.;
      for(const auto & p:segment.points) nearest=std::min(nearest,std::hypot(p.x-current_->x,p.y-current_->y));
      return nearest;
    };
    auto oldest = std::max_element(segments_.begin(), segments_.end(),
        [&](const auto & a, const auto & b) {
          const double da=range(a),db=range(b);
          if(std::abs(da-db)>.001) return da<db;
          return rclcpp::Time(a.points.back().capture_stamp).nanoseconds() >
                 rclcpp::Time(b.points.back().capture_stamp).nanoseconds();
    });
    if(oldest == segments_.end()) {break;}
    if(segments_.size() == 1 && oldest->points.size() > max_points_) {
      oldest->points.resize(max_points_);break;
    }
    segments_.erase(oldest);
  }
  std::set<uint32_t> live;for(const auto & s:segments_) {
    live.insert(s.id);
  }
  for(auto it=locks_.begin();it!=locks_.end();) {
    if(!live.count(it->first) || !live.count(it->second)) it=locks_.erase(it);else ++it;
  }
  for(auto & s:segments_) {
    s.connected_ids.erase(std::remove_if(s.connected_ids.begin(), s.connected_ids.end(),
      [&](auto id){
        if(!live.count(id)) return true;
        const auto other=std::find_if(segments_.begin(),segments_.end(),[&](const auto & b){return b.id==id;});
        double gap=1e9;
        for(const auto & a:{s.points.front(),s.points.back()}) for(const auto & b:{other->points.front(),other->points.back()}) gap=std::min(gap,distance(a,b));
        return gap>.04;
                                                                                                                                               }),
        s.connected_ids.end());
  }
}

std::vector<PathPoint> LocalPathMemory::project_to_base(const Pose2D & current) const
{
  std::vector<PathPoint> points;for(const auto & s:segments_) {
    for(auto p:s.points) {
      points.push_back(to_base(p, current));
    }
  }
  return points;
}

std::vector<PathPoint> LocalPathMemory::path_to_base(
  const Pose2D & current,
  const rclcpp::Time & now) const
{
  decisions_.clear();
  const auto usable=[&](const PathPoint & p) {
    return p.source!=PathPoint::SOURCE_UNKNOWN && std::isfinite(p.x) && std::isfinite(p.y) &&
      std::isfinite(p.uncertainty) && p.uncertainty>=0 && p.uncertainty<.06 && rclcpp::Time(p.valid_until).nanoseconds()>now.nanoseconds();
  };
  struct Root {const LineSegment * segment;size_t index;bool reverse;double cost,length,dx,dy;};
  std::vector<Root> roots;
  const LineSegment * chosen = nullptr;size_t start = 0;bool start_reverse=false;
  for(const auto & s:segments_) {
    for(size_t k = 0; k < s.points.size(); ++k) {
      if(!usable(s.points[k])) continue;
      const auto p = to_base(s.points[k], current);
      if(p.x < -.015 || std::abs(p.y) > .20 || p.x > .45) {continue;}
      size_t before=k? k-1:k,after=k+1<s.points.size()?k+1:k;
      while(before>0 && distance(s.points[k],s.points[before])<.035) --before;
      while(after+1<s.points.size() && distance(s.points[k],s.points[after])<.035) ++after;
      const auto a=to_base(s.points[before],current),b=to_base(s.points[after],current);
      const double dx=b.x-a.x,dy=b.y-a.y;
      if(std::hypot(dx,dy)<1e-6 || std::abs(dx)<.4*std::hypot(dx,dy)) continue;
      // Even a short verified fragment can orient the stopped camera. Root
      // selection below prefers a usable aligned continuation for translation.
      double verified=0.;size_t end=k;
      if(dx<0) {while(end>0 && usable(s.points[end-1])) {verified+=distance(s.points[end],s.points[end-1]);--end;}}
      else {while(end+1<s.points.size() && usable(s.points[end+1])) {verified+=distance(s.points[end],s.points[end+1]);++end;}}
      roots.push_back({&s,k,dx<0,std::hypot(p.x,p.y),verified,dx<0?-dx:dx,dx<0?-dy:dy});
    }
  }
  if(!roots.empty()) {
    const Root * root=&*std::min_element(roots.begin(),roots.end(),[](const auto & a,const auto & b){return a.cost<b.cost;});
    if(root->length<=.055) {
      const Root * alternative=nullptr;const auto near=to_base(root->segment->points[root->index],current);
      for(const auto & candidate:roots) {
        if(candidate.length<=.055) continue;
        const auto point=to_base(candidate.segment->points[candidate.index],current);
        const double alignment=(root->dx*candidate.dx+root->dy*candidate.dy)/
          (std::hypot(root->dx,root->dy)*std::hypot(candidate.dx,candidate.dy));
        const double lateral=std::abs((point.x-near.x)*root->dy-(point.y-near.y)*root->dx)/std::hypot(root->dx,root->dy);
        if(alignment>.85 && lateral<.04 && (!alternative || candidate.cost<alternative->cost)) alternative=&candidate;
      }
      if(alternative) root=alternative;
    }
    chosen=root->segment;start=root->index;start_reverse=root->reverse;
  }
  std::vector<PathPoint> result;if(!chosen) {return result;}
  std::set<uint32_t> visited;bool reverse=start_reverse;
  for(size_t step = 0; chosen && step < max_segments_; ++step) {
    visited.insert(chosen->id);
    if(reverse) {
      for(size_t k = start + 1; k > 0; --k) {
        result.push_back(to_base(chosen->points[k - 1], current));
      }
    } else {
      for(size_t k = start; k < chosen->points.size(); ++k) {
        result.push_back(to_base(chosen->points[k], current));
      }
    }
    if(result.empty()) {break;}
    const auto endpoint = result.back();
    std::vector<const LineSegment *> neighbors;
    for(auto id:chosen->connected_ids) {
      if(!visited.count(id)) {
        auto it = std::find_if(segments_.begin(), segments_.end(), [&](const auto & s){
              return s.id == id;
                                                                                                         });
        if(it != segments_.end()) {
          auto a = to_base(it->points.front(), current), b = to_base(it->points.back(), current);
          size_t overlap = 0,comparable=0;
          for(const auto & q:it->points) {
            const auto candidate = to_base(q, current);
            // The common endpoint is required connectivity, not retracing.
            if(distance(endpoint,candidate)<.035) continue;
            ++comparable;
            if(std::any_of(result.begin(), result.end(), [&](const auto & p){
                return distance(p, candidate) < .03;
                                                                                                                 }))
            {++overlap;}
          }
          if(std::min(distance(endpoint, a), distance(endpoint, b)) < .03 &&
            comparable>0 && overlap < comparable * .25)
          {
            neighbors.push_back(&*it);
          }
        }
      }
    }
    if(neighbors.empty()) break;
    const LineSegment * next=neighbors[0];
    if(neighbors.size()>1) {
      if(result.size()<2) break;
      size_t before=result.size()-2;
      while(before>0 && distance(result[before],endpoint)<.06) --before;
      const double incoming=std::atan2(endpoint.y-result[before].y,endpoint.x-result[before].x);
      std::vector<BranchDecision> options;
      for(auto neighbor:neighbors) {
        auto a=to_base(neighbor->points.front(),current),b=to_base(neighbor->points.back(),current);
        const bool rev=distance(endpoint,b)<distance(endpoint,a);auto far=rev?a:b;
        if(rev) {for(auto it=neighbor->points.rbegin();it!=neighbor->points.rend();++it) {auto q=to_base(*it,current);if(distance(endpoint,q)>=.08) {far=q;break;}}}
        else {for(const auto & point:neighbor->points) {auto q=to_base(point,current);if(distance(endpoint,q)>=.08) {far=q;break;}}}
        const double heading=std::remainder(std::atan2(far.y-endpoint.y,far.x-endpoint.x)-incoming,2*M_PI);
        double length=0.;for(size_t j=1;j<neighbor->points.size();++j) length+=distance(neighbor->points[j-1],neighbor->points[j]);
        const bool feasible=length>.08 && std::abs(heading)<2.2;
        options.push_back({neighbor->id,length,heading,-std::abs(heading)-.01/std::max(.01,length),feasible,false,"observed leg; continuity and local length cost"});
      }
      std::vector<std::vector<size_t>> groups;
      for(size_t i=0;i<options.size();++i) {
        bool grouped=false;
        for(auto & group:groups) {
          if(std::all_of(group.begin(),group.end(),[&](size_t j) {
            return std::abs(std::remainder(options[i].heading-options[j].heading,2*M_PI))<.20 && same_observed_leg(*neighbors[i],*neighbors[j]);
          })) {group.push_back(i);grouped=true;break;}
        }
        if(!grouped) groups.push_back({i});
      }
      std::set<uint32_t> duplicates;std::unordered_map<uint32_t,uint32_t> canonical;
      const auto quality=[](const auto & s) {double u=0.;for(const auto & p:s.points) u=std::max(u,double(p.uncertainty));return u;};
      for(const auto & group:groups) {
        size_t representative=group.front();
        for(auto i:group) {
          const double u=quality(*neighbors[i]),old=quality(*neighbors[representative]);
          const bool held=locks_.count(chosen->id) && locks_.at(chosen->id)==options[i].id;
          const bool old_held=locks_.count(chosen->id) && locks_.at(chosen->id)==options[representative].id;
          if(u<old-.005 || (std::abs(u-old)<=.005 && (held || (!old_held && options[i].length>options[representative].length)))) representative=i;
        }
        for(auto i:group) {
          canonical[options[i].id]=options[representative].id;
          if(i!=representative) {duplicates.insert(options[i].id);options[i].reason="same observed leg as track "+std::to_string(options[representative].id)+"; overlapping evidence grouped";}
        }
      }
      auto best=options.end();for(auto it=options.begin();it!=options.end();++it) if(it->feasible && !duplicates.count(it->id) && (best==options.end() || it->score>best->score)) best=it;
      if(best==options.end()) {if(decisions_.empty()) decisions_=options;break;}
      if(locks_.count(chosen->id)) {
        if(canonical.count(locks_.at(chosen->id))) locks_[chosen->id]=canonical.at(locks_.at(chosen->id));
        auto held=std::find_if(options.begin(),options.end(),[&](const auto & o){return o.id==locks_.at(chosen->id) && o.feasible;});
        if(held!=options.end()) {
          // A partial view may initially look like a corner. A newly observed
          // straight continuation contradicts that provisional turn selection.
          if(std::abs(best->heading)<.30 && std::abs(held->heading)>.65 &&
            best->score>held->score+.5 && endpoint.x>.12) locks_[chosen->id]=best->id;
          else best=held;
        } else locks_.erase(chosen->id);
      }
      if(!locks_.count(chosen->id)) {
        bool ambiguous=false;for(const auto & o:options) if(o.feasible && !duplicates.count(o.id) && o.id!=best->id && best->score-o.score<.25) ambiguous=true;
        if(ambiguous) {for(auto & o:options) if(o.feasible && !duplicates.count(o.id)) o.reason="ambiguous distinct observed legs; stopped for more observations";if(decisions_.empty()) decisions_=options;break;}
        locks_[chosen->id]=best->id;
      }
      best->selected=true;best->reason=std::abs(best->heading)<.45?"locked straight-through line continuity":"locked lowest-cost observed feasible turn";
      if(decisions_.empty()) decisions_=options;
      next=*std::find_if(neighbors.begin(),neighbors.end(),[&](const auto * n){return n->id==best->id;});
    }
    chosen = next;
    auto a = to_base(chosen->points.front(), current), b = to_base(chosen->points.back(), current);
    reverse = distance(endpoint, b) < distance(endpoint, a);
    start = reverse ? chosen->points.size() - 1 : 0;
  }
  return result;
}
}  // namespace line_follower_navigation
