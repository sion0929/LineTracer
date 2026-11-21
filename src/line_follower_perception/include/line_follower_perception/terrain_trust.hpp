#pragma once
#include <cmath>
namespace line_follower_perception
{
// Relative elevation is a projection-trust signal, not new path geometry.
// A level chassis on an elevated surface does not establish the original plane.
class TerrainTrust
{
public:
  void update(double pitch,double travel,double speed)
  {
    if(!initialized_) {baseline_=pitch;initialized_=true;}
    if(speed<.003 && std::abs(pitch-baseline_)<.02 && std::abs(height_)<.015) {
      baseline_=.98*baseline_+.02*pitch;height_*=.99;
    }
    pitch_=pitch;height_-=travel*std::sin(pitch-baseline_);
  }
  bool trusted() const {return initialized_ && std::abs(height_)<.02 && std::abs(pitch_-baseline_)<.035;}
  double height() const {return height_;}
private:
  double height_{0.},baseline_{0.},pitch_{0.};bool initialized_{false};
};
}
