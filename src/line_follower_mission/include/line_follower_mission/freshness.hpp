#pragma once
#include <string>
#include <unordered_map>

namespace line_follower_mission
{
class Freshness
{
public:
  void receive(const std::string & source, double wall_sec) {received_[source] = wall_sec;}
  bool valid(const std::string & source, double wall_sec, double timeout) const
  {
    const auto it = received_.find(source);
    return it != received_.end() && wall_sec-it->second >= 0.0 && wall_sec-it->second <= timeout;
  }
private:
  std::unordered_map<std::string, double> received_;
};
}  // namespace line_follower_mission
