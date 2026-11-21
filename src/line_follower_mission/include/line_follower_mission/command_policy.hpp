#pragma once
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include "line_follower_interfaces/msg/mission_state.hpp"

namespace line_follower_mission
{
struct CommandRequest {std::string request_id; std::string run_id; uint8_t command;};
struct CommandReply
{
  bool accepted{false};
  std::string reason;
  uint8_t state{0};
  std::string run_id;
  bool execute{false};
};

// Bounded per-run idempotency cache, including rejections. Duplicate replies keep
// the original state/run/reason but never execute a second side effect.
class CommandPolicy
{
public:
  explicit CommandPolicy(size_t capacity = 256) : capacity_(capacity ? capacity : 1) {}
  size_t size() const {return cache_.size();}
  CommandReply handle(
    const CommandRequest & req, const std::string & run_id, uint8_t state,
    bool auto_start, bool pending = false)
  {
    using S = line_follower_interfaces::msg::MissionState;
    const auto seen = seen_.find(req.request_id);
    if (seen != seen_.end() && seen->second != req.command) {
      return CommandReply{false, "request_id reused with different command", state, run_id, false};
    }
    const auto found = cache_.find(req.request_id);
    if (found != cache_.end()) {
      auto reply = found->second;
      reply.execute = false;
      return reply;
    }
    CommandReply reply{false, "", state, run_id, false};
    if (req.request_id.empty()) {
      reply.reason = "request_id required";
      return reply;
    }
    if (seen != seen_.end()) {
      reply.reason = "expired duplicate request_id";
      return reply;
    }
    seen_[req.request_id] = req.command;
    if (!req.run_id.empty() && req.run_id != run_id) {
      reply.reason = "run_id mismatch";
    } else if (req.command == 0) {
      if (auto_start) {reply.reason = "auto_start enabled";}
      else if (state != S::READY) {reply.reason = "mission not READY";}
      else if (pending) {reply.reason = "command already pending";}
      else {reply.accepted = true; reply.reason = "start requested";}
    } else if (req.command == 1) {
      if (state == S::FINISHED || state == S::STOPPED || state == S::FAULT || state == S::E_STOP) {
        reply.reason = "mission already inactive";
      } else {reply.accepted = true; reply.reason = "kill requested (software stop, not a physical E-STOP)";}
    } else {reply.reason = "unknown command";}
    reply.execute = reply.accepted;
    if (order_.size() >= capacity_) {cache_.erase(order_.front()); order_.pop_front();}
    order_.push_back(req.request_id);
    cache_[req.request_id] = reply;
    return reply;
  }
private:
  size_t capacity_;
  std::deque<std::string> order_;
  std::unordered_map<std::string, CommandReply> cache_;
  std::unordered_map<std::string, uint8_t> seen_;
};
}  // namespace line_follower_mission
