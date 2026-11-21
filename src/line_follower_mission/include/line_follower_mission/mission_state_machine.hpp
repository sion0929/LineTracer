#pragma once

#include <cstdint>
#include <string>

namespace line_follower_mission
{

struct MissionParams
{
  double startup_delay_sec{20.0};  // R-04: no driving before this (steady time since launch)
  double check_timeout_sec{30.0};  // CHECKING -> FAULT if sensors never become ready
  bool auto_start{true};           // competition: start without any GCS command
};

struct MissionInputs
{
  double elapsed_sec{0.0};         // steady time since mission_manager start
  bool sensors_ready{false};       // camera pipeline, odom, imu all fresh
  bool sensors_lost{false};        // a sensor went stale while running
  bool finish_confirmed{false};    // robot center crossed the finish line (internal criterion)
  bool robot_stopped{false};
  bool start_requested{false};     // dev only (auto_start=false)
  bool kill_requested{false};      // dev only, software stop request
};

// Mission state machine (design §9.1). Values are MissionState.msg constants.
// One transition per step(); drive is enabled only in RUNNING.
class MissionStateMachine
{
public:
  explicit MissionStateMachine(MissionParams params)
  : params_(params) {}

  uint8_t step(const MissionInputs & in);
  uint8_t state() const {return state_;}
  bool drive_enabled() const;
  const std::string & reason() const {return reason_;}

private:
  void transition(uint8_t to, const std::string & reason);

  MissionParams params_;
  uint8_t state_{0};
  double checking_since_{0.0};
  std::string reason_{"waiting for startup delay"};
};

}  // namespace line_follower_mission
