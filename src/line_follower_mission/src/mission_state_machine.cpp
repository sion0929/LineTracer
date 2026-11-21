#include "line_follower_mission/mission_state_machine.hpp"

#include "line_follower_interfaces/msg/mission_state.hpp"

namespace line_follower_mission
{

using line_follower_interfaces::msg::MissionState;

bool MissionStateMachine::drive_enabled() const
{
  return state_ == MissionState::RUNNING;
}

void MissionStateMachine::transition(uint8_t to, const std::string & reason)
{
  state_ = to;
  reason_ = reason;
}

uint8_t MissionStateMachine::step(const MissionInputs & in)
{
  const bool active = state_ != MissionState::FINISHED && state_ != MissionState::STOPPED &&
    state_ != MissionState::E_STOP && state_ != MissionState::FAULT;
  if (in.kill_requested && active) {
    transition(MissionState::E_STOP, "kill requested");
    return state_;
  }

  switch (state_) {
    case MissionState::STARTUP_WAIT:
      if (in.elapsed_sec >= params_.startup_delay_sec) {
        checking_since_ = in.elapsed_sec;
        transition(MissionState::CHECKING, "startup delay elapsed, checking sensors");
      }
      break;
    case MissionState::CHECKING:
      if (in.sensors_ready) {
        transition(MissionState::READY, "sensors ready");
      } else if (in.elapsed_sec - checking_since_ > params_.check_timeout_sec) {
        transition(MissionState::FAULT, "sensor check timed out");
      }
      break;
    case MissionState::READY:
      if (!in.sensors_ready) {
        checking_since_ = in.elapsed_sec;
        transition(MissionState::CHECKING, "sensors dropped before start, re-checking");
      } else if (params_.auto_start || in.start_requested) {
        transition(MissionState::RUNNING, params_.auto_start ? "auto start" : "start requested");
      }
      break;
    case MissionState::RUNNING:
      if (in.sensors_lost) {
        transition(MissionState::FAULT, "sensor lost while running");
      } else if (in.finish_confirmed) {
        transition(MissionState::STOPPING, "finish line crossed");
      }
      break;
    case MissionState::STOPPING:
      if (in.robot_stopped) {
        transition(MissionState::FINISHED, "stopped after finish");
      }
      break;
    case MissionState::FAULT:
      transition(MissionState::STOPPED, "fault: " + reason_);
      break;
    case MissionState::E_STOP:
      transition(MissionState::STOPPED, "e-stop: waiting for manual recovery");
      break;
    default:
      break;
  }
  return state_;
}

}  // namespace line_follower_mission
