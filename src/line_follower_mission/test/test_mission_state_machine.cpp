#include <gtest/gtest.h>

#include "line_follower_interfaces/msg/mission_state.hpp"
#include "line_follower_mission/finish_judge.hpp"
#include "line_follower_mission/mission_state_machine.hpp"

using line_follower_interfaces::msg::MissionState;
using namespace line_follower_mission;

namespace
{
MissionInputs at(double t, bool ready = true)
{
  MissionInputs in;
  in.elapsed_sec = t;
  in.sensors_ready = ready;
  in.sensors_lost = !ready;
  return in;
}

MissionStateMachine running_machine()
{
  MissionStateMachine m(MissionParams{});
  m.step(at(20.0));
  m.step(at(20.1));
  m.step(at(20.2));
  return m;
}
}  // namespace

TEST(MissionStateMachine, NeverDrivesBefore20Seconds)  // T-03
{
  MissionStateMachine m(MissionParams{});
  for (double t = 0.0; t < 20.0; t += 0.1) {
    m.step(at(t));
    EXPECT_EQ(m.state(), MissionState::STARTUP_WAIT) << "t=" << t;
    EXPECT_FALSE(m.drive_enabled()) << "t=" << t;
  }
}

TEST(MissionStateMachine, AutoStartsAfterDelayWhenSensorsReady)  // T-04
{
  MissionStateMachine m(MissionParams{});
  EXPECT_EQ(m.step(at(20.0)), MissionState::CHECKING);
  EXPECT_EQ(m.step(at(20.1)), MissionState::READY);
  EXPECT_EQ(m.step(at(20.2)), MissionState::RUNNING);
  EXPECT_TRUE(m.drive_enabled());
}

TEST(MissionStateMachine, SensorDropInReadyReturnsToChecking)
{
  MissionStateMachine m(MissionParams{});
  m.step(at(20.0));
  EXPECT_EQ(m.step(at(20.1)), MissionState::READY);
  EXPECT_EQ(m.step(at(20.2, false)), MissionState::CHECKING);
  EXPECT_FALSE(m.drive_enabled());
  EXPECT_EQ(m.step(at(20.3)), MissionState::READY);
}

TEST(MissionStateMachine, WaitsInCheckingThenFaults)
{
  MissionStateMachine m(MissionParams{});
  m.step(at(20.0, false));
  EXPECT_EQ(m.step(at(40.0, false)), MissionState::CHECKING);
  EXPECT_EQ(m.step(at(50.5, false)), MissionState::FAULT);
  EXPECT_EQ(m.step(at(50.6, false)), MissionState::STOPPED);
  EXPECT_FALSE(m.drive_enabled());
}

TEST(MissionStateMachine, ManualStartRequiredWhenAutoStartDisabled)
{
  MissionStateMachine m(MissionParams{20.0, 30.0, false});
  m.step(at(20.0));
  m.step(at(20.1));
  EXPECT_EQ(m.step(at(20.2)), MissionState::READY);
  auto in = at(20.3);
  in.start_requested = true;
  EXPECT_EQ(m.step(in), MissionState::RUNNING);
}

TEST(MissionStateMachine, SensorLossWhileRunningFaults)
{
  auto m = running_machine();
  EXPECT_EQ(m.step(at(21.0, false)), MissionState::FAULT);
  EXPECT_FALSE(m.drive_enabled());
}

TEST(MissionStateMachine, FinishStopsThenFinishes)
{
  auto m = running_machine();
  auto in = at(30.0);
  in.finish_confirmed = true;
  EXPECT_EQ(m.step(in), MissionState::STOPPING);
  EXPECT_FALSE(m.drive_enabled());
  in.robot_stopped = true;
  EXPECT_EQ(m.step(in), MissionState::FINISHED);
}

TEST(MissionStateMachine, KillGoesToEStopThenStopped)
{
  auto m = running_machine();
  auto in = at(25.0);
  in.kill_requested = true;
  EXPECT_EQ(m.step(in), MissionState::E_STOP);
  EXPECT_EQ(m.step(in), MissionState::STOPPED);
  EXPECT_EQ(m.step(at(26.0)), MissionState::STOPPED);
}

TEST(FinishJudge, IgnoresStartMarker)  // T-13
{
  FinishJudge judge(1.0);
  EXPECT_FALSE(judge.armed(0.0));
  EXPECT_FALSE(judge.update(true, 0.0));
}
