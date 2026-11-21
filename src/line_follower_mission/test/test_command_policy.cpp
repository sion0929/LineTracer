#include <gtest/gtest.h>
#include "line_follower_mission/command_policy.hpp"
#include "line_follower_interfaces/msg/mission_state.hpp"
using namespace line_follower_mission;
using S = line_follower_interfaces::msg::MissionState;

TEST(CommandPolicy, StartRulesAndWrongRun)
{
  CommandPolicy p(3);
  EXPECT_FALSE(p.handle({"a", "other", 0}, "run", S::READY, false).accepted);
  EXPECT_FALSE(p.handle({"b", "run", 0}, "run", S::READY, true).accepted);
  EXPECT_FALSE(p.handle({"c", "run", 0}, "run", S::STARTUP_WAIT, false).accepted);
  EXPECT_TRUE(p.handle({"d", "run", 0}, "run", S::READY, false).execute);
  EXPECT_FALSE(p.handle({"e", "run", 0}, "run", S::READY, false, true).accepted);
}
TEST(CommandPolicy, DuplicateReturnsOriginalWithoutExecution)
{
  CommandPolicy p;
  const auto original = p.handle({"a", "run", 1}, "run", S::RUNNING, true);
  EXPECT_TRUE(original.accepted);
  EXPECT_TRUE(original.execute);
  const auto duplicate = p.handle({"a", "run", 1}, "run", S::STOPPED, true);
  EXPECT_TRUE(duplicate.accepted);
  EXPECT_FALSE(duplicate.execute);
  EXPECT_EQ(original.reason, duplicate.reason);
  EXPECT_EQ(original.state, duplicate.state);
  EXPECT_EQ(original.run_id, duplicate.run_id);
}
TEST(CommandPolicy, RejectionsAreCachedAndCacheIsBounded)
{
  CommandPolicy p(2);
  EXPECT_FALSE(p.handle({"a", "run", 0}, "run", S::STARTUP_WAIT, false).accepted);
  EXPECT_FALSE(p.handle({"a", "run", 0}, "run", S::READY, false).accepted);
  p.handle({"b", "run", 0}, "run", S::READY, false);
  p.handle({"c", "run", 0}, "run", S::READY, false);
  EXPECT_FALSE(p.handle({"a", "run", 0}, "run", S::READY, false).accepted);
  EXPECT_EQ(p.handle({"a", "run", 0}, "run", S::READY, false).reason, "expired duplicate request_id");
  EXPECT_LE(p.size(), 2u);
}
TEST(CommandPolicy, InactiveKillAndInvalidRequests)
{
  CommandPolicy p;
  for (auto state : {S::FINISHED, S::STOPPED, S::FAULT, S::E_STOP}) {
    EXPECT_FALSE(p.handle({std::to_string(state), "run", 1}, "run", state, true).accepted);
  }
  EXPECT_FALSE(p.handle({"", "run", 1}, "run", S::READY, false).accepted);
  EXPECT_TRUE(p.handle({"kill-overrides-start", "run", 1}, "run", S::READY, false, true).accepted);
  EXPECT_FALSE(p.handle({"unknown", "run", 99}, "run", S::READY, false).accepted);
}

TEST(CommandPolicy, ReusedIdWithDifferentCommandRejected)
{
  CommandPolicy p;
  EXPECT_TRUE(p.handle({"same", "run", 0}, "run", S::READY, false).accepted);
  const auto reply = p.handle({"same", "run", 1}, "run", S::RUNNING, false);
  EXPECT_FALSE(reply.accepted);
  EXPECT_FALSE(reply.execute);
  EXPECT_EQ(reply.reason, "request_id reused with different command");
}
