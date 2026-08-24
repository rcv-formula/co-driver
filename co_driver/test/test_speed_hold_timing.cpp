#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "co_driver/handover_state.hpp"
#include "co_driver/speed_hold_timing.hpp"

namespace co_driver
{
namespace
{

TEST(SpeedHoldTiming, AddsFiniteBlendAndConservativeRateLimitSettleTime)
{
  const auto budget = estimateSpeedTransitionBudget(
    1.3, 3.7, true,
    {{0.3, false}},
    {{6.0, 12.0}},
    {true, 0.0, 8.0});

  ASSERT_TRUE(budget.bounded);
  EXPECT_NEAR(budget.blend_s, 0.3, 1.0e-12);
  EXPECT_NEAR(budget.rate_limit_s, 4.3 / 6.0, 1.0e-12);
  EXPECT_NEAR(budget.total_s(), 0.3 + 4.3 / 6.0, 1.0e-12);
}

TEST(SpeedHoldTiming, DoesNotChargeSwitchBlendWithoutARealSourceSwitch)
{
  const auto budget = estimateSpeedTransitionBudget(
    1.3, 3.7, false,
    {{0.3, false}},
    {{6.0, 12.0}},
    {true, 0.0, 8.0});

  ASSERT_TRUE(budget.bounded);
  EXPECT_DOUBLE_EQ(budget.blend_s, 0.0);
  EXPECT_NEAR(budget.rate_limit_s, 4.3 / 6.0, 1.0e-12);
}

TEST(SpeedHoldTiming, UsesSlowestEnabledSpeedRateAcrossAllStagesAndDirections)
{
  const auto budget = estimateSpeedTransitionBudget(
    4.0, 1.0, false,
    {},
    {{20.0, 12.0}, {8.0, 10.0}},
    {true, -2.0, 5.0});

  ASSERT_TRUE(budget.bounded);
  EXPECT_NEAR(budget.rate_limit_s, 4.0 / 8.0, 1.0e-12);
}

TEST(SpeedHoldTiming, DisabledRateLimitsDoNotAddTime)
{
  const auto budget = estimateSpeedTransitionBudget(
    0.0, 8.0, true,
    {{0.25, false}},
    {{0.0, -1.0},
      {std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}},
    {});

  ASSERT_TRUE(budget.bounded);
  EXPECT_DOUBLE_EQ(budget.rate_limit_s, 0.0);
  EXPECT_DOUBLE_EQ(budget.total_s(), 0.25);
}

TEST(SpeedHoldTiming, EmaBlendHasNoFiniteTransitionBound)
{
  const auto budget = estimateSpeedTransitionBudget(
    1.0, 2.0, true,
    {{0.3, true}},
    {{6.0, 12.0}},
    {true, 0.0, 8.0});

  EXPECT_FALSE(budget.bounded);
  EXPECT_FALSE(std::isfinite(budget.total_s()));
  EXPECT_NE(budget.reason.find("ema"), std::string::npos);
}

TEST(SpeedHoldTiming, ReceiverLimitNeverSilentlyShortensSteadyHold)
{
  const auto budget = estimateSpeedTransitionBudget(
    1.3, 3.7, true,
    {{0.1, false}},
    {{6.0, 12.0}},
    {true, 0.0, 8.0});
  double request_s = -1.0;

  EXPECT_TRUE(speedHoldRequestDuration(0.8, budget, 10.0, &request_s));
  EXPECT_NEAR(request_s, 0.8 + 0.1 + 4.3 / 6.0, 1.0e-12);

  request_s = -1.0;
  EXPECT_FALSE(speedHoldRequestDuration(9.4, budget, 10.0, &request_s));
  EXPECT_DOUBLE_EQ(request_s, -1.0);
}

TEST(SpeedHoldTiming, PathPauseUsesWorstFinalOutputEndpointNotHandbackSnapshot)
{
  // At handback /drive is already 3.7, but while pure_pursuit has no Path its
  // hold clock is paused and the cached PP command may take /drive anywhere in
  // the configured [0, 8] clamp. The budget must still cover the far endpoint.
  const auto budget = estimateSpeedTransitionBudget(
    3.7, 3.7, true,
    {{0.3, false}},
    {{6.0, 12.0}},
    {true, 0.0, 8.0});

  ASSERT_TRUE(budget.bounded);
  EXPECT_NEAR(budget.rate_limit_s, 4.3 / 6.0, 1.0e-12);
}

TEST(SpeedHoldTiming, RateLimitedPipelineWithoutFiniteOutputBoundsIsRejected)
{
  const auto budget = estimateSpeedTransitionBudget(
    1.0, 2.0, true,
    {{0.3, false}},
    {{6.0, 12.0}},
    {});

  EXPECT_FALSE(budget.bounded);
  EXPECT_FALSE(std::isfinite(budget.total_s()));
  EXPECT_NE(budget.reason.find("no finite final speed bounds"), std::string::npos);
}

TEST(HandoverState, NoSelectionTickPreservesGapSourceForNextHandback)
{
  std::string selected;
  selected = rememberSelectedDrive(selected, true, "gap_obs");
  selected = rememberSelectedDrive(selected, false, "");

  ASSERT_EQ(selected, "gap_obs");
  const std::string handed_from = selected;
  selected = rememberSelectedDrive(selected, true, "pp_main");

  EXPECT_TRUE(isConfiguredHandback(
    true, false, handed_from, selected, {"gap_loc", "gap_obs"}, "pp_main"));
}

TEST(HandoverState, MasterFalseDisablesEveryHandbackAction)
{
  const HandbackActions actions = effectiveHandbackActions(
    false, true, true, true);

  EXPECT_FALSE(actions.ramp);
  EXPECT_FALSE(actions.speed_hold);
  EXPECT_FALSE(actions.gain);
}

TEST(HandoverState, FeatureSwitchesRemainIndependentUnderEnabledMaster)
{
  const HandbackActions actions = effectiveHandbackActions(
    true, false, true, false);

  EXPECT_FALSE(actions.ramp);
  EXPECT_TRUE(actions.speed_hold);
  EXPECT_FALSE(actions.gain);
}

TEST(HandoverState, HandbackStillRequiresARealNonLastResortSwitch)
{
  const std::vector<std::string> gap_drives{"gap_loc", "gap_obs"};

  EXPECT_FALSE(isConfiguredHandback(
    false, false, "gap_obs", "pp_main", gap_drives, "pp_main"));
  EXPECT_FALSE(isConfiguredHandback(
    true, true, "gap_obs", "pp_main", gap_drives, "pp_main"));
  EXPECT_FALSE(isConfiguredHandback(
    true, false, "pp_main", "pp_main", gap_drives, "pp_main"));
}

}  // namespace
}  // namespace co_driver
