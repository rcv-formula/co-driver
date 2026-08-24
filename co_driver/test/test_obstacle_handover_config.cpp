#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "co_driver/config.hpp"

namespace co_driver
{
namespace
{

class RclcppScope
{
public:
  RclcppScope()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }
  }

  ~RclcppScope()
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_context_{false};
};

const DriveSpec * findDrive(const Config & config, const std::string & name)
{
  const auto it = std::find_if(
    config.drives.begin(), config.drives.end(),
    [&name](const DriveSpec & drive) {return drive.name == name;});
  return it == config.drives.end() ? nullptr : &*it;
}

TEST(ObstacleHandoverConfig, LoadsMatchingCommitVetoForPpAndLocalizationFallback)
{
  RclcppScope rclcpp_scope;
  const std::string config_dir = CO_DRIVER_TEST_CONFIG_DIR;
  const std::string topics = config_dir + "/co_driver_red_damvi_topics.jsonc";
  const std::string tuning =
    config_dir + "/localization_scoring.jsonc," +
    config_dir + "/obstacle_scoring.jsonc";

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("tuning_file", tuning),
    rclcpp::Parameter("ramp_on_return.enabled", true),
    rclcpp::Parameter("ramp_on_return.topic", "/test_launch_start_reset"),
    rclcpp::Parameter(
      "ramp_on_return.from", std::vector<std::string>{"gap_loc", "gap_obs"}),
    rclcpp::Parameter("ramp_on_return.to", "pp_main")};
  options.parameter_overrides(overrides);
  auto node = std::make_shared<rclcpp::Node>("obstacle_handover_config_test", options);

  Config config;
  std::string error;
  ASSERT_TRUE(Config::load(node.get(), topics, &config, &error)) << error;

  EXPECT_TRUE(config.ramp_on_return.enabled);
  EXPECT_EQ(config.ramp_on_return.topic, "/test_launch_start_reset");
  EXPECT_EQ(
    config.ramp_on_return.from,
    (std::vector<std::string>{"gap_loc", "gap_obs"}));
  EXPECT_EQ(config.ramp_on_return.to, "pp_main");

  const auto obstacle_input = std::find_if(
    config.inputs.begin(), config.inputs.end(),
    [](const InputSpec & input) {return input.name == "obstacles";});
  ASSERT_NE(obstacle_input, config.inputs.end());
  ASSERT_TRUE(obstacle_input->params.contains("commit_demand"));
  ASSERT_TRUE(obstacle_input->params.contains("reference_drive"));
  const double commit_demand = obstacle_input->params.at("commit_demand").get<double>();
  const double commit_score = 1.0 - commit_demand;
  EXPECT_EQ(obstacle_input->params.at("reference_drive").get<std::string>(), "pp_main");

  const DriveSpec * pp_main = findDrive(config, "pp_main");
  const DriveSpec * gap_loc = findDrive(config, "gap_loc");
  const DriveSpec * gap_obs = findDrive(config, "gap_obs");
  ASSERT_NE(pp_main, nullptr);
  ASSERT_NE(gap_loc, nullptr);
  ASSERT_NE(gap_obs, nullptr);

  const Influence & pp_gate = pp_main->influence.at("obstacles");
  const Influence & loc_gate = gap_loc->influence.at("obstacles");
  const Influence & obs_preference = gap_obs->influence.at("obstacles");
  EXPECT_DOUBLE_EQ(commit_demand, 0.85);
  EXPECT_DOUBLE_EQ(pp_gate.weight, 0.0);
  EXPECT_DOUBLE_EQ(loc_gate.weight, 0.0);
  EXPECT_DOUBLE_EQ(pp_gate.veto_below, 0.15);
  EXPECT_DOUBLE_EQ(loc_gate.veto_below, 0.15);
  EXPECT_DOUBLE_EQ(pp_gate.veto_below, commit_score);
  EXPECT_DOUBLE_EQ(loc_gate.veto_below, pp_gate.veto_below);
  EXPECT_FALSE(pp_gate.last_resort_ok);
  EXPECT_FALSE(loc_gate.last_resort_ok);
  EXPECT_LT(obs_preference.veto_below, 0.0);
  EXPECT_DOUBLE_EQ(obs_preference.weight, -2.0);
  EXPECT_EQ(gap_loc->topic, "/drive_gf");
  EXPECT_EQ(gap_obs->topic, "/drive_gf2");
}

}  // namespace
}  // namespace co_driver
