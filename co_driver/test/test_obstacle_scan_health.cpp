#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include "co_driver/scorer.hpp"

namespace co_driver
{
namespace
{

using namespace std::chrono_literals;

class ObstacleScanHealthTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }
  }

  static void TearDownTestSuite()
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    const std::string suffix = std::to_string(++instance_);
    scan_topic_ = "/test_obstacle_scan_health/scan_" + suffix;
    path_topic_ = "/test_obstacle_scan_health/path_" + suffix;
    vesc_topic_ = "/test_obstacle_scan_health/vesc_" + suffix;
    node_ = std::make_shared<rclcpp::Node>("obstacle_scan_health_" + suffix);

    scorer_ = ScorerRegistry::instance().create("obstacle_avoid");
    ASSERT_NE(scorer_, nullptr);
    const auto group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    scorer_->setContext("obstacles", "obstacle_avoid", group);

    const Json params = {
      {"source", "scan"},
      {"scan_topic", scan_topic_},
      {"path_topic", path_topic_},
      {"path_transient_local", false},
      {"reference_drive", "pp_main"},
      {"vesc_state_topic", vesc_topic_},
      {"speed_to_erpm_gain", 1.0},
      {"speed_to_erpm_offset", 0.0},
      {"wheel_speed_deadband", 0.0},
      {"wheel_speed_scale", 1.0},
      {"speed_timeout_ms", 200.0},
      {"base_frame", "map"},
      {"deskew", false},
      {"timeout_ms", 500.0},
      {"min_speed", 0.2},
      {"trigger_time", 2.5},
      {"min_trigger_m", 3.0},
      {"max_trigger_m", 14.0},
      {"reaction_time", 0.8},
      {"lateral_accel", 5.0},
      {"commit_margin_m", 0.8},
      {"commit_demand", 0.85},
      {"beam_stride", 1},
      {"scans_kept", 5},
      {"points_per_scan", 1},
      {"dense_one_scan", 1},
      {"one_scan_within_m", 10.0},
      {"block_ms", 0.0},
      {"clear_ms", 0.0}
    };
    ASSERT_TRUE(scorer_->configure(node_.get(), "obstacles", params));

    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS());
    rclcpp::QoS path_qos(1);
    path_qos.reliable().durability_volatile();
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);
    vesc_pub_ = node_->create_publisher<vesc_msgs::msg::VescStateStamped>(
      vesc_topic_, rclcpp::QoS(10));
    executor_.add_node(node_);

    ASSERT_TRUE(spinUntil([this]() {
      return scan_pub_->get_subscription_count() > 0 &&
             path_pub_->get_subscription_count() > 0 &&
             vesc_pub_->get_subscription_count() > 0;
    }));
  }

  void TearDown() override
  {
    scorer_.reset();
    scan_pub_.reset();
    path_pub_.reset();
    vesc_pub_.reset();
    executor_.remove_node(node_);
    node_.reset();
  }

  bool spinUntil(const std::function<bool()> & done, std::chrono::milliseconds limit = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(10ms);
      if (done()) {return true;}
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some(10ms);
    return done();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(10ms);
      std::this_thread::sleep_for(5ms);
    }
  }

  nav_msgs::msg::Path straightPath(double y = 0.0) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();
    for (double x : {0.0, 10.0}) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  sensor_msgs::msg::LaserScan scan(
    const std::string & frame, const std::vector<float> & ranges) const
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = frame;
    msg.header.stamp = node_->now();
    msg.angle_min = -0.01F;
    msg.angle_max = 0.01F;
    msg.angle_increment = ranges.size() > 1 ?
      (msg.angle_max - msg.angle_min) / static_cast<float>(ranges.size() - 1) : 0.01F;
    msg.range_min = 0.05F;
    msg.range_max = 10.0F;
    msg.ranges = ranges;
    return msg;
  }

  ScoreResult score()
  {
    Drive drive;
    drive.name = "pp_main";
    drive.cmd.drive.speed = 1.0;
    drive.cmd.drive.steering_angle = 0.0;
    Context context;
    context.now = node_->now();
    return scorer_->score(drive, context);
  }

  Drive liveDrive(const std::string & name, double speed) const
  {
    Drive drive;
    drive.name = name;
    drive.enabled = true;
    drive.hold = 5.0;
    drive.has_cmd = true;
    drive.last_rx = node_->now();
    drive.cmd.drive.speed = speed;
    drive.cmd.drive.steering_angle = 0.0;
    return drive;
  }

  Context contextFor(const std::vector<Drive> & drives) const
  {
    Context context;
    context.now = node_->now();
    context.drives = &drives;
    return context;
  }

  void publishVescSpeed(double speed)
  {
    vesc_msgs::msg::VescStateStamped msg;
    msg.header.stamp = node_->now();
    msg.state.speed = speed;
    vesc_pub_->publish(msg);
  }

  inline static bool owns_context_{false};
  inline static int instance_{0};
  std::string scan_topic_;
  std::string path_topic_;
  std::string vesc_topic_;
  rclcpp::Node::SharedPtr node_;
  ScorerPtr scorer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(ObstacleScanHealthTest, ReportsReceivedScanWhenPathIsUnavailable)
{
  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);

  const ScoreResult result = score();
  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.value, 1.0);
  EXPECT_NE(result.note.find("scan received but no planned path"), std::string::npos);
  EXPECT_EQ(result.note.find("no scan yet"), std::string::npos);
  EXPECT_NE(result.note.find("nothing can be seen"), std::string::npos);
}

TEST_F(ObstacleScanHealthTest, VolatilePathUpdatesReplaceThePreviousPlan)
{
  // The first live plan is well away from the scan return, so the path is
  // clear. Publishing a second plan on the same volatile topic must replace it
  // immediately; the same return then sits on the new path and blocks it.
  path_pub_->publish(straightPath(3.0));
  spinFor(100ms);
  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);
  const ScoreResult old_plan = score();
  ASSERT_DOUBLE_EQ(old_plan.value, 1.0) << old_plan.note;

  path_pub_->publish(straightPath(0.0));
  spinFor(100ms);
  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);
  const ScoreResult updated_plan = score();
  EXPECT_DOUBLE_EQ(updated_plan.value, 0.0) << updated_plan.note;
  EXPECT_NE(updated_plan.note.find("committed"), std::string::npos) << updated_plan.note;
}

TEST_F(ObstacleScanHealthTest, FailedLatestScanDropsAccumulatedOccupancy)
{
  path_pub_->publish(straightPath());
  spinFor(100ms);

  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);
  const ScoreResult obstacle = score();
  ASSERT_DOUBLE_EQ(obstacle.value, 0.0) << obstacle.note;
  ASSERT_NE(obstacle.note.find("committed"), std::string::npos) << obstacle.note;

  scan_pub_->publish(scan("missing_lidar", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);
  const ScoreResult blind = score();
  EXPECT_DOUBLE_EQ(blind.value, 1.0);
  EXPECT_NE(blind.note.find("cannot place the scan"), std::string::npos);
  EXPECT_NE(blind.note.find("nothing can be seen"), std::string::npos);

  const float no_return = std::numeric_limits<float>::infinity();
  scan_pub_->publish(scan("map", {no_return, no_return, no_return}));
  spinFor(100ms);
  const ScoreResult recovered = score();
  EXPECT_DOUBLE_EQ(recovered.value, 1.0);
  EXPECT_NE(recovered.note.find("clear (0 clusters considered"), std::string::npos) <<
    recovered.note;
  EXPECT_EQ(recovered.note.find("obstacle"), std::string::npos) << recovered.note;
}

TEST_F(ObstacleScanHealthTest, FreshZeroVescOverridesFastReferenceCommand)
{
  path_pub_->publish(straightPath());
  spinFor(100ms);

  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  publishVescSpeed(0.0);
  spinFor(50ms);

  std::vector<Drive> drives{
    liveDrive("pp_main", 8.0),
    liveDrive("gap_loc", 2.0)};
  const Context context = contextFor(drives);
  scorer_->prepare(context, drives);

  const ScoreResult result = scorer_->score(drives[1], context);
  EXPECT_TRUE(result.available);
  EXPECT_DOUBLE_EQ(result.value, 1.0);
  EXPECT_NE(result.note.find("below min_speed"), std::string::npos) << result.note;
}

TEST_F(ObstacleScanHealthTest, StaleVescReferenceDefersMidEvaluationUpdate)
{
  path_pub_->publish(straightPath());
  publishVescSpeed(0.0);
  spinFor(250ms);  // speed_timeout_ms=200: the received sample is now stale.

  const float no_return = std::numeric_limits<float>::infinity();
  scan_pub_->publish(scan("map", {no_return, no_return, no_return}));
  spinFor(100ms);

  std::vector<Drive> drives{
    liveDrive("pp_main", 8.0),
    liveDrive("gap_loc", 2.0)};

  // Prime the scan horizon with the stale-VESC fallback. Without the common
  // reference this would end at gap_loc's 2 m/s when candidates are scored in
  // order and the next scan would discard the obstacle at 6 m before scoring.
  Context context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult primed_pp = scorer_->score(drives[0], context);
  ASSERT_DOUBLE_EQ(primed_pp.value, 1.0) << primed_pp.note;
  const ScoreResult primed_gap_loc = scorer_->score(drives[1], context);
  ASSERT_DOUBLE_EQ(primed_gap_loc.value, 1.0) << primed_gap_loc.note;

  scan_pub_->publish(scan("map", {6.0F, 6.0F, 6.0F}));
  spinFor(100ms);

  context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult pp = scorer_->score(drives[0], context);

  // A fresh stop sample lands between candidate scores. It must not make the
  // second candidate see a different scene inside this evaluation transaction.
  publishVescSpeed(0.0);
  spinFor(50ms);
  const ScoreResult gap_loc = scorer_->score(drives[1], context);

  EXPECT_DOUBLE_EQ(pp.value, 0.0) << pp.note;
  EXPECT_NE(pp.note.find("committed"), std::string::npos) << pp.note;
  EXPECT_DOUBLE_EQ(gap_loc.value, 0.0) << gap_loc.note;
  EXPECT_NE(gap_loc.note.find("committed"), std::string::npos) << gap_loc.note;

  // The new VESC value becomes visible at the next evaluation boundary and,
  // because fresh measured zero has priority, clears the obstacle input.
  context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult next_gap_loc = scorer_->score(drives[1], context);
  EXPECT_DOUBLE_EQ(next_gap_loc.value, 1.0) << next_gap_loc.note;
  EXPECT_NE(next_gap_loc.note.find("below min_speed"), std::string::npos) <<
    next_gap_loc.note;
}

TEST_F(ObstacleScanHealthTest, InvalidNamedReferenceUsesFastestFiniteLiveDrive)
{
  path_pub_->publish(straightPath());
  publishVescSpeed(0.0);
  spinFor(250ms);

  const float no_return = std::numeric_limits<float>::infinity();
  scan_pub_->publish(scan("map", {no_return, no_return, no_return}));
  spinFor(100ms);

  std::vector<Drive> drives{
    liveDrive("pp_main", std::numeric_limits<double>::quiet_NaN()),
    liveDrive("gap_loc", 8.0),
    liveDrive("gap_obs", 4.0)};
  drives[0].cmd.drive.steering_angle = std::numeric_limits<double>::infinity();

  // pp_main cannot be the geometry reference. prepare() must reject its NaN
  // speed / infinite steering and choose the fastest finite live command
  // (gap_loc=8), including for gap_obs whose own 4 m/s would not commit at 6 m.
  Context context = contextFor(drives);
  scorer_->prepare(context, drives);
  scan_pub_->publish(scan("map", {6.0F, 6.0F, 6.0F}));
  spinFor(100ms);

  context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult result = scorer_->score(drives[2], context);
  EXPECT_TRUE(result.available) << result.note;
  EXPECT_DOUBLE_EQ(result.value, 0.0) << result.note;
  EXPECT_NE(result.note.find("committed"), std::string::npos) << result.note;
}

TEST_F(ObstacleScanHealthTest, NewValidScanWaitsForNextPrepare)
{
  path_pub_->publish(straightPath());
  spinFor(100ms);

  const float no_return = std::numeric_limits<float>::infinity();
  scan_pub_->publish(scan("map", {no_return, no_return, no_return}));
  spinFor(100ms);

  std::vector<Drive> drives{
    liveDrive("pp_main", 8.0),
    liveDrive("gap_loc", 2.0)};
  Context context = contextFor(drives);
  scorer_->prepare(context, drives);  // freezes the empty scan for this tick

  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);

  const ScoreResult same_tick_pp = scorer_->score(drives[0], context);
  const ScoreResult same_tick_gap = scorer_->score(drives[1], context);
  EXPECT_DOUBLE_EQ(same_tick_pp.value, 1.0) << same_tick_pp.note;
  EXPECT_DOUBLE_EQ(same_tick_gap.value, 1.0) << same_tick_gap.note;

  context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult next_tick_pp = scorer_->score(drives[0], context);
  const ScoreResult next_tick_gap = scorer_->score(drives[1], context);
  EXPECT_DOUBLE_EQ(next_tick_pp.value, 0.0) << next_tick_pp.note;
  EXPECT_DOUBLE_EQ(next_tick_gap.value, 0.0) << next_tick_gap.note;
}

TEST_F(ObstacleScanHealthTest, InvalidationAfterSnapshotCannotResurrectCommit)
{
  path_pub_->publish(straightPath());
  spinFor(100ms);

  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);

  std::vector<Drive> drives{
    liveDrive("pp_main", 8.0),
    liveDrive("gap_loc", 2.0)};
  Context context = contextFor(drives);
  scorer_->prepare(context, drives);  // captures an obstacle-bearing snapshot

  // The latest scan cannot be transformed after that snapshot. invalidateScan
  // clears all decisions and advances the generation before score tries to
  // commit the locally computed obstacle result.
  scan_pub_->publish(scan("missing_lidar", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);

  const ScoreResult raced = scorer_->score(drives[0], context);
  EXPECT_DOUBLE_EQ(raced.value, 1.0) << raced.note;
  EXPECT_NE(raced.note.find("stale scan decision discarded"), std::string::npos) <<
    raced.note;

  const float no_return = std::numeric_limits<float>::infinity();
  scan_pub_->publish(scan("map", {no_return, no_return, no_return}));
  spinFor(100ms);
  context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult recovered = scorer_->score(drives[0], context);
  EXPECT_DOUBLE_EQ(recovered.value, 1.0) << recovered.note;
  EXPECT_EQ(recovered.note.find("committed"), std::string::npos) << recovered.note;
}

TEST_F(ObstacleScanHealthTest, PrepareTimeoutClearsOldOccupancyBeforeRecovery)
{
  path_pub_->publish(straightPath());
  spinFor(100ms);

  scan_pub_->publish(scan("map", {0.5F, 0.5F, 0.5F}));
  spinFor(100ms);

  std::vector<Drive> drives{
    liveDrive("pp_main", 8.0),
    liveDrive("gap_loc", 2.0)};
  Context context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult obstacle = scorer_->score(drives[0], context);
  ASSERT_DOUBLE_EQ(obstacle.value, 0.0) << obstacle.note;

  spinFor(550ms);  // timeout_ms=500, with no replacement scan.
  context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult timed_out = scorer_->score(drives[0], context);
  EXPECT_DOUBLE_EQ(timed_out.value, 1.0) << timed_out.note;
  EXPECT_NE(timed_out.note.find("nothing can be seen"), std::string::npos) <<
    timed_out.note;

  // The first healthy empty scan must start a new ring. If timeout only reset
  // the decision state, the old dense obstacle scan would vote again here.
  const float no_return = std::numeric_limits<float>::infinity();
  scan_pub_->publish(scan("map", {no_return, no_return, no_return}));
  spinFor(100ms);
  context = contextFor(drives);
  scorer_->prepare(context, drives);
  const ScoreResult recovered = scorer_->score(drives[0], context);
  EXPECT_DOUBLE_EQ(recovered.value, 1.0) << recovered.note;
  EXPECT_EQ(recovered.note.find("committed"), std::string::npos) << recovered.note;
}

}  // namespace
}  // namespace co_driver
