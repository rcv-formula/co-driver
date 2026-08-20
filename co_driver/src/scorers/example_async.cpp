// Async scorer example -- runs heavy computation decoupled from the evaluation cycle.
//
// score() is called every evaluation cycle (50Hz by default), so anything slow
// there delays selection. Heavy scorers should **compute in their own
// timer/callback** and have score() only hand back the result.
//
// The protocol is just three rules.
//   1) Compute in your own callback/timer (different thread, so mutex)
//   2) If there is no new result yet, return `ScoreResult::pending()` from score()
//   3) The framework then **substitutes the previous score**.
//      Once past inputs[].hold_ms it automatically becomes unavailable and
//      drops out of scoring.
//
// So a scorer never needs to implement caching/expiry itself; just return pending.
//
// This example runs a computation that takes work_ms at rate_hz and returns the
// last result. In practice this is where heavy logic like scan traversal or map
// lookups goes.
//
//   {"name": "heavy", "type": "example_async",
//    "hold_ms": 500,                                 // how long to keep using the previous score
//    "params": {"rate_hz": 5.0, "work_ms": 30.0},
//    "influence": {"weight": 1.0}}
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

#include "co_driver/scorer.hpp"

namespace co_driver
{

class ExampleAsyncScorer : public Scorer
{
public:
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    work_ = jms(p, "work_ms", 0.0);
    const double rate = std::max(0.1, jnum(p, "rate_hz", 5.0));

    // Compute on our own timer. It must go in group() to run **concurrently** with
    // the evaluation/output timers (the default group is MutuallyExclusive, so they
    // would block each other).
    timer_ = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      [this]() {compute();}, group());

    RCLCPP_INFO(
      node->get_logger(), "async scorer '%s' (%.1fHz, work %.0fms)",
      name.c_str(), rate, work_ * 1e3);
    return true;
  }

  // Called every evaluation cycle -- must never take long.
  ScoreResult score(const Drive &, const Context &) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_result_) {
      // No first result yet -> the framework looks for a previous score and excludes us if none.
      return ScoreResult::pending("waiting for first result");
    }
    if (consumed_) {
      // No new result yet -> let the previous score keep being used for hold_ms.
      return ScoreResult::pending("computing");
    }
    consumed_ = true;
    return ScoreResult::ok(value_);
  }

private:
  void compute()
  {
    // Heavy computation goes here. Being an example, we only fake the delay.
    if (work_ > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(work_)));
    }
    const double v = 0.5;   // in practice, compute the [0,1] score here

    std::lock_guard<std::mutex> lock(mtx_);
    value_ = v;
    has_result_ = true;
    consumed_ = false;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  double work_{0.0};

  std::mutex mtx_;
  double value_{0.0};
  bool has_result_{false};
  bool consumed_{true};
};

CO_DRIVER_REGISTER_SCORER(ExampleAsyncScorer, "example_async")

}  // namespace co_driver
