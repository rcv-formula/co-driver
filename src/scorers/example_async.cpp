// 비동기 채점기 예시 — 무거운 계산을 평가 주기와 분리해서 돌립니다.
//
// score() 는 평가 주기(기본 50Hz)마다 호출되므로, 여기서 오래 걸리는 일을 하면
// 선택이 늦어집니다. 무거운 채점기는 **자기 타이머·콜백에서 계산해 두고**
// score() 에서는 결과만 꺼내 주세요.
//
// 프로토콜은 세 가지뿐입니다.
//   1) 계산은 자기 콜백/타이머에서 (다른 스레드이므로 뮤텍스)
//   2) 아직 새 결과가 없으면 score() 에서 `ScoreResult::pending()` 반환
//   3) 그러면 프레임워크가 **직전 점수를 대신 씁니다**.
//      단 inputs[].hold_ms 를 넘기면 자동으로 unavailable 이 되어 점수에서 빠집니다.
//
// 즉 채점기는 캐시·만료를 직접 구현할 필요가 없습니다. pending 만 내면 됩니다.
//
// 이 예시는 work_ms 만큼 걸리는 계산을 rate_hz 로 돌면서, 마지막 결과를 돌려줍니다.
// 실제로는 여기에 scan 순회·맵 조회 같은 무거운 로직이 들어갑니다.
//
//   {"name": "heavy", "type": "example_async",
//    "hold_ms": 500,                                 // 직전 점수를 얼마나 더 쓸지
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

    // 자기 타이머로 계산합니다. group() 에 넣어야 평가·출력 타이머와 **동시에**
    // 돕니다(기본 그룹은 MutuallyExclusive 라 서로를 막습니다).
    timer_ = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      [this]() {compute();}, group());

    RCLCPP_INFO(
      node->get_logger(), "async scorer '%s' (%.1fHz, work %.0fms)",
      name.c_str(), rate, work_ * 1e3);
    return true;
  }

  // 평가 주기마다 호출 — 여기서는 절대 오래 걸리면 안 됩니다.
  ScoreResult score(const Drive &, const Context &) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_result_) {
      // 아직 첫 결과가 없다 -> 프레임워크가 직전 점수를 찾아보고, 없으면 제외합니다.
      return ScoreResult::pending("waiting for first result");
    }
    if (consumed_) {
      // 새 결과가 아직 안 나왔다 -> 직전 점수를 hold_ms 동안 계속 쓰게 둡니다.
      return ScoreResult::pending("computing");
    }
    consumed_ = true;
    return ScoreResult::ok(value_);
  }

private:
  void compute()
  {
    // 무거운 계산 자리. 예시라 지연만 흉내 냅니다.
    if (work_ > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(work_)));
    }
    const double v = 0.5;   // 실제로는 여기서 [0,1] 점수를 산출합니다

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
