// 채점기 예시 겸 스켈레톤 — 외부 노드가 쏘는 점수 토픽을 구독해 점수로 씁니다.
//
// 새 채점기를 만들 때 이 파일을 복사해서 시작하세요. 채점기는 헤더가 필요 없고
// .cpp 하나로 완결됩니다(레지스트리가 type 문자열로 만들어 주므로).
// 구조는 딱 세 부분입니다.
//
//   1) configure()  설정을 읽고, **필요하면 자기 구독자를 만든다**
//   2) 콜백          받은 데이터를 멤버에 저장 (다른 스레드이므로 뮤텍스)
//   3) score()      드라이브 하나에 [0,1] 점수를 준다
//
// ---------------------------------------------------------------------------
// 이 채점기(type: "external_score")가 지원하는 계약
//
//   메시지: std_msgs/Float64MultiArray
//
//   mode: "scalar"         배열의 index 한 칸이 "차량 전체" 점수. 모든 드라이브에
//                          같은 값이 적용됩니다. localization_pf 의
//                          /localization_confidence (`[stamp_sec, stamp_nanosec,
//                          confidence]`, index=2) 가 이 형태입니다.
//
//   mode: "per_candidate"  드라이브별 점수. 짝짓기는 아래 우선순위입니다.
//                            (a) layout.dim[i].label 이 드라이브 이름과 같은 칸
//                            (b) params 의 order: ["a","b",...] 순서대로 data[i]
//
//   stamp_indices 를 주면 그 칸들을 [sec, nanosec] 로 읽어 신선도를 판정합니다(timeout_ms)
//   (std_msgs 에는 header 가 없어 localization_pf 가 쓰는 관례).
//   값은 [input_min, input_max] -> [0,1] 로 정규화되므로 원 단위 그대로 내도 됩니다.
//   NaN 이 오면 그 드라이브에서만 "판단 불가"로 처리합니다.
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>

#include "co_driver/scorer.hpp"

namespace co_driver
{

class ExternalScoreScorer : public Scorer
{
public:
  // --- 1) 설정 읽기 + 자기 구독자 만들기 -------------------------------------
  bool configure(rclcpp::Node * node, const std::string & name, const Json & p) override
  {
    node_ = node;
    topic_ = jstr(p, "topic", "");
    if (topic_.empty()) {
      RCLCPP_ERROR(node->get_logger(), "'%s': params.topic 이 필요합니다.", name.c_str());
      return false;
    }
    mode_ = jstr(p, "mode", "scalar");
    if (mode_ != "scalar" && mode_ != "per_candidate") {
      RCLCPP_ERROR(
        node->get_logger(), "'%s': mode 는 scalar 또는 per_candidate 여야 합니다 (받은 값: %s)",
        name.c_str(), mode_.c_str());
      return false;
    }
    index_ = jint(p, "index", 0);
    stamp_indices_ = jnums(p, "stamp_indices");
    order_ = jstrs(p, "order");
    timeout_ = jms(p, "timeout_ms", 500.0);
    input_min_ = jnum(p, "input_min", 0.0);
    input_max_ = jnum(p, "input_max", 1.0);
    invert_ = jbool(p, "invert", false);
    on_missing_ = jstr(p, "on_missing", "unavailable");   // unavailable | value | veto
    default_score_ = jnum(p, "default_score", 0.5);
    if (std::abs(input_max_ - input_min_) < 1e-9) {
      RCLCPP_ERROR(node->get_logger(), "'%s': input_min 과 input_max 가 같습니다.", name.c_str());
      return false;
    }

    // 점수 토픽은 최신 값만 의미가 있으므로 큐를 짧게 잡습니다.
    // 콜백 그룹은 채점기 전용(Reentrant)에 넣어 평가·출력 타이머를 막지 않게 합니다.
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group();
    sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_, rclcpp::QoS(5),
      [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {onMsg(msg);}, opts);

    RCLCPP_INFO(
      node->get_logger(), "채점기 '%s' <- %s (mode=%s, index=%d, timeout=%.0fms)",
      name.c_str(), topic_.c_str(), mode_.c_str(), index_, timeout_ * 1e3);
    return true;
  }

  // --- 3) 드라이브 하나 채점 -------------------------------------------------
  ScoreResult score(const Drive & drive, const Context & ctx) override
  {
    double raw = 0.0;
    bool found = false;
    std::string why;

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_msg_) {
        why = "no message on " + topic_;
      } else if (timeout_ > 0.0 && (ctx.now - stamp_).seconds() > timeout_) {
        why = "stale " + std::to_string((ctx.now - stamp_).seconds()) + "s";
      } else if (mode_ == "scalar") {
        const auto i = static_cast<std::size_t>(std::max(0, index_));
        if (i < data_.size()) {
          raw = data_[i];
          found = true;
        } else {
          why = "index out of range";
        }
      } else {
        // per_candidate: (a) layout label 매칭 우선, (b) order 목록의 순서
        std::size_t slot = 0;
        bool have = false;
        for (std::size_t i = 0; i < labels_.size(); ++i) {
          if (labels_[i] == drive.name) {slot = i; have = true; break;}
        }
        if (!have) {
          for (std::size_t i = 0; i < order_.size(); ++i) {
            if (order_[i] == drive.name) {slot = i; have = true; break;}
          }
        }
        if (!have) {
          why = "drive not in labels/order";
        } else if (slot < data_.size()) {
          raw = data_[slot];
          found = true;
        } else {
          why = "slot out of range";
        }
      }
    }

    if (!found) {
      if (on_missing_ == "veto") {return ScoreResult::vetoed(why);}
      if (on_missing_ == "value") {
        return ScoreResult::ok(std::clamp(default_score_, 0.0, 1.0), why);
      }
      return ScoreResult::unavailable(why);
    }
    // 외부 노드가 죽어가며 NaN 을 뱉는 경우가 실제로 있습니다.
    if (!std::isfinite(raw)) {return ScoreResult::unavailable("non-finite value");}

    double v = std::clamp((raw - input_min_) / (input_max_ - input_min_), 0.0, 1.0);
    return ScoreResult::ok(invert_ ? 1.0 - v : v);
  }

private:
  // --- 2) 콜백: 받은 것을 저장만 합니다 (채점은 score() 에서) -----------------
  void onMsg(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    data_ = msg->data;
    labels_.clear();
    for (const auto & d : msg->layout.dim) {labels_.push_back(d.label);}

    // std_msgs 에는 header 가 없어, 배열 앞 칸에 실린 stamp 를 씁니다(있으면).
    bool stamped = false;
    if (stamp_indices_.size() == 2) {
      const auto si = static_cast<std::size_t>(stamp_indices_[0]);
      const auto ni = static_cast<std::size_t>(stamp_indices_[1]);
      if (si < data_.size() && ni < data_.size()) {
        stamp_ = rclcpp::Time(
          static_cast<int64_t>(data_[si]), static_cast<uint32_t>(data_[ni]),
          node_->get_clock()->get_clock_type());
        stamped = true;
      }
    }
    if (!stamped) {stamp_ = node_->now();}
    has_msg_ = true;
  }

  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

  std::string topic_;
  std::string mode_{"scalar"};
  int index_{0};
  std::vector<double> stamp_indices_;
  std::vector<std::string> order_;
  double timeout_{0.5};
  double input_min_{0.0};
  double input_max_{1.0};
  bool invert_{false};
  std::string on_missing_{"unavailable"};
  double default_score_{0.5};

  std::mutex mtx_;
  bool has_msg_{false};
  rclcpp::Time stamp_;
  std::vector<double> data_;
  std::vector<std::string> labels_;
};

// 이 한 줄이 type 문자열과 클래스를 이어 줍니다. 파일을 복사했다면 여기만 바꾸세요.
CO_DRIVER_REGISTER_SCORER(ExternalScoreScorer, "external_score")

}  // namespace co_driver
