// 채점기(Scorer) 인터페이스 — "어떤 정보로 드라이브에 점수를 줄 것인가".
//
// 채점기 하나가 config 의 inputs[] 원소 하나에 대응합니다. 각 채점기는
// **자기가 필요한 토픽을 스스로 구독**할 수 있습니다(configure 로 노드가 넘어옴).
// 그래서 새 판단 로직을 넣을 때 노드나 다른 채점기를 건드릴 일이 없습니다.
//
// 새 채점기 만들기 — src/scorers/ 에 .cpp 파일 하나만 추가하면 끝입니다.
// 헤더도 필요 없습니다(레지스트리가 type 문자열로 만들어 주므로).
//
//   #include "co_driver/scorer.hpp"
//   namespace co_driver {
//   class MyScorer : public Scorer {
//    public:
//     bool configure(rclcpp::Node * node, const std::string &, const Json & p) override {
//       // 필요하면 여기서 자기 구독자를 만듭니다(콜백 그룹은 group()).
//       rclcpp::SubscriptionOptions o; o.callback_group = group();
//       sub_ = node->create_subscription<...>(jstr(p, "topic", "/x"), 10, cb, o);
//       return true;
//     }
//     ScoreResult score(const Drive & d, const Context & ctx) override {
//       if (센서 없음) return ScoreResult::unavailable("no data");
//       if (위험)      return ScoreResult::vetoed("이유");
//       return ScoreResult::ok(0.0 ~ 1.0);
//     }
//   };
//   CO_DRIVER_REGISTER_SCORER(MyScorer, "my_scorer")
//   }
//
// 그 다음 CMakeLists 의 scorer_sources 목록에 파일을 넣고,
// 토픽 목록 JSON 의 inputs 에 {"name": ..., "type": "my_scorer"} 를 적으면 됩니다.
//
// ── 비동기 채점기 ────────────────────────────────────────────────────────
// score() 는 평가 주기(기본 50Hz)마다 호출되므로 오래 걸리면 안 됩니다. 무거운
// 계산은 자기 타이머·콜백에서 하고(노드가 MultiThreadedExecutor 라 동시에 돕니다),
// 아직 새 결과가 없으면 `ScoreResult::pending()` 을 반환하세요.
//
//   pending -> 프레임워크가 **직전 점수를 대신 씁니다**.
//              그 점수가 inputs[].hold_ms 를 넘기면 자동으로 unavailable 이 되어
//              점수 결합에서 빠집니다(가중치째 제외).
//
// 캐시와 만료는 프레임워크가 처리하므로 채점기는 pending 만 내면 됩니다.
// 예시: src/scorers/example_async.cpp
#ifndef CO_DRIVER__SCORER_HPP_
#define CO_DRIVER__SCORER_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "co_driver/compute.hpp"
#include "co_driver/config.hpp"

namespace co_driver
{

class Scorer
{
public:
  virtual ~Scorer() = default;

  // node   : 자기 구독자·타이머를 만들 수 있도록 넘겨줍니다.
  // name   : config inputs[].name (같은 type 을 이름만 바꿔 여러 번 쓸 수 있음)
  // params : config inputs[].params 를 그대로
  // false 를 돌려주면 노드가 기동을 중단합니다(설정 오류).
  virtual bool configure(rclcpp::Node * node, const std::string & name, const Json & params) = 0;

  // 드라이브를 하나씩 채점하기 전에 한 번 호출. 전체를 봐야 하는 정규화를 여기서.
  virtual void prepare(const Context &, const std::vector<Drive> &) {}

  // 드라이브 하나의 점수. 반드시 [0,1] (또는 unavailable / vetoed).
  virtual ScoreResult score(const Drive & drive, const Context & ctx) = 0;

  const std::string & name() const {return name_;}
  const std::string & type() const {return type_;}

  // 노드가 configure() 전에 채워 줍니다.
  void setContext(const std::string & n, const std::string & t, rclcpp::CallbackGroup::SharedPtr g)
  {
    name_ = n;
    type_ = t;
    group_ = std::move(g);
  }

protected:
  // 채점기가 만드는 구독·타이머는 **이 그룹에 넣으세요**. Reentrant 그룹이라
  // 다른 채점기, 그리고 평가·출력 타이머와 동시에 실행됩니다.
  // 이걸 빼먹으면 기본 그룹(MutuallyExclusive)에 들어가 서로를 막습니다.
  //
  //   구독: rclcpp::SubscriptionOptions o; o.callback_group = group();
  //         node->create_subscription<T>(topic, qos, cb, o);
  //   타이머: node->create_wall_timer(period, cb, group());
  rclcpp::CallbackGroup::SharedPtr group() const {return group_;}

  std::string name_;
  std::string type_;
  rclcpp::CallbackGroup::SharedPtr group_;
};

using ScorerPtr = std::shared_ptr<Scorer>;

// type 문자열 -> 생성자. 각 채점기 .cpp 가 자기 자신을 등록합니다.
class ScorerRegistry
{
public:
  static ScorerRegistry & instance()
  {
    static ScorerRegistry r;
    return r;
  }
  void add(const std::string & type, std::function<ScorerPtr()> factory)
  {
    factories_[type] = std::move(factory);
  }
  ScorerPtr create(const std::string & type) const
  {
    const auto it = factories_.find(type);
    return it == factories_.end() ? nullptr : it->second();
  }
  std::vector<std::string> types() const
  {
    std::vector<std::string> out;
    for (const auto & kv : factories_) {out.push_back(kv.first);}
    return out;
  }

private:
  std::map<std::string, std::function<ScorerPtr()>> factories_;
};

// 파일 하나 = 채점기 하나. 이 매크로가 정적 초기화 때 레지스트리에 등록합니다.
#define CO_DRIVER_REGISTER_SCORER(CLASS, TYPE) \
  namespace { \
  struct CLASS ## Registrar \
  { \
    CLASS ## Registrar() \
    { \
      ::co_driver::ScorerRegistry::instance().add( \
        TYPE, []() {return std::static_pointer_cast<::co_driver::Scorer>( \
            std::make_shared<CLASS>());}); \
    } \
  }; \
  const CLASS ## Registrar g_ ## CLASS ## _registrar; \
  }

// --- JSON params 를 읽는 작은 헬퍼 (채점기 구현에서 씁니다) ---
double jnum(const Json & j, const std::string & key, double fallback);
// 시간 값 전용 — 설정에는 **ms** 로 적고 초로 돌려받습니다. fallback 도 ms 단위.
double jms(const Json & j, const std::string & key, double fallback_ms);
int jint(const Json & j, const std::string & key, int fallback);
bool jbool(const Json & j, const std::string & key, bool fallback);
std::string jstr(const Json & j, const std::string & key, const std::string & fallback);
std::vector<std::string> jstrs(const Json & j, const std::string & key);
std::vector<double> jnums(const Json & j, const std::string & key);

}  // namespace co_driver

#endif  // CO_DRIVER__SCORER_HPP_
