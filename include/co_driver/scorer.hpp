// Scorer interface -- "what information scores a drive, and how".
//
// One scorer corresponds to one element of config's inputs[]. Each scorer can
// **subscribe to the topics it needs itself** (the node is handed over via
// configure). So adding new judgment logic never touches the node or other scorers.
//
// Writing a new scorer -- add a single .cpp file under src/scorers/, done.
// No header needed (the registry instantiates it from the type string).
//
//   #include "co_driver/scorer.hpp"
//   namespace co_driver {
//   class MyScorer : public Scorer {
//    public:
//     bool configure(rclcpp::Node * node, const std::string &, const Json & p) override {
//       // Create your own subscriptions here if needed (callback group: group()).
//       rclcpp::SubscriptionOptions o; o.callback_group = group();
//       sub_ = node->create_subscription<...>(jstr(p, "topic", "/x"), 10, cb, o);
//       return true;
//     }
//     ScoreResult score(const Drive & d, const Context & ctx) override {
//       if (no sensor data) return ScoreResult::unavailable("no data");
//       if (dangerous)      return ScoreResult::vetoed("reason");
//       return ScoreResult::ok(0.0 ~ 1.0);
//     }
//   };
//   CO_DRIVER_REGISTER_SCORER(MyScorer, "my_scorer")
//   }
//
// Then add the file to the scorer_sources list in CMakeLists and put
// {"name": ..., "type": "my_scorer"} into inputs of the topic-list JSON.
//
// -- Async scorers ----------------------------------------------------------
// score() is called every evaluation cycle (default 50Hz), so it must not take
// long. Do heavy computation in your own timers/callbacks (the node runs a
// MultiThreadedExecutor, so they run concurrently) and return
// `ScoreResult::pending()` while there is no new result yet.
//
//   pending -> the framework **substitutes the previous score**.
//              Once that score exceeds inputs[].hold_ms it automatically becomes
//              unavailable and drops out of score combination (weight and all).
//
// Caching and expiry are handled by the framework; the scorer only emits pending.
// Example: src/scorers/example_async.cpp
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

  // node   : handed over so the scorer can create its own subscriptions/timers.
  // name   : config inputs[].name (the same type can be reused under different names)
  // params : config inputs[].params, verbatim
  // Returning false aborts node startup (configuration error).
  virtual bool configure(rclcpp::Node * node, const std::string & name, const Json & params) = 0;

  // Called once before scoring drives one by one. Do whole-set normalization here.
  virtual void prepare(const Context &, const std::vector<Drive> &) {}

  // Score for one drive. Must be in [0,1] (or unavailable / vetoed).
  virtual ScoreResult score(const Drive & drive, const Context & ctx) = 0;

  const std::string & name() const {return name_;}
  const std::string & type() const {return type_;}

  // Filled in by the node before configure().
  void setContext(const std::string & n, const std::string & t, rclcpp::CallbackGroup::SharedPtr g)
  {
    name_ = n;
    type_ = t;
    group_ = std::move(g);
  }

protected:
  // Put every subscription/timer the scorer creates **into this group**. It is a
  // Reentrant group, so they run concurrently with other scorers and with the
  // evaluation/output timers.
  // Forgetting this puts them in the default (MutuallyExclusive) group where they
  // block each other.
  //
  //   subscription: rclcpp::SubscriptionOptions o; o.callback_group = group();
  //                 node->create_subscription<T>(topic, qos, cb, o);
  //   timer: node->create_wall_timer(period, cb, group());
  rclcpp::CallbackGroup::SharedPtr group() const {return group_;}

  std::string name_;
  std::string type_;
  rclcpp::CallbackGroup::SharedPtr group_;
};

using ScorerPtr = std::shared_ptr<Scorer>;

// type string -> factory. Each scorer .cpp registers itself.
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

// One file = one scorer. This macro registers it during static initialization.
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

// --- Small helpers for reading JSON params (used by scorer implementations) ---
double jnum(const Json & j, const std::string & key, double fallback);
// Time values only -- configured in **ms**, returned in seconds. fallback is in ms too.
double jms(const Json & j, const std::string & key, double fallback_ms);
int jint(const Json & j, const std::string & key, int fallback);
bool jbool(const Json & j, const std::string & key, bool fallback);
std::string jstr(const Json & j, const std::string & key, const std::string & fallback);
std::vector<std::string> jstrs(const Json & j, const std::string & key);
std::vector<double> jnums(const Json & j, const std::string & key);

}  // namespace co_driver

#endif  // CO_DRIVER__SCORER_HPP_
