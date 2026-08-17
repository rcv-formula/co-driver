#include "co_driver/compute.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace co_driver
{

// ---------------------------------------------------------------------------
// 응답 곡선 φ
// ---------------------------------------------------------------------------
double shapeInfluence(const Influence & inf, double x)
{
  double u = inf.invert ? (1.0 - x) : x;
  if (std::abs(inf.in_max - inf.in_min) > 1e-12) {
    u = (u - inf.in_min) / (inf.in_max - inf.in_min);
  }
  u = std::clamp(u, 0.0, 1.0);

  // (e^{k·u} - 1) / (e^{k} - 1) : [0,1] -> [0,1] 단조. k 의 부호가 곡률을 정합니다.
  //   k > 0 볼록("아주 좋아야 점수를 준다"), k < 0 오목("조금만 나빠도 급락"), k ~ 0 선형
  const double e = (std::abs(inf.exp_k) > 1e-9) ?
    std::expm1(inf.exp_k * u) / std::expm1(inf.exp_k) : u;

  // 선형 성분과 지수 성분을 배합비 하나로 섞습니다. 둘 다 [0,1] 이라 결과도 [0,1].
  const double mix = std::clamp(inf.exp_mix, 0.0, 1.0);
  const double s = (1.0 - mix) * u + mix * e;
  return std::isfinite(s) ? std::clamp(s, 0.0, 1.0) : 0.0;
}

// ---------------------------------------------------------------------------
// 채점 결과 확정 — 비동기 채점기의 pending 을 캐시로 메웁니다.
// ---------------------------------------------------------------------------
ScoreResult resolveScore(
  Drive & d, const std::string & input, const ScoreResult & fresh, double hold,
  const rclcpp::Time & now, double * held_age)
{
  if (held_age) {*held_age = -1.0;}

  // 새 점수가 나왔다 -> 캐시 갱신 후 그대로 사용.
  if (fresh.available && !fresh.veto) {
    CachedScore & c = d.score_cache[input];
    c.value = std::clamp(fresh.value, 0.0, 1.0);
    c.stamp = now;
    c.valid = true;
    return fresh;
  }

  // 아직 계산 중 -> 직전 점수를 대신 쓴다(있고, 아직 안 늙었으면).
  if (fresh.is_pending) {
    const auto it = d.score_cache.find(input);
    if (it == d.score_cache.end() || !it->second.valid) {
      return ScoreResult::unavailable("no score yet");
    }
    const double age = (now - it->second.stamp).seconds();
    if (hold > 0.0 && age > hold) {
      char buf[80];
      std::snprintf(buf, sizeof(buf), "score not updated for %.0fms (hold %.0fms)", age * 1e3, hold * 1e3);
      return ScoreResult::unavailable(buf);
    }
    if (held_age) {*held_age = age;}
    return ScoreResult::ok(it->second.value, "held");
  }

  // unavailable / vetoed 는 채점기의 명시적 판단이라 캐시로 덮지 않습니다.
  return fresh;
}

// ---------------------------------------------------------------------------
// 1단계 — 드라이브 하나의 선형층 출력 z_j = Σ W·φ(x) + b_j
// ---------------------------------------------------------------------------
void computeLogit(Drive & d, const ScoringSpec & spec, const rclcpp::Time & now)
{
  d.reject.clear();
  d.active = true;
  d.valid = false;
  d.score = 0.0;

  // 비활성 사유를 기록하되, 점수 계산은 계속합니다(가능한 한).
  auto deactivate = [&d](const std::string & why) {
      if (d.active) {d.reject = why;}   // 첫 사유만 남깁니다
      d.active = false;
    };
  // 아예 점수를 낼 수 없는 경우(명령이 없거나 NaN)만 z 를 0 으로 두고 끝냅니다.
  auto noScore = [&d](const std::string & why) {
      d.active = false;
      d.reject = why;
      d.raw_logit = 0.0;
      d.logit = 0.0;
      d.logit_initialized = false;
    };

  if (!d.has_cmd) {
    noScore("no message ever received");
    return;
  }
  if (!std::isfinite(d.cmd.drive.speed) || !std::isfinite(d.cmd.drive.steering_angle)) {
    noScore("command is NaN/inf");
    return;
  }

  // --- 하드 게이트: 통과 못 하면 순위·선택에서 빠지지만 점수는 계속 계산합니다 ---
  if (!d.enabled) {
    deactivate("disabled");
  }
  if (!d.isFresh(now)) {
    // hold 를 넘겼다 = 이 토픽은 지금 살아 있지 않다.
    char buf[96];
    std::snprintf(
      buf, sizeof(buf), "stale %.0fms (hold %.0fms)", d.age(now) * 1e3, d.hold * 1e3);
    deactivate(buf);
  }
  for (const auto & kv : d.results) {
    if (kv.second.veto) {
      deactivate("veto[" + kv.first + "]" + (kv.second.note.empty() ? "" : ": " + kv.second.note));
      break;
    }
  }

  const bool softmax_mode = (spec.combine == "softmax");

  double linear_sum = 0.0;   // Σ W·φ
  double abs_used = 0.0;     // Σ |W| (이번 주기에 쓸 수 있었던 것)
  double abs_all = 0.0;      // Σ |W| (전체)
  double pos_weight = 0.0;   // weighted_sum 정규화 분모 (φ 가 [0,1] 이라 Σw 로 충분)
  int used = 0;

  for (const auto & kv : d.influence) {
    const Influence & inf = kv.second;
    abs_all += std::abs(inf.weight);
    const auto it = d.results.find(kv.first);

    if (it == d.results.end() || !it->second.available) {
      if (inf.required) {
        deactivate("required[" + kv.first + "] unavailable");
      }
      // missing: "zero" 면 φ=0 을 더한 셈, "mask" 면 아래에서 되스케일합니다.
      continue;
    }

    const double x = std::clamp(it->second.value, 0.0, 1.0);
    const double shaped = shapeInfluence(inf, x);

    if (inf.veto_below >= 0.0 && shaped < inf.veto_below) {
      deactivate("veto_below[" + kv.first + "]");
    }
    if (std::abs(inf.weight) <= 1e-12) {continue;}   // 점수 미반영(veto 판정만)

    linear_sum += inf.weight * shaped;   // 음수 가중치는 감점 항으로 그대로
    abs_used += std::abs(inf.weight);
    if (inf.weight > 0.0) {
      pos_weight += inf.weight;
      ++used;
    }
  }

  if (softmax_mode) {
    double z = linear_sum;
    if (spec.missing == "mask" && abs_used > 1e-9 && abs_all > 1e-9) {
      // 빠진 입력이 있으면 남은 항으로 전체 스케일을 복원합니다. 채점기 하나가
      // 데이터를 못 받았다고 그 드라이브의 logit 만 작아져 밀려나는 일을 막습니다.
      z *= abs_all / abs_used;
    }
    d.raw_logit = z + d.bias;
  } else if (used == 0 || pos_weight <= 1e-9) {
    d.raw_logit = 0.5 + d.bias;   // 판단 근거 없음 -> 중립
  } else {
    d.raw_logit = linear_sum / pos_weight + d.bias;
  }
  if (!std::isfinite(d.raw_logit)) {d.raw_logit = 0.0;}

  // EMA 는 softmax 이전 logit 에 겁니다(확률이 항상 정규화된 분포로 유지되도록).
  const double alpha = std::clamp(spec.ema_alpha, 0.0, 1.0);
  if (!d.logit_initialized || alpha >= 1.0) {
    d.logit = d.raw_logit;
    d.logit_initialized = true;
  } else {
    d.logit += alpha * (d.raw_logit - d.logit);
  }
}

// ---------------------------------------------------------------------------
// 2단계 — softmax
// ---------------------------------------------------------------------------
void finalizeScores(std::vector<Drive> & drives, const ScoringSpec & spec)
{
  // 활성 드라이브만 모읍니다 — 비활성은 score 가 0 으로 남고 순위에 들어가지 않습니다.
  if (spec.combine != "softmax") {
    for (auto & d : drives) {
      if (!d.active) {continue;}
      d.score = d.logit;
      d.valid = d.score >= spec.min_valid_score;
      if (!d.valid) {d.reject = "score < min_valid_score";}
    }
    return;
  }

  const double T = std::max(spec.temperature, 1e-6);
  double max_logit = -std::numeric_limits<double>::infinity();
  int n = 0;
  for (const auto & d : drives) {
    if (!d.active) {continue;}
    max_logit = std::max(max_logit, d.logit);
    ++n;
  }
  if (n == 0) {return;}   // 전원 실격 — 선택은 노드가 처리합니다

  // max 를 빼서 exp 오버플로를 막습니다(softmax 는 이 이동에 불변).
  double denom = 0.0;
  for (const auto & d : drives) {
    if (d.active) {denom += std::exp((d.logit - max_logit) / T);}
  }
  const bool ok = (denom > 0.0) && std::isfinite(denom);

  for (auto & d : drives) {
    if (!d.active) {continue;}
    d.score = ok ? std::exp((d.logit - max_logit) / T) / denom : 1.0 / static_cast<double>(n);
    d.valid = d.score >= spec.min_valid_score;
    if (!d.valid) {d.reject = "p " + std::to_string(d.score) + " < min_valid_score";}
  }
}

}  // namespace co_driver
