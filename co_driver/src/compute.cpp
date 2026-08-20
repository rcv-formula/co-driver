#include "co_driver/compute.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace co_driver
{

// ---------------------------------------------------------------------------
// Response curve phi
// ---------------------------------------------------------------------------
double shapeInfluence(const Influence & inf, double x)
{
  double u = inf.invert ? (1.0 - x) : x;
  if (std::abs(inf.in_max - inf.in_min) > 1e-12) {
    u = (u - inf.in_min) / (inf.in_max - inf.in_min);
  }
  u = std::clamp(u, 0.0, 1.0);

  // (e^{k*u} - 1) / (e^{k} - 1) : [0,1] -> [0,1], monotonic. The sign of k sets the curvature:
  //   k > 0 convex ("reward only when very good"), k < 0 concave ("drops fast when slightly bad"),
  //   k ~ 0 linear
  const double e = (std::abs(inf.exp_k) > 1e-9) ?
    std::expm1(inf.exp_k * u) / std::expm1(inf.exp_k) : u;

  // Blend the linear and exponential components with a single mix ratio.
  // Both are in [0,1], so the result is too.
  const double mix = std::clamp(inf.exp_mix, 0.0, 1.0);
  const double s = (1.0 - mix) * u + mix * e;
  return std::isfinite(s) ? std::clamp(s, 0.0, 1.0) : 0.0;
}

// ---------------------------------------------------------------------------
// Resolve a score -- fill an async scorer's pending result from the cache.
// ---------------------------------------------------------------------------
ScoreResult resolveScore(
  Drive & d, const std::string & input, const ScoreResult & fresh, double hold,
  const rclcpp::Time & now, double * held_age)
{
  if (held_age) {*held_age = -1.0;}

  // Fresh score arrived -> update the cache and use it as-is.
  if (fresh.available && !fresh.veto) {
    CachedScore & c = d.score_cache[input];
    c.value = std::clamp(fresh.value, 0.0, 1.0);
    c.stamp = now;
    c.valid = true;
    return fresh;
  }

  // Still computing -> substitute the previous score (if present and not too old).
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

  // unavailable / vetoed are the scorer's explicit decisions; do not override them with the cache.
  return fresh;
}

// ---------------------------------------------------------------------------
// Stage 1 -- one drive's linear-layer output z_j = sum(W * phi(x)) + b_j
// ---------------------------------------------------------------------------
void computeLogit(Drive & d, const ScoringSpec & spec, const rclcpp::Time & now)
{
  d.reject.clear();
  d.active = true;
  d.valid = false;
  d.score = 0.0;

  // Record the deactivation reason but keep computing the score (as far as possible).
  auto deactivate = [&d](const std::string & why) {
      if (d.active) {d.reject = why;}   // keep only the first reason
      d.active = false;
    };
  // Only when no score is possible at all (no command, or NaN) do we zero z and stop.
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

  // --- Hard gates: failing one drops the drive from ranking/selection, but scoring continues ---
  if (!d.enabled) {
    deactivate("disabled");
  }
  if (!d.isFresh(now)) {
    // Past the hold window = this topic is not alive right now.
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

  double linear_sum = 0.0;   // sum(W * phi)
  double abs_used = 0.0;     // sum(|W|) (terms usable this cycle)
  double abs_all = 0.0;      // sum(|W|) (all terms)
  double pos_weight = 0.0;   // weighted_sum normalization denominator (phi is in [0,1], so sum(w) suffices)
  int used = 0;

  for (const auto & kv : d.influence) {
    const Influence & inf = kv.second;
    abs_all += std::abs(inf.weight);
    const auto it = d.results.find(kv.first);

    if (it == d.results.end() || !it->second.available) {
      if (inf.required) {
        deactivate("required[" + kv.first + "] unavailable");
      }
      // missing: "zero" effectively adds phi=0; "mask" rescales below.
      continue;
    }

    const double x = std::clamp(it->second.value, 0.0, 1.0);
    const double shaped = shapeInfluence(inf, x);

    if (inf.veto_below >= 0.0 && shaped < inf.veto_below) {
      deactivate("veto_below[" + kv.first + "]");
    }
    if (std::abs(inf.weight) <= 1e-12) {continue;}   // not scored (veto check only)

    linear_sum += inf.weight * shaped;   // negative weights stay as penalty terms
    abs_used += std::abs(inf.weight);
    if (inf.weight > 0.0) {
      pos_weight += inf.weight;
      ++used;
    }
  }

  if (softmax_mode) {
    double z = linear_sum;
    if (spec.missing == "mask" && abs_used > 1e-9 && abs_all > 1e-9) {
      // With missing inputs, restore the full scale from the remaining terms, so a
      // drive is not pushed down just because one scorer received no data.
      z *= abs_all / abs_used;
    }
    d.raw_logit = z + d.bias;
  } else if (used == 0 || pos_weight <= 1e-9) {
    d.raw_logit = 0.5 + d.bias;   // no evidence -> neutral
  } else {
    d.raw_logit = linear_sum / pos_weight + d.bias;
  }
  if (!std::isfinite(d.raw_logit)) {d.raw_logit = 0.0;}

  // Apply the EMA to the pre-softmax logit (so the probabilities always stay a normalized distribution).
  const double alpha = std::clamp(spec.ema_alpha, 0.0, 1.0);
  if (!d.logit_initialized || alpha >= 1.0) {
    d.logit = d.raw_logit;
    d.logit_initialized = true;
  } else {
    d.logit += alpha * (d.raw_logit - d.logit);
  }
}

// ---------------------------------------------------------------------------
// Stage 2 -- softmax
// ---------------------------------------------------------------------------
void finalizeScores(std::vector<Drive> & drives, const ScoringSpec & spec)
{
  // Only active drives take part -- inactive ones keep score 0 and are excluded from ranking.
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
  if (n == 0) {return;}   // everyone disqualified -- selection is handled by the node

  // Subtract the max to avoid exp overflow (softmax is invariant to this shift).
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
