// Shared reader for the localization confidence contract.
//
//   data = [stamp_sec, stamp_nanosec, confidence, term_score, term_outlier,
//           term_skip, term_spread, term_mass]
//   confidence = state_multiplier x min(the five terms)
//
// Older producers publish only the first three slots. Three scorers consume
// this topic - the score input, the early-degradation trip and the recovery
// latch - and all three must agree on which terms count, so the parsing and
// the "smallest judged term" rule live here rather than in each of them.
//
// The reason any term is excluded at all: term_skip counts beams masked out as
// unexplained, i.e. how much unmapped or dynamic obstacle is in view. The
// producer masks those beams BEFORE scoring, so term_score and term_outlier
// keep reading 1.000 through heavy occlusion - judging the aggregate would
// penalise the map-based drive for having another vehicle beside it.
#ifndef CO_DRIVER__CONFIDENCE_TERMS_HPP_
#define CO_DRIVER__CONFIDENCE_TERMS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>

namespace co_driver
{

// Fills terms/labels from an appended-term array. Clears both and returns
// false when the producer only sends the aggregate.
inline bool readConfidenceTerms(
  const std_msgs::msg::Float64MultiArray & msg,
  std::vector<double> * terms, std::vector<std::string> * labels)
{
  terms->clear();
  labels->clear();
  if (msg.data.size() < 8) {return false;}
  static const char * kTerm[5] =
  {"term_score", "term_outlier", "term_skip", "term_spread", "term_mass"};
  for (std::size_t k = 0; k < 5; ++k) {
    const std::size_t slot = 3 + k;
    terms->push_back(msg.data[slot]);
    // Prefer the producer's own label when the layout carries one.
    labels->push_back(
      (slot < msg.layout.dim.size() && !msg.layout.dim[slot].label.empty()) ?
      msg.layout.dim[slot].label : kTerm[k]);
  }
  return true;
}

inline bool isIgnoredTerm(
  const std::string & label, const std::vector<std::string> & ignore)
{
  for (const auto & s : ignore) {
    if (s == label) {return true;}
  }
  return false;
}

// Smallest term that is not on the ignore list, plus the smallest term overall
// (which the callers report as the reason the aggregate looks low). Returns
// false when there is nothing left to judge.
inline bool minJudgedTerm(
  const std::vector<double> & terms, const std::vector<std::string> & labels,
  const std::vector<std::string> & ignore,
  double * judged_value, std::string * judged_label,
  double * overall_value = nullptr, std::string * overall_label = nullptr)
{
  if (terms.size() != 5 || labels.size() != 5) {return false;}
  std::size_t arg_all = 0;
  std::size_t arg = 0;
  bool have = false;
  for (std::size_t k = 0; k < 5; ++k) {
    if (terms[k] < terms[arg_all]) {arg_all = k;}
    if (isIgnoredTerm(labels[k], ignore)) {continue;}
    if (!have || terms[k] < terms[arg]) {arg = k; have = true;}
  }
  if (overall_value) {*overall_value = terms[arg_all];}
  if (overall_label) {*overall_label = labels[arg_all];}
  if (!have) {return false;}
  *judged_value = terms[arg];
  *judged_label = labels[arg];
  return true;
}

}  // namespace co_driver

#endif  // CO_DRIVER__CONFIDENCE_TERMS_HPP_
