#include "ros1_maze_escape/frontier_scorer.hpp"

#include <cmath>

namespace ros1_maze_escape {

HeuristicFrontierScorer::HeuristicFrontierScorer(
    const HeuristicWeights& weights)
    : weights_(weights) {}

FrontierScore HeuristicFrontierScorer::score(
    const FrontierFeatures& features) const noexcept {
  FrontierScore result;
  result.source = "heuristic";
  if (!areFinite(features)) {
    return result;
  }

  result.value =
      weights_.information_gain * features.information_gain +
      weights_.actual_path_length * features.actual_path_length +
      weights_.clearance * features.clearance +
      weights_.heading_change * features.heading_change +
      weights_.visit_count * features.visit_count +
      weights_.failure_count * features.failure_count +
      weights_.exit_probability * features.exit_probability;
  result.valid = std::isfinite(result.value);
  return result;
}

bool areFinite(const FrontierFeatures& features) noexcept {
  return std::isfinite(features.information_gain) &&
         std::isfinite(features.actual_path_length) &&
         std::isfinite(features.clearance) &&
         std::isfinite(features.heading_change) &&
         std::isfinite(features.visit_count) &&
         std::isfinite(features.failure_count) &&
         std::isfinite(features.exit_probability);
}

}  // namespace ros1_maze_escape

