#pragma once

#include <string>

namespace ros1_maze_escape {

struct FrontierFeatures {
  double information_gain{0.0};
  double actual_path_length{0.0};
  double clearance{0.0};
  double heading_change{0.0};
  double visit_count{0.0};
  double failure_count{0.0};
  double exit_probability{0.0};
};

struct FrontierScore {
  double value{0.0};
  bool valid{false};
  std::string source;
};

struct HeuristicWeights {
  double information_gain{2.0};
  double actual_path_length{-1.5};
  double clearance{1.5};
  double heading_change{-0.4};
  double visit_count{-3.0};
  double failure_count{-5.0};
  double exit_probability{3.0};
};

class FrontierScorer {
 public:
  virtual ~FrontierScorer() = default;
  virtual FrontierScore score(const FrontierFeatures& features) const noexcept = 0;
};

class HeuristicFrontierScorer final : public FrontierScorer {
 public:
  explicit HeuristicFrontierScorer(const HeuristicWeights& weights);
  FrontierScore score(const FrontierFeatures& features) const noexcept override;

 private:
  HeuristicWeights weights_;
};

bool areFinite(const FrontierFeatures& features) noexcept;

}  // namespace ros1_maze_escape

