#pragma once

#include <cstddef>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include "ros1_maze_escape/frontier_scorer.hpp"

namespace ros1_maze_escape {

struct FrontierCandidate {
  geometry_msgs::PoseStamped goal;
  FrontierFeatures features;
  std::size_t frontier_cell_count{0U};
};

struct FrontierDetectorConfig {
  double min_frontier_size_m{0.30};
  double clearance_search_radius_m{0.60};
};

class FrontierDetector {
 public:
  explicit FrontierDetector(const FrontierDetectorConfig& config);

  std::vector<FrontierCandidate> detect(
      const nav_msgs::OccupancyGrid& map,
      const geometry_msgs::PoseStamped& robot_pose) const;

 private:
  FrontierDetectorConfig config_;
};

}  // namespace ros1_maze_escape

