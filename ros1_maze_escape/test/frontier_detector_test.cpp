#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>

#include "ros1_maze_escape/frontier_detector.hpp"

namespace ros1_maze_escape {
namespace {

constexpr std::int8_t kUnknown = -1;
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;

nav_msgs::OccupancyGrid makeMap(unsigned int width, unsigned int height,
                                double resolution = 1.0) {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(static_cast<std::size_t>(width) * height, kOccupied);
  return map;
}

geometry_msgs::PoseStamped makeRobotPose() {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  return pose;
}

std::size_t indexOf(const nav_msgs::OccupancyGrid& map,
                    unsigned int x, unsigned int y) {
  return static_cast<std::size_t>(y) * map.info.width + x;
}

TEST(FrontierDetectorTest, DetectsFrontierInSyntheticOccupancyGrid) {
  nav_msgs::OccupancyGrid map = makeMap(5U, 5U);
  for (unsigned int x = 1U; x <= 3U; ++x) {
    map.data[indexOf(map, x, 1U)] = kFree;
    map.data[indexOf(map, x, 2U)] = kUnknown;
  }
  FrontierDetectorConfig config;
  config.min_frontier_size_m = 1.0;
  const FrontierDetector detector(config);

  const std::vector<FrontierCandidate> frontiers =
      detector.detect(map, makeRobotPose());

  ASSERT_EQ(frontiers.size(), 1U);
  EXPECT_EQ(frontiers.front().frontier_cell_count, 3U);
}

TEST(FrontierDetectorTest, FiltersFrontierBelowMinimumSize) {
  nav_msgs::OccupancyGrid map = makeMap(3U, 3U);
  map.data[indexOf(map, 1U, 1U)] = kFree;
  map.data[indexOf(map, 1U, 2U)] = kUnknown;
  FrontierDetectorConfig config;
  config.min_frontier_size_m = 2.0;
  const FrontierDetector detector(config);

  const std::vector<FrontierCandidate> frontiers =
      detector.detect(map, makeRobotPose());

  EXPECT_TRUE(frontiers.empty());
}

TEST(FrontierDetectorTest, PlacesGoalOnlyOnFreeCell) {
  nav_msgs::OccupancyGrid map = makeMap(5U, 5U);
  for (unsigned int x = 1U; x <= 3U; ++x) {
    map.data[indexOf(map, x, 1U)] = kFree;
    map.data[indexOf(map, x, 2U)] = kUnknown;
  }
  FrontierDetectorConfig config;
  config.min_frontier_size_m = 1.0;
  const FrontierDetector detector(config);

  const std::vector<FrontierCandidate> frontiers =
      detector.detect(map, makeRobotPose());

  ASSERT_FALSE(frontiers.empty());
  for (const FrontierCandidate& frontier : frontiers) {
    const int grid_x = static_cast<int>(
        std::floor(frontier.goal.pose.position.x / map.info.resolution));
    const int grid_y = static_cast<int>(
        std::floor(frontier.goal.pose.position.y / map.info.resolution));
    ASSERT_GE(grid_x, 0);
    ASSERT_GE(grid_y, 0);
    ASSERT_LT(grid_x, static_cast<int>(map.info.width));
    ASSERT_LT(grid_y, static_cast<int>(map.info.height));
    EXPECT_EQ(map.data[indexOf(
                  map, static_cast<unsigned int>(grid_x),
                  static_cast<unsigned int>(grid_y))],
              kFree);
  }
}

TEST(FrontierDetectorTest, RejectsMapWithMismatchedDataSize) {
  nav_msgs::OccupancyGrid map = makeMap(4U, 4U);
  map.data.pop_back();
  const FrontierDetector detector(FrontierDetectorConfig{});

  const std::vector<FrontierCandidate> frontiers =
      detector.detect(map, makeRobotPose());

  EXPECT_TRUE(frontiers.empty());
}

TEST(FrontierDetectorTest, TransformsGoalUsingRotatedMapOrigin) {
  nav_msgs::OccupancyGrid map = makeMap(3U, 3U);
  constexpr double kHalfPi = 1.57079632679489661923;
  map.info.origin.position.x = 10.0;
  map.info.origin.position.y = 20.0;
  map.info.origin.orientation.z = std::sin(kHalfPi / 2.0);
  map.info.origin.orientation.w = std::cos(kHalfPi / 2.0);
  map.data[indexOf(map, 1U, 1U)] = kFree;
  map.data[indexOf(map, 1U, 2U)] = kUnknown;
  FrontierDetectorConfig config;
  config.min_frontier_size_m = 0.1;
  const FrontierDetector detector(config);

  const std::vector<FrontierCandidate> frontiers =
      detector.detect(map, makeRobotPose());

  ASSERT_EQ(frontiers.size(), 1U);
  EXPECT_NEAR(frontiers.front().goal.pose.position.x, 8.5, 1e-9);
  EXPECT_NEAR(frontiers.front().goal.pose.position.y, 21.5, 1e-9);
}

}  // namespace
}  // namespace ros1_maze_escape

