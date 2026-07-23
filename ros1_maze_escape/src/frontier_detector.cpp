#include "ros1_maze_escape/frontier_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ros1_maze_escape {
namespace {

struct GridCell {
  int x;
  int y;
};

bool inBounds(int x, int y, int width, int height) {
  return x >= 0 && y >= 0 && x < width && y < height;
}

std::size_t gridIndex(int x, int y, int width) {
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
         static_cast<std::size_t>(x);
}

geometry_msgs::Point gridToWorld(const nav_msgs::MapMetaData& info,
                                 double grid_x, double grid_y) {
  const double yaw = tf2::getYaw(info.origin.orientation);
  const double local_x = (grid_x + 0.5) * info.resolution;
  const double local_y = (grid_y + 0.5) * info.resolution;
  geometry_msgs::Point point;
  point.x = info.origin.position.x + std::cos(yaw) * local_x -
            std::sin(yaw) * local_y;
  point.y = info.origin.position.y + std::sin(yaw) * local_x +
            std::cos(yaw) * local_y;
  point.z = info.origin.position.z;
  return point;
}

double normalizedAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

FrontierDetector::FrontierDetector(const FrontierDetectorConfig& config)
    : config_(config) {}

std::vector<FrontierCandidate> FrontierDetector::detect(
    const nav_msgs::OccupancyGrid& map,
    const geometry_msgs::PoseStamped& robot_pose) const {
  std::vector<FrontierCandidate> result;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const std::size_t expected_size =
      static_cast<std::size_t>(map.info.width) *
      static_cast<std::size_t>(map.info.height);
  if (width <= 0 || height <= 0 || map.info.resolution <= 0.0 ||
      map.data.size() != expected_size) {
    return result;
  }

  std::vector<std::uint8_t> is_frontier(expected_size, 0U);
  constexpr int kCardinalX[4] = {1, -1, 0, 0};
  constexpr int kCardinalY[4] = {0, 0, 1, -1};
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t index = gridIndex(x, y, width);
      if (map.data[index] != 0) {
        continue;
      }
      for (int neighbor = 0; neighbor < 4; ++neighbor) {
        const int nx = x + kCardinalX[neighbor];
        const int ny = y + kCardinalY[neighbor];
        if (inBounds(nx, ny, width, height) &&
            map.data[gridIndex(nx, ny, width)] < 0) {
          is_frontier[index] = 1U;
          break;
        }
      }
    }
  }

  std::vector<std::uint8_t> visited(expected_size, 0U);
  const std::size_t minimum_cell_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
              std::ceil(config_.min_frontier_size_m / map.info.resolution)));
  constexpr int kNeighborX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  constexpr int kNeighborY[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  for (int seed_y = 0; seed_y < height; ++seed_y) {
    for (int seed_x = 0; seed_x < width; ++seed_x) {
      const std::size_t seed_index = gridIndex(seed_x, seed_y, width);
      if (is_frontier[seed_index] == 0U || visited[seed_index] != 0U) {
        continue;
      }

      std::queue<GridCell> pending;
      std::vector<GridCell> cluster;
      pending.push({seed_x, seed_y});
      visited[seed_index] = 1U;
      while (!pending.empty()) {
        const GridCell cell = pending.front();
        pending.pop();
        cluster.push_back(cell);
        for (int neighbor = 0; neighbor < 8; ++neighbor) {
          const int nx = cell.x + kNeighborX[neighbor];
          const int ny = cell.y + kNeighborY[neighbor];
          if (!inBounds(nx, ny, width, height)) {
            continue;
          }
          const std::size_t neighbor_index = gridIndex(nx, ny, width);
          if (is_frontier[neighbor_index] != 0U &&
              visited[neighbor_index] == 0U) {
            visited[neighbor_index] = 1U;
            pending.push({nx, ny});
          }
        }
      }

      if (cluster.size() < minimum_cell_count) {
        continue;
      }

      double centroid_x = 0.0;
      double centroid_y = 0.0;
      std::unordered_set<std::size_t> adjacent_unknown;
      for (const GridCell& cell : cluster) {
        centroid_x += cell.x;
        centroid_y += cell.y;
        for (int neighbor = 0; neighbor < 8; ++neighbor) {
          const int nx = cell.x + kNeighborX[neighbor];
          const int ny = cell.y + kNeighborY[neighbor];
          if (inBounds(nx, ny, width, height)) {
            const std::size_t neighbor_index = gridIndex(nx, ny, width);
            if (map.data[neighbor_index] < 0) {
              adjacent_unknown.insert(neighbor_index);
            }
          }
        }
      }
      centroid_x /= static_cast<double>(cluster.size());
      centroid_y /= static_cast<double>(cluster.size());

      const GridCell* goal_cell = &cluster.front();
      double best_distance_squared = std::numeric_limits<double>::infinity();
      for (const GridCell& cell : cluster) {
        const double dx = cell.x - centroid_x;
        const double dy = cell.y - centroid_y;
        const double distance_squared = dx * dx + dy * dy;
        if (distance_squared < best_distance_squared) {
          best_distance_squared = distance_squared;
          goal_cell = &cell;
        }
      }

      FrontierCandidate candidate;
      candidate.goal.header.frame_id = map.header.frame_id;
      candidate.goal.header.stamp = ros::Time(0);
      candidate.goal.pose.position =
          gridToWorld(map.info, goal_cell->x, goal_cell->y);
      const geometry_msgs::Point centroid_world =
          gridToWorld(map.info, centroid_x, centroid_y);
      const double frontier_yaw =
          std::atan2(centroid_world.y - candidate.goal.pose.position.y,
                     centroid_world.x - candidate.goal.pose.position.x);
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, frontier_yaw);
      candidate.goal.pose.orientation = tf2::toMsg(orientation);
      candidate.frontier_cell_count = cluster.size();
      candidate.features.information_gain =
          static_cast<double>(adjacent_unknown.size()) * map.info.resolution *
          map.info.resolution;

      const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
      const double goal_bearing =
          std::atan2(candidate.goal.pose.position.y - robot_pose.pose.position.y,
                     candidate.goal.pose.position.x - robot_pose.pose.position.x);
      candidate.features.heading_change =
          std::abs(normalizedAngle(goal_bearing - robot_yaw));

      const int search_cells = std::max(
          1, static_cast<int>(
                 std::ceil(config_.clearance_search_radius_m /
                           map.info.resolution)));
      double nearest_obstacle = config_.clearance_search_radius_m;
      for (int offset_y = -search_cells; offset_y <= search_cells; ++offset_y) {
        for (int offset_x = -search_cells; offset_x <= search_cells; ++offset_x) {
          const int x = goal_cell->x + offset_x;
          const int y = goal_cell->y + offset_y;
          if (!inBounds(x, y, width, height) ||
              map.data[gridIndex(x, y, width)] < 50) {
            continue;
          }
          nearest_obstacle = std::min(
              nearest_obstacle,
              std::hypot(offset_x * map.info.resolution,
                         offset_y * map.info.resolution));
        }
      }
      candidate.features.clearance = nearest_obstacle;
      // TODO(EXIT_MODEL): derive this from a separately validated exit detector.
      candidate.features.exit_probability = 0.0;
      result.push_back(std::move(candidate));
    }
  }

  return result;
}

}  // namespace ros1_maze_escape
