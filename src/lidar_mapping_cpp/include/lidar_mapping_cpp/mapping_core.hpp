// mapping_core.hpp
// ─────────────────────────────────────────────────────────────────────────────
// LiDAR 2D 지도작성의 순수 로직 (ROS 의존 없음 → 단독 테스트 가능).
//
// 방식: tinySLAM/CoreSLAM 계열의 경량 SLAM
//   1) 점유격자(occupancy grid)를 로그오즈(log-odds)로 유지
//   2) 새 스캔이 오면 현재 지도에 대해 언덕오르기(hill climbing) 스캔매칭으로
//      로봇 자세(x, y, θ)를 보정
//   3) 보정된 자세에서 스캔을 지도에 통합 (Bresenham 광선: 경로=비움, 끝점=점유)
//
// 한계(정직하게): 루프 클로저가 없어 큰 공간을 오래 돌면 누적 오차가 생긴다.
// 정밀 지도가 필요하면 공식 slam_toolbox(이미 C++)를 쓰고, 이 노드는
// 교육/경량 용도 + 오도메트리 없는 PuppyPi에서 즉시 동작하는 것이 목적.
// ─────────────────────────────────────────────────────────────────────────────
#ifndef LIDAR_MAPPING_CPP__MAPPING_CORE_HPP_
#define LIDAR_MAPPING_CPP__MAPPING_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace lidar_mapping_cpp
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// 스캔 한 발의 로봇(base) 기준 좌표
using ScanPoints = std::vector<std::pair<float, float>>;

class OccGrid
{
public:
  // 로그오즈 갱신 단위 (정수 고정소수점). HIT 3번이면 확실한 벽 취급.
  static constexpr int16_t kHit = 24;
  static constexpr int16_t kMiss = -6;
  static constexpr int16_t kMin = -120;
  static constexpr int16_t kMax = 120;

  OccGrid(int width, int height, double resolution)
  : width_(width), height_(height), resolution_(resolution),
    origin_x_(-width * resolution / 2.0),   // 지도 중앙이 월드 (0,0)
    origin_y_(-height * resolution / 2.0),
    logodds_(static_cast<size_t>(width) * height, 0),
    known_(static_cast<size_t>(width) * height, 0)
  {
  }

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}
  double origin_x() const {return origin_x_;}
  double origin_y() const {return origin_y_;}

  bool world_to_cell(double wx, double wy, int & cx, int & cy) const
  {
    cx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    cy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return cx >= 0 && cx < width_ && cy >= 0 && cy < height_;
  }

  int16_t logodds_at(int cx, int cy) const
  {
    return logodds_[static_cast<size_t>(cy) * width_ + cx];
  }

  // ROS OccupancyGrid 규약: -1 = 미탐사, 0 = 빈 공간, 100 = 점유
  int8_t occupancy_at(int cx, int cy) const
  {
    const size_t i = static_cast<size_t>(cy) * width_ + cx;
    if (!known_[i]) {
      return -1;
    }
    const double p = 1.0 / (1.0 + std::exp(-logodds_[i] * 0.05));
    return static_cast<int8_t>(std::lround(p * 100.0));
  }

  // 광선 하나 반영: (x0,y0)→(x1,y1) 경로는 비움, hit 이면 끝점 셀은 점유
  void update_ray(double x0, double y0, double x1, double y1, bool hit)
  {
    int cx0, cy0, cx1, cy1;
    if (!world_to_cell(x0, y0, cx0, cy0) || !world_to_cell(x1, y1, cx1, cy1)) {
      return;  // 지도 밖 광선은 통째로 무시 (부분 반영으로 인한 왜곡 방지)
    }
    // Bresenham
    const int dx = std::abs(cx1 - cx0);
    const int dy = std::abs(cy1 - cy0);
    const int sx = cx0 < cx1 ? 1 : -1;
    const int sy = cy0 < cy1 ? 1 : -1;
    int err = dx - dy;
    int x = cx0, y = cy0;
    while (true) {
      if (x == cx1 && y == cy1) {
        add(x, y, hit ? kHit : kMiss);
        break;
      }
      add(x, y, kMiss);
      const int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }
  }

private:
  void add(int cx, int cy, int16_t delta)
  {
    const size_t i = static_cast<size_t>(cy) * width_ + cx;
    known_[i] = 1;
    logodds_[i] = std::clamp<int16_t>(logodds_[i] + delta, kMin, kMax);
  }

  int width_, height_;
  double resolution_;
  double origin_x_, origin_y_;
  std::vector<int16_t> logodds_;
  std::vector<uint8_t> known_;
};

// base 기준 스캔 점들을 자세(pose)로 월드 좌표로 변환했을 때,
// 끝점들이 지도의 점유 셀에 얼마나 잘 얹히는지 점수화
inline int64_t score_scan(const OccGrid & grid, const Pose2D & pose, const ScanPoints & pts)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  int64_t score = 0;
  for (const auto & p : pts) {
    const double wx = pose.x + c * p.first - s * p.second;
    const double wy = pose.y + s * p.first + c * p.second;
    int cx, cy;
    if (grid.world_to_cell(wx, wy, cx, cy)) {
      score += grid.logodds_at(cx, cy);
    }
  }
  return score;
}

struct MatchParams
{
  double lin_step{0.05};    // 탐색 시작 보폭 (m)
  double ang_step{0.03};    // 탐색 시작 보폭 (rad)
  int halvings{4};          // 보폭을 절반으로 줄이는 횟수
  int max_iters{200};       // 안전 상한
};

// 언덕오르기 스캔매칭: init 주변에서 (±x, ±y, ±θ) 이웃을 시도하며
// 점수가 좋아지는 방향으로 이동, 개선이 없으면 보폭을 절반으로
inline Pose2D match_scan(
  const OccGrid & grid, const Pose2D & init, const ScanPoints & pts,
  const MatchParams & mp = MatchParams())
{
  Pose2D best = init;
  int64_t best_score = score_scan(grid, best, pts);
  double lin = mp.lin_step;
  double ang = mp.ang_step;
  int iters = 0;

  for (int level = 0; level <= mp.halvings; ) {
    bool improved = false;
    const Pose2D candidates[6] = {
      {best.x + lin, best.y, best.theta}, {best.x - lin, best.y, best.theta},
      {best.x, best.y + lin, best.theta}, {best.x, best.y - lin, best.theta},
      {best.x, best.y, best.theta + ang}, {best.x, best.y, best.theta - ang},
    };
    for (const auto & cand : candidates) {
      const int64_t sc = score_scan(grid, cand, pts);
      if (sc > best_score) {
        best_score = sc;
        best = cand;
        improved = true;
      }
    }
    if (!improved) {
      lin /= 2.0;
      ang /= 2.0;
      ++level;
    }
    if (++iters >= mp.max_iters) {
      break;
    }
  }
  return best;
}

// 스캔을 지도에 통합. pts_hit 는 유효 반사점, pose 는 base 의 월드 자세
inline void integrate_scan(OccGrid & grid, const Pose2D & pose, const ScanPoints & pts)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  for (const auto & p : pts) {
    const double wx = pose.x + c * p.first - s * p.second;
    const double wy = pose.y + s * p.first + c * p.second;
    grid.update_ray(pose.x, pose.y, wx, wy, true);
  }
}

}  // namespace lidar_mapping_cpp

#endif  // LIDAR_MAPPING_CPP__MAPPING_CORE_HPP_
