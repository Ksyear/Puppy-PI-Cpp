// mapping_core.hpp 시뮬레이션 검증:
// 8×8m 정사각형 방을 도는 로봇의 합성 LiDAR 스캔으로 지도를 만들고,
// 스캔매칭이 실제 이동을 추정해내는지 + 지도가 올바른지 확인한다.
#include <cassert>
#include <cstdio>
#include <cmath>

#include "lidar_mapping_cpp/mapping_core.hpp"

using namespace lidar_mapping_cpp;

// 방 [-4,4]^2 안의 점 (px,py)에서 방향 (dx,dy)로 쏜 광선이 벽에 닿는 거리
static double ray_to_wall(double px, double py, double dx, double dy)
{
  double t = 1e9;
  if (dx > 1e-9) {t = std::min(t, (4.0 - px) / dx);}
  if (dx < -1e-9) {t = std::min(t, (-4.0 - px) / dx);}
  if (dy > 1e-9) {t = std::min(t, (4.0 - py) / dy);}
  if (dy < -1e-9) {t = std::min(t, (-4.0 - py) / dy);}
  return t;
}

// 자세(pose)에서 360빔 스캔 생성 (base 기준 좌표로 반환)
static ScanPoints make_scan(const Pose2D & pose)
{
  ScanPoints pts;
  for (int i = 0; i < 360; ++i) {
    const double beam = i * M_PI / 180.0;                 // base 기준 빔 각도
    const double world_ang = pose.theta + beam;
    const double r = ray_to_wall(pose.x, pose.y, std::cos(world_ang), std::sin(world_ang));
    if (r < 12.0) {
      pts.emplace_back(
        static_cast<float>(r * std::cos(beam)),
        static_cast<float>(r * std::sin(beam)));
    }
  }
  return pts;
}

int main()
{
  OccGrid grid(400, 400, 0.05);  // 20×20m, 5cm 해상도

  // 진짜 궤적: 방 안을 사각형으로 도는 40스텝 (스텝당 5cm 전진 + 소회전)
  Pose2D truth{0.0, 0.0, 0.0};
  Pose2D est{0.0, 0.0, 0.0};

  integrate_scan(grid, est, make_scan(truth));  // 첫 스캔은 그대로 반영

  double worst_err = 0.0;
  for (int step = 1; step <= 60; ++step) {
    // 진짜 로봇 이동 (매칭기는 이 이동을 모름 — 이전 추정 자세에서 시작)
    truth.x += 0.05 * std::cos(truth.theta);
    truth.y += 0.05 * std::sin(truth.theta);
    truth.theta += 0.03;

    const ScanPoints scan = make_scan(truth);
    est = match_scan(grid, est, scan);
    integrate_scan(grid, est, scan);

    const double err = std::hypot(est.x - truth.x, est.y - truth.y);
    worst_err = std::max(worst_err, err);
  }
  const double final_pos_err = std::hypot(est.x - truth.x, est.y - truth.y);
  double final_ang_err = std::fabs(est.theta - truth.theta);
  while (final_ang_err > M_PI) {final_ang_err = std::fabs(final_ang_err - 2 * M_PI);}

  printf("60스텝 후 위치 오차: %.3f m (최대 %.3f m), 각도 오차: %.2f°\n",
    final_pos_err, worst_err, final_ang_err * 180 / M_PI);
  assert(final_pos_err < 0.15);              // 5cm 격자에서 15cm 이내면 정상 추적
  assert(final_ang_err < 5.0 * M_PI / 180);

  // 지도 검사: 벽 위치는 점유, 방 중앙은 빈 공간, 방 밖은 미탐사
  // (벽 셀은 추정 오차만큼 흔들릴 수 있어 벽 주변 밴드에서 최댓값 확인)
  int cx, cy;
  int wall_occ = -1;
  for (double wx = 3.9; wx <= 4.1; wx += 0.05) {
    for (double wy = -0.5; wy <= 0.5; wy += 0.05) {
      if (grid.world_to_cell(wx, wy, cx, cy)) {
        wall_occ = std::max(wall_occ, static_cast<int>(grid.occupancy_at(cx, cy)));
      }
    }
  }
  grid.world_to_cell(1.0, 1.0, cx, cy);       // 방 안
  const int free_occ = grid.occupancy_at(cx, cy);
  grid.world_to_cell(8.0, 8.0, cx, cy);       // 방 밖
  const int unknown_occ = grid.occupancy_at(cx, cy);
  printf("벽 셀=%d (>=65 기대), 내부 셀=%d (<=25 기대), 외부 셀=%d (-1 기대)\n",
    wall_occ, free_occ, unknown_occ);
  assert(wall_occ >= 65);
  assert(free_occ >= 0 && free_occ <= 25);
  assert(unknown_occ == -1);

  printf("지도작성 코어 검증 통과\n");
  return 0;
}
