#include "omo_dwa_planner/global_path_overlap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace omo_dwa {

namespace {
inline double clamp(double v, double lo, double hi){
  return std::max(lo, std::min(v, hi));
}

// 점-선분 거리
inline double distancePointToSegment(double px, double py,
                                     double ax, double ay,
                                     double bx, double by)
{
  const double vx = bx - ax, vy = by - ay;
  const double wx = px - ax, wy = py - ay;
  const double vv = vx*vx + vy*vy;
  double t = 0.0;
  if (vv > 1e-12) t = clamp((wx*vx + wy*vy)/vv, 0.0, 1.0);
  const double dx = wx - t*vx;
  const double dy = wy - t*vy;
  return std::sqrt(dx*dx + dy*dy);
}
} // namespace

OverlapResult check_global_path_obstacle_overlap(
    const nav_msgs::msg::OccupancyGrid& grid,
    double target_bl_x, double target_bl_y,
    double corridor_radius_m,
    uint8_t lethal_thresh,
    uint8_t near_thresh,
    double  x_forward_limit_m,
    double  y_half_width_m)
{
  (void)x_forward_limit_m; // 전체 스캔 버전: 미사용
  (void)y_half_width_m;

  OverlapResult out{};

  // 타깃-로봇 선분 길이 체크
  const double L = std::hypot(target_bl_x, target_bl_y);
  if (!(L > 1e-6)) {
    out.has_overlap = false;
    out.min_distance = std::numeric_limits<double>::infinity();
    return out;
  }

  const auto& info = grid.info;
  const double res    = (info.resolution > 0.0) ? info.resolution : 0.05;
  const int    size_x = static_cast<int>(info.width);
  const int    size_y = static_cast<int>(info.height);
  const double org_x  = info.origin.position.x; // 프로젝트: bottom-center origin 가정
  const double org_y  = info.origin.position.y;

  // base_link 선분 A(0,0) → B(target_bl)
  const double ax = 0.0, ay = 0.0;
  const double bx = target_bl_x, by = target_bl_y;

  // 선분 AABB를 코리도 반경만큼 확장 (논리적으로 안전한 프루닝)
  const double seg_xmin = std::min(ax, bx) - corridor_radius_m;
  const double seg_xmax = std::max(ax, bx) + corridor_radius_m;
  const double seg_ymin = std::min(ay, by) - corridor_radius_m;
  const double seg_ymax = std::max(ay, by) + corridor_radius_m;

  const double corridor_r2 = corridor_radius_m * corridor_radius_m;
  const double to254 = 254.0 / 100.0; // 0..100 → 0..254 환산

  out.min_distance = std::numeric_limits<double>::infinity();

  // 전체 셀 순회
  for (int iy = 0; iy < size_y; ++iy) {
    for (int ix = 0; ix < size_x; ++ix) {
      const std::size_t idx = static_cast<std::size_t>(iy) * size_x + ix;
      const int8_t occ = (idx < grid.data.size()) ? grid.data[idx] : 0;
      if (occ < 0) continue; // unknown은 기본 무시 (원하면 위험 처리로 변경 가능)

      // 셀 중심(base_link)
      const double cx = org_x + (static_cast<double>(ix) + 0.5) * res;
      const double cy = org_y + (static_cast<double>(iy) + 0.5) * res;

      // 선분 확장 AABB 바깥이면 스킵
      if (cx < seg_xmin || cx > seg_xmax || cy < seg_ymin || cy > seg_ymax) continue;

      // 선분까지 거리
      const double dist = distancePointToSegment(cx, cy, ax, ay, bx, by);
      if (dist < out.min_distance) out.min_distance = dist;

      // 코리도 내부면 점유값 판정
      if (dist * dist <= corridor_r2) {
        double cost254 = static_cast<double>(occ);
        if (occ <= 100) cost254 = std::clamp(occ * to254, 0.0, 254.0);

        if (cost254 >= static_cast<double>(lethal_thresh)) {
          ++out.lethal_hits;
        } else if (cost254 >= static_cast<double>(near_thresh)) {
          ++out.near_hits;
        }
      }
    }
  }

  out.has_overlap = (out.lethal_hits > 0);
  if (!std::isfinite(out.min_distance)) {
    out.min_distance = std::numeric_limits<double>::infinity();
  }
  return out;
}

} // namespace omo_dwa
