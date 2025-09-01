#pragma once

#include <cstdint>
#include <utility>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace omo_dwa {

struct OverlapResult {
  bool    has_overlap{false};     // LETHAL 셀이 경로 코리도와 겹치면 true
  int     lethal_hits{0};         // 코리도 내 LETHAL 셀 개수
  int     near_hits{0};           // 코리도 내 INSCRIBED~LETHAL 셀 개수
  double  min_distance{1e9};      // 어떤 점유셀과 경로 선분 사이의 최소거리 [m]
};

// (odom) 로봇 포즈와 타깃을 (base_link)으로 변환: 타깃의 base_link 좌표 반환
inline std::pair<double,double>
odomTargetToBaseLink(double rob_x_odom, double rob_y_odom, double rob_yaw,
                     double tgt_x_odom, double tgt_y_odom)
{
  const double dx = tgt_x_odom - rob_x_odom;
  const double dy = tgt_y_odom - rob_y_odom;
  const double c = std::cos(rob_yaw), s = std::sin(rob_yaw);
  const double bx =  c * dx + s * dy;  // base_link x (전방+)
  const double by = -s * dx + c * dy;  // base_link y (좌측+)
  return {bx, by};
}

// 로컬 OccupancyGrid( base_link 기준 ) 전 셀을 훑어
// 직선 글로벌 경로(0,0 → target_bl_x,target_bl_y) "코리도(반경=corridor_radius_m)"와의 겹침을 검사
// lethal_thresh, near_thresh 는 0..254 스케일 가정(0..100도 자동 환산)
OverlapResult check_global_path_obstacle_overlap(
    const nav_msgs::msg::OccupancyGrid& grid,
    double target_bl_x, double target_bl_y,
    double corridor_radius_m,
    uint8_t lethal_thresh = 254,
    uint8_t near_thresh   = 180,
    double  x_forward_limit_m = 4.0,   // (전체 스캔 버전에선 사용 안 함; 시그니처 호환용)
    double  y_half_width_m    = 2.0
);

} // namespace omo_dwa
