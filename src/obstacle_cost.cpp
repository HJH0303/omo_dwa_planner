// src/obstacle_cost.cpp
#include "omo_dwa_planner/obstacle_cost.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace omo_dwa {

namespace {
constexpr uint8_t LETHAL_OBSTACLE = 254;   // Occupied cell (definite collision)
constexpr uint8_t INSCRIBED_COST  = 180;   // Footprint boundary (near-collision)
constexpr uint8_t FREE_SPACE      = 0;     // Free cell
constexpr double COLLISION_SCORE = -1000.0;

inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}
} // namespace

void ObstacleCost::clear() {
  map_.reset();
}

ObstacleCost::ObstacleCost(const Config& cfg)
: cfg_(cfg),
  map_(cfg.map_width, cfg.map_height, cfg.cell_resolution)
{
  // Precompute inflation kernel offsets (dx, dy, cost) within inflation_radius.
  const double res = map_.resolution();
  const int r_cells = static_cast<int>(std::ceil(cfg_.inflation_radius / res));
  inflation_offsets_.clear();
  inflation_offsets_.reserve((2 * r_cells + 1) * (2 * r_cells + 1));

  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      const double dist = std::hypot(dx * res, dy * res);
      if (dist <= cfg_.inflation_radius) {
        // Exponential decay similar to the navigation stack.
        uint8_t c = FREE_SPACE;
        if (dist <= std::max(1e-6, cfg_.robot_radius)) {
          c = INSCRIBED_COST;
        } else {
          // cost ~ exp(-k * (d - r_inscribed))
          const double k = std::max(1e-6, cfg_.cost_scaling_factor);
          const double di = dist - cfg_.robot_radius;
          const double factor = std::exp(-k * std::max(0.0, di));
          // Map to [1 .. INSCRIBED_COST-1]
          int val = static_cast<int>(1 + (INSCRIBED_COST - 1) * factor);
          if (val > INSCRIBED_COST - 1) val = INSCRIBED_COST - 1;
          if (val < 1) val = 1;
          c = static_cast<uint8_t>(val);
        }
        inflation_offsets_.push_back({dx, dy, c});
      }
    }
  }
}

void ObstacleCost::updateFromPointCloud2(const sensor_msgs::msg::PointCloud2& cloud)
{
  const double res = map_.resolution();
  const int sx = map_.sizeX();
  const int sy = map_.sizeY();
  const double Xmax = sx * res;
  const double Ymax = sy * res;
  const double half_y = 0.5 * Ymax;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double x = *iter_x;  // forward
    const double y = *iter_y;  // left
    const double z = *iter_z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    if (z < cfg_.min_z_threshold || z > cfg_.max_z_threshold) continue;
    if (x < 0.0 || x >= Xmax) continue;
    if (y < -half_y || y >= half_y) continue;

    const int ix = static_cast<int>(std::floor(x / res));
    const int iy = static_cast<int>(std::floor((y + half_y) / res));
    markObstacleCell(ix, iy);
  }
}
void ObstacleCost::updateFromLaserScan(const sensor_msgs::msg::LaserScan& scan)
{
  const double res = map_.resolution();
  const int sx = map_.sizeX();                 // cells along +x
  const int sy = map_.sizeY();                 // cells along +y
  const double Xmax = sx * res;
  const double Ymax = sy * res;
  const double half_y = 0.5 * Ymax;

  const double off_x = cfg_.scan_offset_x;
  const double off_y = cfg_.scan_offset_y;

  const int N = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
  double angle = scan.angle_min;
  for (int i = 0; i < N; ++i, angle += scan.angle_increment) {
    const float r = (i < static_cast<int>(scan.ranges.size())) ? scan.ranges[i]
                                                              : std::numeric_limits<float>::quiet_NaN();
    if (!std::isfinite(r)) continue;
    if (r < scan.range_min || r > scan.range_max) continue;

    // base_link frame + mounting offset
    const double x = r * std::cos(angle) + off_x; // forward
    const double y = r * std::sin(angle) + off_y; // left

    if (x < 0.0 || x >= Xmax) continue;
    if (y < -half_y || y >= half_y) continue;

    const int ix = static_cast<int>(std::floor(x / res));
    const int iy = static_cast<int>(std::floor((y + half_y) / res));
    markObstacleCell(ix, iy);
  }

}

void ObstacleCost::markObstacleCell(int ix, int iy)
{
  // Set the center cell to lethal and inflate around it using the precomputed kernel.
  if (ix < 0 || iy < 0 || ix >= map_.sizeX() || iy >= map_.sizeY()) return;
  map_.setCell(ix, iy, std::max<uint8_t>(map_.getCell(ix, iy), LETHAL_OBSTACLE));

  for (const auto& off : inflation_offsets_) {
    const int nx = ix + off.dx;
    const int ny = iy + off.dy;
    if (nx < 0 || ny < 0 || nx >= map_.sizeX() || ny >= map_.sizeY()) continue;

    const uint8_t cur = map_.getCell(nx, ny);
    const uint8_t val = off.cost;
    if (val > cur) {
      map_.setCell(nx, ny, val);
    }
  }
}

std::vector<double> ObstacleCost::evaluate(const TrajSet& trjs) const
{
  // For each trajectory, return the mean normalized cell cost.
  // If it hits lethal/inscribed cells or goes out of the map, return +inf.
  std::vector<double> out;
  out.reserve(trjs.size());

  const int sx = map_.sizeX();
  const int sy = map_.sizeY();
  const double res = map_.resolution();
  const double Xmax = sx * res;
  const double Ymax = sy * res;
  const double half_y = 0.5 * Ymax;

  for (const auto& trj : trjs) {
    if (trj.empty()) {
    out.push_back(COLLISION_SCORE); // -1000
    continue;
    }

    double acc = 0.0;
    int valid = 0;
    bool collided = false;

    for (const auto& s : trj) {
      const double x = s.x; // forward
      const double y = s.y; // left

      if (x < 0.0 || x >= Xmax || y < -half_y || y >= half_y) { collided = true; break; }

      const int ix = static_cast<int>(std::floor(x / res));
      const int iy = static_cast<int>(std::floor((y + half_y) / res));
      const uint8_t c = map_.getCell(ix, iy);
      if (c >= INSCRIBED_COST) { collided = true; break; }

      acc += static_cast<double>(c);
      ++valid;
      }

    if (collided || valid == 0) {
        out.push_back(COLLISION_SCORE); // -1000
    } else {
        out.push_back(acc / static_cast<double>(valid)); // 원시값 평균(정규화 없음)
    }
  }

  return out;
}

nav_msgs::msg::OccupancyGrid ObstacleCost::getCostmapMsg() const
{
  // Convert the internal CostMap2D to an OccupancyGrid message (for RViz).
  return map_.toMsg();
}

}
 // namespace omo_dwa
