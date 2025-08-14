// src/costmap2d.cpp
#include "omo_dwa_planner/local_costmap2d.hpp"

#include <algorithm>
#include <cmath>

namespace omo_dwa {

CostMap2D::CostMap2D(double width_m, double height_m, double resolution)
: res_(resolution)
{
  if (res_ <= 0.0) res_ = 0.05; // fallback
  size_x_ = std::max(1, static_cast<int>(std::round(width_m  / res_)));
  size_y_ = std::max(1, static_cast<int>(std::round(height_m / res_)));
  data_.assign(static_cast<std::size_t>(size_x_ * size_y_), 0u);
}

void CostMap2D::reset()
{
  std::fill(data_.begin(), data_.end(), static_cast<uint8_t>(0));
}

void CostMap2D::setCell(int ix, int iy, uint8_t cost)
{
  if (!inBounds(ix, iy)) return;
  data_[idx(ix, iy)] = cost;
}

uint8_t CostMap2D::getCell(int ix, int iy) const
{
  if (!inBounds(ix, iy)) return 0u;
  return data_[idx(ix, iy)];
}
nav_msgs::msg::OccupancyGrid CostMap2D::toMsg() const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = static_cast<float>(res_);
  msg.info.width  = static_cast<uint32_t>(size_x_);   // cells along +x (forward)
  msg.info.height = static_cast<uint32_t>(size_y_);   // cells along +y (left)
  const double Wm_y = heightMeters();  // span in +y (left)

  // Origin = lower-left corner of the grid in the publishing frame.
  // We want x in [0, Wm_x), y in [-Wm_y/2, +Wm_y/2), so:
  msg.info.origin.position.x = 0.0;               // start at robot front origin
  msg.info.origin.position.y = -0.5 * Wm_y;       // center laterally around robot
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;            // no rotation

  // Fill row-major to avoid transpose.
  msg.data.resize(static_cast<std::size_t>(size_x_ * size_y_));
  const double scale = 100.0 / 254.0;
  for (int iy = 0; iy < size_y_; ++iy) {
    for (int ix = 0; ix < size_x_; ++ix) {
      const int idx = iy * size_x_ + ix;
      const int v = static_cast<int>(std::lround(static_cast<double>(getCell(ix, iy)) * scale));
      msg.data[idx] = static_cast<int8_t>(std::clamp(v, 0, 100));
    }
  }
  return msg;
}

} // namespace omo_dwa
