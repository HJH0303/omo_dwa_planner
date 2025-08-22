// include/omo_dwa_planner/local_costmap2d.hpp
#pragma once

#include <vector>
#include <cstdint>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace omo_dwa {

/**
 * @brief Lightweight 2D grid for obstacle costs.
 *
 * Storage is row-major: index = iy * size_x_ + ix
 * - ix: column index along lateral (y), in [0 .. size_x_-1]
 * - iy: row index along forward (x), in [0 .. size_y_-1]
 *
 * Values are raw uint8_t costs in [0..254], where higher means more dangerous.
 * Conversion to nav_msgs/OccupancyGrid is provided via toMsg().
 *
 * NOTE on visualization origin (bottom-centered):
 * - This grid is used with a bottom-centered convention (robot at bottom center).
 * - toMsg() sets origin.position = (-width_m/2, 0), so RViz matches the indexing.
 */
class CostMap2D {
public:
  /// @param width_m   map width in meters (lateral, y)
  /// @param height_m  map height in meters (forward, x)
  /// @param resolution cell size [m/cell]
  CostMap2D(double width_m, double height_m, double resolution);

  /// Set all cells to 0 (free).
  void reset();

  /// Set/Get a cell value (bounds-checked; out-of-range calls are ignored/return 0).
  void setCell(int ix, int iy, uint8_t cost);
  uint8_t getCell(int ix, int iy) const;

  /// Grid geometry
  int sizeX() const { return size_x_; }
  int sizeY() const { return size_y_; }
  double resolution() const { return res_; }
  double widthMeters() const { return size_x_ * res_; }
  double heightMeters() const { return size_y_ * res_; }

  /// Convert to OccupancyGrid for visualization/debugging (0..100, no unknowns).
  /// The caller should set the header (stamp/frame_id) as appropriate.
  nav_msgs::msg::OccupancyGrid toMsg() const;

private:
  inline bool inBounds(int ix, int iy) const {
    return (ix >= 0 && iy >= 0 && ix < size_x_ && iy < size_y_);
  }
  inline std::size_t idx(int ix, int iy) const {
    return static_cast<std::size_t>(iy) * static_cast<std::size_t>(size_x_) +
           static_cast<std::size_t>(ix);
  }

private:
  int size_x_{0};
  int size_y_{0};
  double res_{0.05};
  std::vector<uint8_t> data_; // row-major storage
};

} // namespace omo_dwa
