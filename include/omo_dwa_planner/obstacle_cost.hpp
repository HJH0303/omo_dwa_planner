// include/omo_dwa_planner/obstacle_cost.hpp
#pragma once

#include <vector>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "omo_dwa_planner/types.hpp"
#include "omo_dwa_planner/local_costmap2d.hpp"
#include "omo_dwa_planner/config.hpp"

namespace omo_dwa {

/**
 * @brief Maintains a local 2D costmap in the robot frame (base_link-centered)
 * and computes obstacle costs for candidate trajectories.
 *
 * Update the internal grid from PointCloud2 and/or LaserScan. Each obstacle
 * cell is marked as LETHAL and an inflation layer is applied around it with
 * exponential decay. During evaluation, a trajectory incurs +inf if it hits
 * lethal/inscribed cells or goes outside the grid; otherwise its cost is the
 * average of normalized cell costs along the path.
 */
class ObstacleCost {
public:
  explicit ObstacleCost(const Config& cfg);

  /// Update costmap from a PointCloud2 already expressed in base_link frame.
  void updateFromPointCloud2(const sensor_msgs::msg::PointCloud2& cloud);

  /// Update costmap from a LaserScan expressed in the same local frame (z=0).
  void updateFromLaserScan(const sensor_msgs::msg::LaserScan& scan);

  /**
   * @brief Evaluate obstacle cost for each trajectory.
   * @param trjs Set of candidate trajectories in local frame coordinates.
   * @return Vector of costs (size == trjs.size()).
   *         +inf indicates collision or out-of-map.
   */
  std::vector<double> evaluate(const TrajSet& trjs) const;

  /// Export current grid as an OccupancyGrid for visualization/debugging.
  nav_msgs::msg::OccupancyGrid getCostmapMsg() const;

  /// (Optional) Clear the map if you want per-frame (non-accumulating) behavior.
  void clear();

private:
  struct InflationOffset {
    int dx{0};
    int dy{0};
    uint8_t cost{0};
  };

  /// Mark a cell as lethal and apply precomputed inflation around it.
  void markObstacleCell(int ix, int iy);

private:
  Config cfg_;
  CostMap2D map_;
  std::vector<InflationOffset> inflation_offsets_;
};

} // namespace omo_dwa
