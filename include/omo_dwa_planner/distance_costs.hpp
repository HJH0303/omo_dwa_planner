#pragma once

#include <vector>
#include <limits>
#include "omo_dwa_planner/types.hpp"
#include "omo_dwa_planner/config.hpp"

namespace omo_dwa {

/**
 * @brief Distance-based cost terms used by the DWA evaluator.
 *
 * Raw (non-normalized) costs:
 *  - path_cost(trjs, A,B,C):      mean perpendicular distance of all points to line Ax+By+C=0
 *  - alignment_cost(trjs, A,B,C, xshift, yshift):
 *        mean perpendicular distance using a shifted body point
 *        p = (x, y) + xshift * body_forward + yshift * body_left
 *  - goal_cost(trjs, goal):       Euclidean distance of final pose to the local goal
 *  - goal_center_cost(...):       corridor-penalized cross-track error of final pose to origin->goal line
 *
 * IMPORTANT: Trajectories and line parameters must be expressed in the SAME frame.
 */
class DistanceCosts {
public:
  explicit DistanceCosts(const Config& cfg);

  /// Mean |A x + B y + C| along each trajectory. (A,B,C normalized internally)
  std::vector<double> path_cost(const TrajSet& trjs,
                                double A, double B, double C) const;

  /// Mean |A px + B py + C| along each trajectory where
  /// px = x + xshift*cos(theta) + yshift*cos(theta + pi/2),
  /// py = y + xshift*sin(theta) + yshift*sin(theta + pi/2).
  /// Defaults mirror the Python (-0.3, 0.0).
  std::vector<double> alignment_cost(const TrajSet& trjs,
                                     double A, double B, double C,
                                     double xshift, double yshift) const;


  /// Euclidean distance ||p_end - goal|| for each trajectory (local frame).
  std::vector<double> goal_cost(const TrajSet& trjs, const Pose2D& goal) const;

  /// Cross-track error of final pose to origin->goal line, minus corridor radius, clamped at 0.
  std::vector<double> goal_center_cost(const TrajSet& trjs,
                                       const Pose2D& goal,
                                       double center_radius) const;

private:
  Config cfg_;
};

} // namespace omo_dwa
