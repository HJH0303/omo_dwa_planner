// src/distance_costs.cpp
#include "omo_dwa_planner/distance_costs.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace omo_dwa {

namespace {
inline double normalize_angle(double a) {
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline bool normalize_line(double& A, double& B, double& C) {
  const double n = std::hypot(A, B);
  if (n <= 1e-9) return false;
  A /= n; B /= n; C /= n;
  return true;
}

} // namespace

DistanceCosts::DistanceCosts(const Config& cfg)
: cfg_(cfg)
{
  // Nothing to initialize beyond config reference.
}

// -----------------------------------------------------------------------------
// Path cost using a line Ax + By + C = 0.
// We normalize (A,B,C) internally so distance = |A*x + B*y + C|.
// Return the mean distance of all points in each trajectory.
// -----------------------------------------------------------------------------
std::vector<double> DistanceCosts::path_cost(const TrajSet& trjs,
                                             double A, double B, double C) const
{
  std::vector<double> out;
  out.reserve(trjs.size());

  const bool ok = normalize_line(A, B, C);
  for (const auto& trj : trjs) {
    if (trj.empty()) {
      out.push_back(std::numeric_limits<double>::infinity());
      continue;
    }
    if (!ok) {
      // Degenerate line -> no lateral error
      out.push_back(0.0);
      continue;
    }

    double acc = 0.0;
    for (const auto& s : trj) {
      acc += std::abs(A * s.x + B * s.y + C);
    }
    out.push_back(acc / static_cast<double>(trj.size()));
  }
  return out;
}

// -----------------------------------------------------------------------------
// Alignment cost to the line tangent.
// For line Ax+By+C=0 with unit normal (A,B), the tangent direction is (-B, A).
// Let psi_line = atan2(A, -B). The error is |wrapToPi(psi_end - psi_line)|.
// -----------------------------------------------------------------------------
std::vector<double> DistanceCosts::alignment_cost(const TrajSet& trjs,
                                                  double A, double B, double C,
                                                  double xshift, double yshift) const
{
  std::vector<double> out;
  out.reserve(trjs.size());

  const bool ok = normalize_line(A, B, C);
  for (const auto& trj : trjs) {
    if (trj.empty()) {
      out.push_back(std::numeric_limits<double>::infinity());
      continue;
    }
    if (!ok) {
      out.push_back(0.0);
      continue;
    }

    double acc = 0.0;
    for (const auto& s : trj) {
      const double ct = std::cos(s.yaw);
      const double st = std::sin(s.yaw);
      const double px = s.x + xshift * ct + yshift * std::cos(s.yaw + M_PI / 2.0);
      const double py = s.y + xshift * st + yshift * std::sin(s.yaw + M_PI / 2.0);
      acc += std::abs(A * px + B * py + C);
    }
    out.push_back(acc / static_cast<double>(trj.size()));
  }
  return out;
}

// -----------------------------------------------------------------------------
// Goal cost: Euclidean distance of final pose to the local goal.
// -----------------------------------------------------------------------------
std::vector<double> DistanceCosts::goal_cost(const TrajSet& trjs, const Pose2D& goal) const
{
  std::vector<double> out;
  out.reserve(trjs.size());

  for (const auto& trj : trjs) {
    if (trj.empty()) { out.push_back(0.0); continue; }
    const Pose2D& s_end = trj.back();
    const double d = std::hypot(goal.x - s_end.x, goal.y - s_end.y);
    out.push_back(d);
  }
  return out;
}

// -----------------------------------------------------------------------------
// Goal-center cost: cross-track error w.r.t. the line from origin to goal,
// with a free corridor radius. Only the excess outside the corridor is penalized.
// -----------------------------------------------------------------------------
std::vector<double> DistanceCosts::goal_center_cost(const TrajSet& trjs,
                                                    const Pose2D& goal,
                                                    double center_radius) const
{
  std::vector<double> out;
  out.reserve(trjs.size());

  const double L = std::hypot(goal.x, goal.y);
  if (L < 1e-9) {
    out.assign(trjs.size(), 0.0);
    return out;
  }
  // Unit normal n to the origin->goal line
  const double nx =  (goal.y / L);
  const double ny = -(goal.x / L);
  const double r  = std::max(0.0, center_radius);

  for (const auto& trj : trjs) {
    if (trj.empty()) { out.push_back(0.0); continue; }
    const Pose2D& s_end = trj.back();
    const double cte = std::abs(nx * s_end.x + ny * s_end.y);
    out.push_back(std::max(0.0, cte - r));
  }
  return out;
}

} // namespace omo_dwa
