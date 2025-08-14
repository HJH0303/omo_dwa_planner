#include "omo_dwa_planner/cost_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace omo_dwa {

static inline bool is_finite(double v) {
  return std::isfinite(v);
}

CostEvaluator::CostEvaluator(const Config& cfg)
: w_obst_(cfg.w_obstacle),
  w_path_(cfg.w_path),
  w_align_(cfg.w_alignment),
  w_goal_(cfg.w_goal),
  w_center_(cfg.w_goal_center)
{
  // Nothing else to do.
}

std::vector<double> CostEvaluator::minmax_normalize(const std::vector<double>& col)
{
  // Collect finite values to compute robust min/max
  double mn = std::numeric_limits<double>::infinity();
  double mx = -std::numeric_limits<double>::infinity();
  for (double v : col) {
    if (!is_finite(v)) continue;
    mn = std::min(mn, v);
    mx = std::max(mx, v);
  }
  if (!is_finite(mn) || !is_finite(mx)) {
    // No finite values -> return 1.0 (max)
    return std::vector<double>(col.size(), 1.0);
  }
  double range = mx - mn;
  if (range == 0.0) range = 1.0;

  std::vector<double> out;
  out.reserve(col.size());
  for (double v : col) {
    if (is_finite(v)) {
      out.push_back((v - mn) / range);
    } else {
      // Non-finite -> map to worst (1.0) to avoid NaNs propagating
      out.push_back(1.0);
    }
  }
  return out;
}

std::vector<double> CostEvaluator::obstacle_normalize(const std::vector<double>& col)
{
  // Positive finite subset min-max
  double mn = std::numeric_limits<double>::infinity();
  double mx = -std::numeric_limits<double>::infinity();
  for (double v : col) {
    if (v > 0.0 && is_finite(v)) {
      mn = std::min(mn, v);
      mx = std::max(mx, v);
    }
  }
  if (!is_finite(mn) || !is_finite(mx)) {
    // No positive finite values -> fall back to (min=0, range=1)
    mn = 0.0;
    mx = 1.0;
  }
  double range = mx - mn;
  if (range == 0.0) range = 1.0;

  std::vector<double> out;
  out.reserve(col.size());
  for (double v : col) {
    if (v > 0.0 && is_finite(v)) {
      out.push_back((v - mn) / range);
    } else {
      // Keep negatives (collisions) and non-finite as-is (negatives will be filtered later)
      out.push_back(v);
    }
  }
  return out;
}

void CostEvaluator::split_columns(const std::vector<std::vector<double>>& terms,
                                  std::vector<double>& c_obst,
                                  std::vector<double>& c_path,
                                  std::vector<double>& c_align,
                                  std::vector<double>& c_goal,
                                  std::vector<double>& c_center)
{
  const std::size_t N = terms.size();
  c_obst.resize(N); c_path.resize(N); c_align.resize(N); c_goal.resize(N); c_center.resize(N);

  for (std::size_t i = 0; i < N; ++i) {
    const auto& row = terms[i];
    // Best-effort extraction with bounds checks
    c_obst[i]   = (row.size() > 0) ? row[0] : 0.0;
    c_path[i]   = (row.size() > 1) ? row[1] : 0.0;
    c_align[i]  = (row.size() > 2) ? row[2] : 0.0;
    c_goal[i]   = (row.size() > 3) ? row[3] : 0.0;
    c_center[i] = (row.size() > 4) ? row[4] : 0.0;
  }
}

CostEvaluator::Result
CostEvaluator::evaluate(const std::vector<VelPair>& samples,
                        const std::vector<std::vector<double>>& terms) const
{
  Result res;

  const std::size_t N = std::min(samples.size(), terms.size());
  if (N == 0) {
    // Nothing to evaluate
    res.total_costs.clear();
    res.best_cost_terms.assign(5, 0.0);
    res.best_index = -1;
    res.best_cmd = VelPair{0.0, 0.0};
    res.normalized_terms.clear();
    return res;
  }

  // 1) Split columns
  std::vector<double> c_obst, c_path, c_align, c_goal, c_center;
  split_columns(terms, c_obst, c_path, c_align, c_goal, c_center);

  // Trim to N if needed
  c_obst.resize(N); c_path.resize(N); c_align.resize(N); c_goal.resize(N); c_center.resize(N);

  // 2) Normalize (Python-compatible behavior with robustness)
  auto obst_norm   = obstacle_normalize(c_obst);
  auto path_norm   = minmax_normalize(c_path);
  auto align_norm  = minmax_normalize(c_align);
  auto goal_norm   = minmax_normalize(c_goal);
  auto center_norm = minmax_normalize(c_center);

  // Store normalized matrix for debugging/plotting
  res.normalized_terms.resize(N, std::vector<double>(5, 0.0));
  for (std::size_t i = 0; i < N; ++i) {
    res.normalized_terms[i][0] = obst_norm[i];
    res.normalized_terms[i][1] = path_norm[i];
    res.normalized_terms[i][2] = align_norm[i];
    res.normalized_terms[i][3] = goal_norm[i];
    res.normalized_terms[i][4] = center_norm[i];
  }

  // 3) Weighted sum and best selection
  res.total_costs.assign(N, std::numeric_limits<double>::infinity());

  double best_cost = std::numeric_limits<double>::infinity();
  int best_idx = -1;

  for (std::size_t i = 0; i < N; ++i) {
    const double c_obs = obst_norm[i];

    // Skip collisions: negative obstacle cost indicates collision
    if (c_obs < 0.0) continue;

    const double critic =
        w_obst_   * c_obs +
        w_path_   * path_norm[i] +
        w_align_  * align_norm[i] +
        w_goal_   * goal_norm[i] +
        w_center_ * center_norm[i];

    res.total_costs[i] = critic;

    if (critic < best_cost) {
      best_cost = critic;
      best_idx  = static_cast<int>(i);
    }
  }

  res.best_index = best_idx;
  if (best_idx >= 0) {
    res.best_cmd = samples[static_cast<std::size_t>(best_idx)];
    // normalized terms (not weighted) for the best sample (size 5)
    res.best_cost_terms = {
      obst_norm[best_idx],
      path_norm[best_idx],
      align_norm[best_idx],
      goal_norm[best_idx],
      center_norm[best_idx]
    };
  } else {
    res.best_cmd = VelPair{0.0, 0.0};
    res.best_cost_terms.assign(5, 0.0);
  }

  return res;
}

} // namespace omo_dwa
