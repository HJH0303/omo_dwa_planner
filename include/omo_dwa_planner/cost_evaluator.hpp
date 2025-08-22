#pragma once

#include <vector>
#include <cstddef>
#include "omo_dwa_planner/config.hpp"
#include "omo_dwa_planner/types.hpp"

namespace omo_dwa {

/**
 * @brief Combines multiple normalized cost terms into a single score per sample,
 *        mirroring the Python dwa_cost_evaluator.py behavior.
 *
 * Terms layout (columns in "terms" matrix):
 *   0: obstacle
 *   1: path
 *   2: alignment
 *   3: goal
 *   4: goal_center
 *
 * Normalization:
 *  - obstacle: min-max only over positive values; negatives (collisions) are kept as-is.
 *  - others:   standard min-max over the column; if range==0, use 1 to avoid div-by-zero.
 */
class CostEvaluator {
public:
  explicit CostEvaluator(const Config& cfg);

  struct Result {
    VelPair best_cmd{0.0, 0.0};            // best (v, w)
    int best_index{-1};                     // index in samples, or -1 if none
    std::vector<double> total_costs;        // critic per sample (inf for skipped)
    std::vector<double> best_cost_terms;    // normalized terms (size 5) for the best sample
    std::vector<std::vector<double>> normalized_terms; // Nx5 normalized terms (for debugging/plotting)
  };

  /**
   * @brief Evaluate weighted sum on normalized terms and pick the best sample.
   *
   * @param samples velocity samples (N)
   * @param terms   cost terms matrix (N x 5) in the fixed column order described above
   * @return Result with best command and diagnostic data
   */
  Result evaluate(const std::vector<VelPair>& samples,
                  const std::vector<std::vector<double>>& terms) const;

private:
  // Min-max normalize a column; if all values are equal, range is set to 1.
  static std::vector<double> minmax_normalize(const std::vector<double>& col);

  // Obstacle-special normalization: positive values min-max; negatives are kept as-is.
  static std::vector<double> obstacle_normalize(const std::vector<double>& col);

  // Split Nx5 matrix into 5 column vectors; if sizes mismatch, behavior is best-effort.
  static void split_columns(const std::vector<std::vector<double>>& terms,
                            std::vector<double>& c_obst,
                            std::vector<double>& c_path,
                            std::vector<double>& c_align,
                            std::vector<double>& c_goal,
                            std::vector<double>& c_center);

private:
  // Weights from config (raw; normalization is applied before weighting).
  double w_obst_;
  double w_path_;
  double w_align_;
  double w_goal_;
  double w_center_;
};

} // namespace omo_dwa
