// include/omo_dwa_planner/trajectory_generator.hpp
#pragma once

#include <vector>
#include "omo_dwa_planner/types.hpp"
#include "omo_dwa_planner/config.hpp"

namespace omo_dwa {

/**
 * @brief Samples (v, w) pairs in a dynamic window and simulates local trajectories.
 *
 * Dynamic window is centered at current (v_now, w_now) and bounded by acceleration
 * limits over one control period (1 / sim_period_hz), and finally clamped to [v_min,v_max],
 * [w_min,w_max]. Each (v,w) pair is then integrated by a unicycle model in the robot
 * local frame for sim_time seconds with step dt.
 */
class TrajectoryGenerator {
public:
  explicit TrajectoryGenerator(const Config& cfg);

  /**
   * @brief Uniform grid sampling within the dynamic window.
   * @param v_now Current linear velocity.
   * @param w_now Current angular velocity.
   * @return Vector of (v,w) samples with size v_samples * w_samples.
   */
  std::vector<VelPair> sample_window(double v_now, double w_now) const;

  /**
   * @brief Simulate trajectories for the given samples using a unicycle model.
   * Integration starts at the local origin (x=0,y=0,yaw=0) to keep consistent
   * with local cost evaluation. If you want to start with the robot's current
   * yaw, pass it in @p start and modify the implementation accordingly.
   *
   * @param samples Velocity samples returned by sample_window().
   * @return A set of trajectories (each is a sequence of Pose2D).
   */
  TrajSet simulate(const std::vector<VelPair>& samples) const;

private:
  Config cfg_;
};

} // namespace omo_dwa
