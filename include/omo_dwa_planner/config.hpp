// include/omo_dwa_planner/config.hpp
#pragma once
#include <string>

namespace omo_dwa {

struct Config {
  // Sampling & sim
  double sim_period_hz{20.0};
  double dt{0.2};
  double sim_time{6.0};
  int    v_samples{15};
  int    w_samples{15};

  // Sensor mounting offsets (base_link frame)
  double scan_offset_x{-0.3};  // sensor offset along +x (forward). negative: sensor behind base
  double scan_offset_y{ 0.0};  // sensor offset along +y (left)

  // Limits
  double acc_lim_v{5.0};
  double acc_lim_w{8.0};
  double v_min{0.0};
  double v_max{1.5};
  double w_min{-2.0};
  double w_max{ 2.0};

  // Costmap (bottom-centered)
  double map_width{4.0};
  double map_height{4.0};
  double cell_resolution{0.1};
  double min_z_threshold{-0.15};
  double max_z_threshold{ 0.40};
  double robot_radius{0.35};
  double inflation_radius{0.30};
  double cost_scaling_factor{10.0};
  std::string robot_base_frame{"base_link"};

  // Distance costs tuning
  double goal_corridor_radius{0.30}; // corridor half-width for goal_center_cost (meters)
  double align_xshift{-0.30};        // body-forward offset used in alignment_cost (meters)
  double align_yshift{ 0.00};        // body-left offset used in alignment_cost (meters)

  // Weights for evaluator (still raw, normalization done in evaluator)
  double w_obstacle{0.4};
  double w_path{0.2};
  double w_alignment{0.2};
  double w_goal{0.3};
  double w_goal_center{0.1};

  // NOTE: obstacle_cost (int) was unused; removed to avoid confusion.
};

} // namespace omo_dwa
