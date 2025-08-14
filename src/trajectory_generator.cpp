// src/trajectory_generator.cpp
#include "omo_dwa_planner/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

namespace omo_dwa {

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

TrajectoryGenerator::TrajectoryGenerator(const Config& cfg)
: cfg_(cfg)
{}

// Uniform grid sampling in the dynamic window around (v_now, w_now)
std::vector<VelPair> TrajectoryGenerator::sample_window(double v_now, double w_now) const
{
  // Control period used for accel-bounded window
  const double ctrl_dt = 1.0 / std::max(1.0, cfg_.sim_period_hz);
  const double v_lo = clamp(v_now - cfg_.acc_lim_v * ctrl_dt, cfg_.v_min, cfg_.v_max);
  const double v_hi = clamp(v_now + cfg_.acc_lim_v * ctrl_dt, cfg_.v_min, cfg_.v_max);
  const double w_center = clamp(w_now * 0.5, cfg_.w_min, cfg_.w_max);
  const double w_lo = clamp(w_center - cfg_.acc_lim_w * ctrl_dt, cfg_.w_min, cfg_.w_max);
  const double w_hi = clamp(w_center + cfg_.acc_lim_w * ctrl_dt, cfg_.w_min, cfg_.w_max);

  const int nv = std::max(1, cfg_.v_samples);
  const int nw = std::max(1, cfg_.w_samples);

  std::vector<VelPair> out;
  out.reserve(static_cast<std::size_t>(nv * nw));

  const double dv = (nv > 1) ? (v_hi - v_lo) / static_cast<double>(nv - 1) : 0.0;
  const double dw = (nw > 1) ? (w_hi - w_lo) / static_cast<double>(nw - 1) : 0.0;

  for (int i = 0; i < nv; ++i) {
    const double v = clamp(v_lo + dv * i, cfg_.v_min, cfg_.v_max);
    for (int j = 0; j < nw; ++j) {
      const double w = clamp(w_lo + dw * j, cfg_.w_min, cfg_.w_max);
      out.push_back(VelPair{v, w});
    }
  }
  // --- Force-include (0,0) like the Python version ---
  bool has_zero = false;
  for (const auto& s : out) {
    if (std::abs(s.v) < 1e-12 && std::abs(s.w) < 1e-12) { has_zero = true; break; }
  }
  if (!has_zero) {
    out.push_back(VelPair{0.0, 0.0});
  }
  // ---------------------------------------------------

  return out;
}

TrajSet TrajectoryGenerator::simulate(const std::vector<VelPair>& samples) const
{
  TrajSet set;
  set.reserve(samples.size());

  // Number of integration steps
  const double T = std::max(0.0, cfg_.sim_time);
  const double dt = std::max(1e-3, cfg_.dt);
  const int steps = std::max(1, static_cast<int>(std::ceil(T / dt)));

  for (const auto& vw : samples) {
    Trajectory trj;
    trj.reserve(static_cast<std::size_t>(steps) + 1);
    Pose2D s{0.0, 0.0, 0.0};
    trj.push_back(s);

    for (int k = 0; k < steps; ++k) {
      const double v = clamp(vw.v, cfg_.v_min, cfg_.v_max);
      const double w = clamp(vw.w, cfg_.w_min, cfg_.w_max);

      s.x   += v * std::cos(s.yaw) * dt;
      s.y   += v * std::sin(s.yaw) * dt;
      s.yaw += w * dt;

      // Normalize yaw to [-pi, pi]
      if (s.yaw > M_PI)      s.yaw -= 2.0 * M_PI;
      else if (s.yaw < -M_PI) s.yaw += 2.0 * M_PI;

      trj.push_back(s);
    }

    set.push_back(std::move(trj));
  }

  return set;
}

} // namespace omo_dwa
