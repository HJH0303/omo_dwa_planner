#include "omo_dwa_planner/dwa_log_saver.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cassert>
#include <iostream>
#include <cstdint>
#include <limits>
namespace fs = std::filesystem;

namespace omo_dwa {

static inline std::string to_csv_quoted(const std::string& s) {
  std::ostringstream oss;
  oss << '"';
  for (char c : s) {
    switch (c) {
      case '"':  oss << "\"\""; break; // CSV double-quote escape
      case '\n': oss << "\\n";  break;
      case '\r': oss << "\\r";  break;
      case '\t': oss << "\\t";  break;
      default:   oss << c;      break;
    }
  }
  oss << '"';
  return oss.str();
}

DwaLogSaver::DwaLogSaver(const std::string& base_dir, bool compress)
: enabled_(!base_dir.empty()),
  compress_(compress),
  base_dir_(base_dir)
{
}

void DwaLogSaver::set_config(const Config& cfg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  cfg_snapshot_ = cfg;
}

std::string DwaLogSaver::now_timestamp()
{
  using clock = std::chrono::system_clock;
  auto tp = clock::now();
  std::time_t t = clock::to_time_t(tp);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

bool DwaLogSaver::ensure_dir(const std::string& path)
{
  std::error_code ec;
  if (fs::exists(path, ec)) return true;
  return fs::create_directories(path, ec);
}

void DwaLogSaver::begin_run()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!enabled_ || started_) return;

  // Create run directory
  run_dir_ = base_dir_ + "/" + now_timestamp();
  if (!ensure_dir(run_dir_)) {
    enabled_ = false;
    return;
  }

  // Write params.csv if config snapshot is present
  write_params_json_unlocked();

  started_ = true;
}

void DwaLogSaver::end_run()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!enabled_ || saved_) return;

  // Write run data & costmaps & diagnostics (CSV versions)
  write_run_data_json_unlocked();
  write_costmap_meta_unlocked();
  write_costmaps_unlocked();
  write_meta_json_unlocked();
  write_diag_json_unlocked();
  write_target_points_unlocked();
  write_target_center_points_unlocked();
  write_odometry_unlocked(); 
  saved_ = true;
}

void DwaLogSaver::log_iteration(
     int tick_idx,
     const std::vector<VelPair>& samples,
     const std::vector<double>& total_costs,
     const std::vector<double>& best_cost_terms,
     const std::optional<std::vector<std::vector<double>>>& normalized_terms,
     const std::optional<std::vector<std::vector<double>>>& raw_terms,
     std::optional<VelPair> best_cmd,
     int best_index)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (!enabled_ || !started_) {
    ++log_skipped_;
    std::cout<<"skip"<<std::endl;
    return;
  }

  const int S = static_cast<int>(samples.size());
  if (S <= 0) { ++log_skipped_; return; }

  if (S_ < 0) S_ = S;              // Fix S at first tick
  if (S != S_) { ++log_skipped_; return; }

  // Expect exactly 5 best terms: obst, path, align, goal, center
  if (best_cost_terms.size() != 5) { ++log_skipped_; return; }

  // Shapes check for total_costs
  if (static_cast<int>(total_costs.size()) != S_) { ++log_skipped_; return; }

  // Accept normalized_terms if provided and shape-consistent: Nx5
  bool store_norm = false;
  if (normalized_terms.has_value()) {
    const auto& M = normalized_terms.value();
    if (static_cast<int>(M.size()) == S_) {
      bool ok = true;
      for (const auto& row : M) {
        if (row.size() != 5) { ok = false; break; }
      }
      if (ok) {
        // Store in [T][S][5]
        norm_TCS_.push_back(M);
        store_norm = true;
      }
    }
  }
  if (!store_norm) {
    norm_TCS_.push_back(std::vector<std::vector<double>>{});
  }

  // Accept raw_terms (pre-normalization) if provided and shape-consistent: Nx5
  bool store_raw = false;
  if (raw_terms.has_value()) {
    const auto& R = raw_terms.value();
    if (static_cast<int>(R.size()) == S_) {
      bool ok = true;
      for (const auto& row : R) {
        if (row.size() != 5) { ok = false; break; }
      }
      if (ok) {
        raw_TCS_.push_back(R);
        store_raw = true;
      }
    }
  }
  if (!store_raw) {
    raw_TCS_.push_back(std::vector<std::vector<double>>{});
  }

  ticks_.push_back(tick_idx);
  vel_pairs_T_.push_back(samples);
  total_costs_T_.push_back(total_costs);
  best_terms_T_.push_back(best_cost_terms);

  if (best_cmd.has_value()) {
    best_cmds_.push_back(*best_cmd);
  } else {
    best_cmds_.push_back(VelPair{0.0, 0.0});
  }
  best_indices_.push_back(best_index);
  double best_cost = std::numeric_limits<double>::quiet_NaN();
  if (best_index >= 0 && best_index < static_cast<int>(total_costs.size())
      && std::isfinite(total_costs[static_cast<std::size_t>(best_index)])) {
    best_cost = total_costs[static_cast<std::size_t>(best_index)];
  }
  best_costs_.push_back(best_cost);

  ++log_ok_;
}

// -------------------- Costmap --------------------

void DwaLogSaver::add_costmap(const std::vector<int16_t>& cells_row_major,
                              int size_x, int size_y,
                              double stamp_sec,
                              const CostmapMeta& meta)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!enabled_ || !started_) return;

  if (!cm_meta_written_) {
    cm_meta_ = meta;
    cm_meta_written_ = true;
  }
  if (size_x <= 0 || size_y <= 0) return;
  if (static_cast<int>(cells_row_major.size()) != size_x * size_y) return;

  cm_stamps_.push_back(stamp_sec);
  cm_maps_.push_back(cells_row_major); // copy
}

void DwaLogSaver::add_target_point(double stamp_sec,
                                   double x, double y, double z,
                                   const std::string& frame_id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!enabled_ || !started_) return;
  target_points_.push_back(TargetPoint{stamp_sec, x, y, z, frame_id});
}
void DwaLogSaver::add_target_center_point(double stamp_sec,
                                   double x, double y, double z,
                                   const std::string& frame_id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!enabled_ || !started_) return;
  target_center_points_.push_back(TargetPoint{stamp_sec, x, y, z, frame_id});
}
void DwaLogSaver::add_odometry_tick(int t_idx,
                                    double stamp_sec,
                                    double x, double y, double yaw,
                                    double vx, double wz,
                                    const std::string& frame_id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!enabled_ || !started_) return;
  odom_ticks_.push_back(OdomTickSample{t_idx, stamp_sec, x, y, yaw, vx, wz, frame_id});
}
void DwaLogSaver::write_odometry_unlocked()
{
  if (odom_ticks_.empty()) return;
  std::ofstream ofs(run_dir_ + "/odometry.csv");
  if (!ofs) return;

  ofs << "t_idx,stamp_sec,x,y,yaw,vx,wz,frame_id\n";
  ofs << std::fixed << std::setprecision(6);
  for (const auto& o : odom_ticks_) {
    ofs << o.t_idx << ","
        << o.stamp_sec << ","
        << o.x << "," << o.y << "," << o.yaw << ","
        << o.vx << "," << o.wz << ","
        << to_csv_quoted(o.frame_id) << "\n";
  }
}
// -------------------- Writers (CSV) --------------------

void DwaLogSaver::write_params_json_unlocked()
{
  if (!cfg_snapshot_.has_value()) return;

  const auto& c = cfg_snapshot_.value();
  std::ofstream ofs(run_dir_ + "/params.csv");
  if (!ofs) return;

  ofs << "key,value\n";
  ofs << "sim_period_hz,"      << c.sim_period_hz      << "\n";
  ofs << "dt,"                  << c.dt                  << "\n";
  ofs << "sim_time,"            << c.sim_time            << "\n";
  ofs << "v_samples,"           << c.v_samples           << "\n";
  ofs << "w_samples,"           << c.w_samples           << "\n";
  ofs << "acc_lim_v,"           << c.acc_lim_v           << "\n";
  ofs << "acc_lim_w,"           << c.acc_lim_w           << "\n";
  ofs << "v_min,"               << c.v_min               << "\n";
  ofs << "v_max,"               << c.v_max               << "\n";
  ofs << "w_min,"               << c.w_min               << "\n";
  ofs << "w_max,"               << c.w_max               << "\n";
  ofs << "map_width,"           << c.map_width           << "\n";
  ofs << "map_height,"          << c.map_height          << "\n";
  ofs << "cell_resolution,"     << c.cell_resolution     << "\n";
  ofs << "min_z_threshold,"     << c.min_z_threshold     << "\n";
  ofs << "max_z_threshold,"     << c.max_z_threshold     << "\n";
  ofs << "robot_radius,"        << c.robot_radius        << "\n";
  ofs << "inflation_radius,"    << c.inflation_radius    << "\n";
  ofs << "cost_scaling_factor," << c.cost_scaling_factor << "\n";
  ofs << "robot_base_frame,"    << to_csv_quoted(c.robot_base_frame) << "\n";
  ofs << "scan_offset_x,"       << c.scan_offset_x       << "\n";
  ofs << "scan_offset_y,"       << c.scan_offset_y       << "\n";
  ofs << "goal_corridor_radius,"<< c.goal_corridor_radius<< "\n";
  ofs << "align_xshift,"        << c.align_xshift        << "\n";
  ofs << "align_yshift,"        << c.align_yshift        << "\n";
  ofs << "w_obstacle,"          << c.w_obstacle          << "\n";
  ofs << "w_path,"              << c.w_path              << "\n";
  ofs << "w_alignment,"         << c.w_alignment         << "\n";
  ofs << "w_goal,"              << c.w_goal              << "\n";
  ofs << "w_goal_center,"       << c.w_goal_center       << "\n";
}

void DwaLogSaver::write_run_data_json_unlocked()
{
  if (ticks_.empty()){
    std::cout<<"hi"<<std::endl;
    return;
  }

  const int T = static_cast<int>(ticks_.size());
  const int S = S_ < 0 ? 0 : S_;

  // ticks.csv
  {
    std::ofstream ofs(run_dir_ + "/ticks.csv");
    if (ofs) {
      ofs << "t_idx\n";
      for (int t = 0; t < T; ++t) ofs << ticks_[t] << "\n";
    }
  }

  // best_cost_terms.csv : columns = t_idx, obst,path,align,goal,center
  {
    std::ofstream ofs(run_dir_ + "/best_cost_terms.csv");
    if (ofs) {
      ofs << "t_idx,obstacle,path,alignment,goal,center\n";
      for (int t = 0; t < T; ++t) {
        ofs << ticks_[t];
        for (int j = 0; j < 5; ++j) ofs << "," << best_terms_T_[t][j];
        ofs << "\n";
      }
    }
  }

  // total_costs.csv : columns = t_idx,cost_0,...,cost_{S-1}
  {
    std::ofstream ofs(run_dir_ + "/total_costs.csv");
    if (ofs) {
      ofs << "t_idx";
      for (int i = 0; i < S; ++i) ofs << ",cost_" << i;
      ofs << "\n";
      for (int t = 0; t < T; ++t) {
        ofs << ticks_[t];
        for (int i = 0; i < S; ++i) ofs << "," << total_costs_T_[t][i];
        ofs << "\n";
      }
    }
  }

  // vel_pairs.csv : columns = t_idx,sample_idx,v,w
  {
    std::ofstream ofs(run_dir_ + "/vel_pairs.csv");
    if (ofs) {
      ofs << "t_idx,sample_idx,v,w\n";
      for (int t = 0; t < T; ++t) {
        for (int i = 0; i < S; ++i) {
          ofs << ticks_[t] << "," << i << "," << vel_pairs_T_[t][i].v << "," << vel_pairs_T_[t][i].w << "\n";
        }
      }
    }
  }

  // best_vel_pair.csv : columns = t_idx,v,w
  {
    std::ofstream ofs(run_dir_ + "/best_vel_pair.csv");
    if (ofs) {
      ofs << "t_idx,v,w\n";
      for (int t = 0; t < T; ++t) {
        ofs << ticks_[t] << "," << best_cmds_[t].v << "," << best_cmds_[t].w << "\n";
      }
    }
  }

  // best_index.csv : columns = t_idx,best_index
  {
    std::ofstream ofs(run_dir_ + "/best_index.csv");
    if (ofs) {
      ofs << "t_idx,best_index\n";
      for (int t = 0; t < T; ++t) ofs << ticks_[t] << "," << best_indices_[t] << "\n";
    }
  }

  // best_total_cost.csv : columns = t_idx,best_total_cost
  {
    std::ofstream ofs(run_dir_ + "/best_total_cost.csv");
    if (ofs) {
      ofs << "t_idx,best_total_cost\n";
      for (int t = 0; t < T; ++t) {
        double v = best_costs_[t];
        if (!std::isfinite(v)) v = 0.0;
        ofs << ticks_[t] << "," << v << "\n";
      }
    }
  }

  // normalized_terms.csv (optional) : columns = t_idx,sample_idx,obstacle,path,alignment,goal,center
  {
    bool has_norm = !norm_TCS_.empty();
    if (has_norm) {
      std::ofstream ofs(run_dir_ + "/normalized_terms.csv");
      if (ofs) {
        ofs << "t_idx,sample_idx,obstacle,path,alignment,goal,center\n";
        for (int t = 0; t < T; ++t) {
          if (norm_TCS_[t].empty()) {
            for (int i = 0; i < S; ++i) {
              ofs << ticks_[t] << "," << i << ",0,0,0,0,0\n";
            }
          } else {
            for (int i = 0; i < S; ++i) {
              const auto& row = norm_TCS_[t][i];
              ofs << ticks_[t] << "," << i << ","
                  << row[0] << "," << row[1] << "," << row[2] << "," << row[3] << "," << row[4] << "\n";
            }
          }
        }
      }
    }
  }

  // raw_cost.csv (optional) : columns = t_idx,sample_idx,obstacle,path,alignment,goal,center
  {
    bool has_raw = !raw_TCS_.empty();
    if (has_raw) {
      std::ofstream ofs(run_dir_ + "/raw_cost.csv");
      if (ofs) {
        ofs << "t_idx,sample_idx,obstacle,path,alignment,goal,center\n";
        for (int t = 0; t < T; ++t) {
          if (raw_TCS_[t].empty()) {
            for (int i = 0; i < S; ++i) {
              ofs << ticks_[t] << "," << i << ",0,0,0,0,0\n";
            }
          } else {
            for (int i = 0; i < S; ++i) {
              const auto& row = raw_TCS_[t][i];
              ofs << ticks_[t] << "," << i << ","
                  << row[0] << "," << row[1] << "," << row[2] << "," << row[3] << "," << row[4] << "\n";
            }
          }
        }
      }
    }
  }
}

void DwaLogSaver::write_meta_json_unlocked()
{
  std::ofstream ofs(run_dir_ + "/meta.csv");
  if (!ofs) return;

  ofs << "key,value\n";
  ofs << "save_mode," << "fixed" << "\n";
  ofs << "samples_per_tick_hint.v_samples," << (cfg_snapshot_ ? cfg_snapshot_->v_samples : -1) << "\n";
  ofs << "samples_per_tick_hint.w_samples," << (cfg_snapshot_ ? cfg_snapshot_->w_samples : -1) << "\n";
}

void DwaLogSaver::write_diag_json_unlocked()
{
  std::ofstream ofs(run_dir_ + "/diag.csv");
  if (!ofs) return;

  ofs << "key,value\n";
  ofs << "log_ok," << log_ok_ << "\n";
  ofs << "log_skipped," << log_skipped_ << "\n";
  ofs << "cm_added," << cm_maps_.size() << "\n";
  ofs << "S," << (S_ < 0 ? 0 : S_) << "\n";
  ofs << "C," << C_ << "\n";
}

void DwaLogSaver::write_costmap_meta_unlocked()
{
  if (!cm_meta_written_) return;
  std::ofstream ofs(run_dir_ + "/costmap_meta.csv");
  if (!ofs) return;

  ofs << "created_at,resolution,origin_x,origin_y,size_x,size_y\n";
  ofs << to_csv_quoted(now_timestamp()) << ","
      << cm_meta_.resolution << ","
      << cm_meta_.origin_x  << ","
      << cm_meta_.origin_y  << ","
      << cm_meta_.size_x    << ","
      << cm_meta_.size_y    << "\n";
}

void DwaLogSaver::write_costmaps_unlocked()
{
  if (cm_maps_.empty()) return;
  const std::string subdir = run_dir_ + "/costmap";
  if (!ensure_dir(subdir)) return;  // silently skip if cannot create

  for (std::size_t k = 0; k < cm_maps_.size(); ++k) {
    const auto& cells = cm_maps_[k];
    const int W = cm_meta_.size_x;
    const int H = cm_meta_.size_y;

    const std::string filename = subdir + "/costmap_" + std::to_string(k) + ".csv";
    std::ofstream ofs(filename);
    if (!ofs) continue;

    for (int r = 0; r < H; ++r) {
      for (int c = 0; c < W; ++c) {
        if (c) ofs << ",";
        ofs << static_cast<int>(cells[r * W + c]);
      }
      ofs << "\n";
    }
  }
}

void DwaLogSaver::write_target_points_unlocked()
{
  if (target_points_.empty()) return;
  std::ofstream ofs(run_dir_ + "/target_points.csv");
  if (!ofs) return;
  ofs << "stamp_sec,x,y,z,frame_id\n";
  for (const auto& tp : target_points_) {
    ofs << std::fixed << std::setprecision(6)
        << tp.stamp_sec << ","
        << tp.x << ","
        << tp.y << ","
        << tp.z << ","
        << to_csv_quoted(tp.frame_id) << "\n";
  }
}
void DwaLogSaver::write_target_center_points_unlocked()
{
  if (target_center_points_.empty()) return;
  std::ofstream ofs(run_dir_ + "/target_center_points.csv");
  if (!ofs) return;
  ofs << "stamp_sec,x,y,z,frame_id\n";
  for (const auto& tp : target_center_points_) {
    ofs << std::fixed << std::setprecision(6)
        << tp.stamp_sec << ","
        << tp.x << ","
        << tp.y << ","
        << tp.z << ","
        << to_csv_quoted(tp.frame_id) << "\n";
  }
}
} // namespace omo_dwa
