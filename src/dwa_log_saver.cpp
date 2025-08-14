#include "omo_dwa_planner/dwa_log_saver.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cassert>
#include <iostream>
namespace fs = std::filesystem;

namespace omo_dwa {

static inline std::string to_json_string(const std::string& s) {
  std::ostringstream oss;
  oss << '"';
  for (char c : s) {
    switch (c) {
      case '"':  oss << "\\\""; break;
      case '\\': oss << "\\\\"; break;
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
    // Disable if we cannot create directory
    enabled_ = false;
    return;
  }

  // Write params.json if config snapshot is present
  write_params_json_unlocked();

  started_ = true;
}

void DwaLogSaver::end_run()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!enabled_ || saved_) return;

  // Write run data & costmaps & diagnostics
  write_run_data_json_unlocked();
  write_costmap_meta_unlocked();
  write_costmaps_unlocked();
  write_meta_json_unlocked();
  write_diag_json_unlocked();

  saved_ = true;
}

void DwaLogSaver::log_iteration(
    int tick_idx,
    const std::vector<VelPair>& samples,
    const std::vector<double>& total_costs,
    const std::vector<double>& best_cost_terms,
    const std::optional<std::vector<std::vector<double>>>& normalized_terms)
{
  std::lock_guard<std::mutex> lk(mtx_);

  if (!enabled_ || !started_) {
    // Count as skipped if logging is disabled or not started.
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
    // Keep alignment with ticks_: push empty placeholder to maintain indices
    norm_TCS_.push_back(std::vector<std::vector<double>>{});
  }

  ticks_.push_back(tick_idx);
  vel_pairs_T_.push_back(samples);
  total_costs_T_.push_back(total_costs);
  best_terms_T_.push_back(best_cost_terms);

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

// -------------------- Writers (JSON/CSV) --------------------

void DwaLogSaver::write_params_json_unlocked()
{
  if (!cfg_snapshot_.has_value()) return;

  const auto& c = cfg_snapshot_.value();
  std::ofstream ofs(run_dir_ + "/params.json");
  if (!ofs) return;

  ofs << "{\n";
  ofs << "  \"sim_period_hz\": "      << c.sim_period_hz      << ",\n";
  ofs << "  \"dt\": "                  << c.dt                  << ",\n";
  ofs << "  \"sim_time\": "            << c.sim_time            << ",\n";
  ofs << "  \"v_samples\": "           << c.v_samples           << ",\n";
  ofs << "  \"w_samples\": "           << c.w_samples           << ",\n";
  ofs << "  \"acc_lim_v\": "           << c.acc_lim_v           << ",\n";
  ofs << "  \"acc_lim_w\": "           << c.acc_lim_w           << ",\n";
  ofs << "  \"v_min\": "               << c.v_min               << ",\n";
  ofs << "  \"v_max\": "               << c.v_max               << ",\n";
  ofs << "  \"w_min\": "               << c.w_min               << ",\n";
  ofs << "  \"w_max\": "               << c.w_max               << ",\n";
  ofs << "  \"map_width\": "           << c.map_width           << ",\n";
  ofs << "  \"map_height\": "          << c.map_height          << ",\n";
  ofs << "  \"cell_resolution\": "     << c.cell_resolution     << ",\n";
  ofs << "  \"min_z_threshold\": "     << c.min_z_threshold     << ",\n";
  ofs << "  \"max_z_threshold\": "     << c.max_z_threshold     << ",\n";
  ofs << "  \"robot_radius\": "        << c.robot_radius        << ",\n";
  ofs << "  \"inflation_radius\": "    << c.inflation_radius    << ",\n";
  ofs << "  \"cost_scaling_factor\": " << c.cost_scaling_factor << ",\n";
  ofs << "  \"robot_base_frame\": "    << to_json_string(c.robot_base_frame) << ",\n";
  ofs << "  \"scan_offset_x\": "       << c.scan_offset_x       << ",\n";
  ofs << "  \"scan_offset_y\": "       << c.scan_offset_y       << ",\n";
  ofs << "  \"goal_corridor_radius\": "<< c.goal_corridor_radius<< ",\n";
  ofs << "  \"align_xshift\": "        << c.align_xshift        << ",\n";
  ofs << "  \"align_yshift\": "        << c.align_yshift        << ",\n";
  ofs << "  \"w_obstacle\": "          << c.w_obstacle          << ",\n";
  ofs << "  \"w_path\": "              << c.w_path              << ",\n";
  ofs << "  \"w_alignment\": "         << c.w_alignment         << ",\n";
  ofs << "  \"w_goal\": "              << c.w_goal              << ",\n";
  ofs << "  \"w_goal_center\": "       << c.w_goal_center       << "\n";
  ofs << "}\n";
}

void DwaLogSaver::write_run_data_json_unlocked()
{
  if (ticks_.empty()) return;

  std::ofstream ofs(run_dir_ + "/run_data.json");
  if (!ofs) return;

  const int T = static_cast<int>(ticks_.size());
  const int S = S_ < 0 ? 0 : S_;
  const int C = C_; // fixed to 5

  ofs << "{\n";
  ofs << "  \"T\": " << T << ",\n";
  ofs << "  \"S\": " << S << ",\n";
  ofs << "  \"C\": " << C << ",\n";

  // ticks
  ofs << "  \"ticks\": [";
  for (int t = 0; t < T; ++t) {
    if (t) ofs << ",";
    ofs << ticks_[t];
  }
  ofs << "],\n";

  // best_cost_terms: [T][5]
  ofs << "  \"best_cost_terms\": [\n";
  for (int t = 0; t < T; ++t) {
    ofs << "    [";
    for (int j = 0; j < 5; ++j) {
      if (j) ofs << ",";
      ofs << best_terms_T_[t][j];
    }
    ofs << "]";
    ofs << (t+1<T ? ",\n" : "\n");
  }
  ofs << "  ],\n";

  // total_costs: [T][S]
  ofs << "  \"total_costs\": [\n";
  for (int t = 0; t < T; ++t) {
    ofs << "    [";
    for (int i = 0; i < S; ++i) {
      if (i) ofs << ",";
      ofs << total_costs_T_[t][i];
    }
    ofs << "]";
    ofs << (t+1<T ? ",\n" : "\n");
  }
  ofs << "  ],\n";

  // vel_pairs: [T][S][2]
  ofs << "  \"vel_pairs\": [\n";
  for (int t = 0; t < T; ++t) {
    ofs << "    [";
    for (int i = 0; i < S; ++i) {
      if (i) ofs << ",";
      ofs << "[" << vel_pairs_T_[t][i].v << "," << vel_pairs_T_[t][i].w << "]";
    }
    ofs << "]";
    ofs << (t+1<T ? ",\n" : "\n");
  }
  ofs << "  ]";

  // normalized_terms (optional): [T][S][5]
  bool has_norm = !norm_TCS_.empty() && !norm_TCS_.front().empty();
  if (has_norm) {
    ofs << ",\n  \"normalized_terms\": [\n";
    for (int t = 0; t < T; ++t) {
      if (norm_TCS_[t].empty()) {
        // write zeros if missing
        ofs << "    [";
        for (int i = 0; i < S; ++i) {
          if (i) ofs << ",";
          ofs << "[0,0,0,0,0]";
        }
        ofs << "]";
      } else {
        ofs << "    [";
        for (int i = 0; i < S; ++i) {
          if (i) ofs << ",";
          ofs << "["
              << norm_TCS_[t][i][0] << ","
              << norm_TCS_[t][i][1] << ","
              << norm_TCS_[t][i][2] << ","
              << norm_TCS_[t][i][3] << ","
              << norm_TCS_[t][i][4] << "]";
        }
        ofs << "]";
      }
      ofs << (t+1<T ? ",\n" : "\n");
    }
    ofs << "  ]\n";
  } else {
    ofs << "\n";
  }

  ofs << "}\n";
}

void DwaLogSaver::write_meta_json_unlocked()
{
  std::ofstream ofs(run_dir_ + "/meta.json");
  if (!ofs) return;

  ofs << "{\n";
  ofs << "  \"save_mode\": \"fixed\",\n";
  ofs << "  \"samples_per_tick_hint\": {\n";
  ofs << "    \"v_samples\": " << (cfg_snapshot_ ? cfg_snapshot_->v_samples : -1) << ",\n";
  ofs << "    \"w_samples\": " << (cfg_snapshot_ ? cfg_snapshot_->w_samples : -1) << "\n";
  ofs << "  }\n";
  ofs << "}\n";
}

void DwaLogSaver::write_diag_json_unlocked()
{
  std::ofstream ofs(run_dir_ + "/diag.json");
  if (!ofs) return;

  ofs << "{\n";
  ofs << "  \"log_ok\": " << log_ok_ << ",\n";
  ofs << "  \"log_skipped\": " << log_skipped_ << ",\n";
  ofs << "  \"cm_added\": " << cm_maps_.size() << ",\n";
  ofs << "  \"S\": " << (S_ < 0 ? 0 : S_) << ",\n";
  ofs << "  \"C\": " << C_ << "\n";
  ofs << "}\n";
}

void DwaLogSaver::write_costmap_meta_unlocked()
{
  if (!cm_meta_written_) return;
  std::ofstream ofs(run_dir_ + "/costmap_meta.json");
  if (!ofs) return;

  ofs << "{\n";
  ofs << "  \"created_at\": " << to_json_string(now_timestamp()) << ",\n";
  ofs << "  \"resolution\": " << cm_meta_.resolution << ",\n";
  ofs << "  \"origin_x\": "  << cm_meta_.origin_x  << ",\n";
  ofs << "  \"origin_y\": "  << cm_meta_.origin_y  << ",\n";
  ofs << "  \"size_x\": "    << cm_meta_.size_x    << ",\n";
  ofs << "  \"size_y\": "    << cm_meta_.size_y    << "\n";
  ofs << "}\n";
}

void DwaLogSaver::write_costmaps_unlocked()
{
  if (cm_maps_.empty()) return;

  // Each snapshot as CSV: costmap_<k>.csv
  for (std::size_t k = 0; k < cm_maps_.size(); ++k) {
    const auto& cells = cm_maps_[k];
    const int W = cm_meta_.size_x;
    const int H = cm_meta_.size_y;

    std::ofstream ofs(run_dir_ + "/costmap_" + std::to_string(k) + ".csv");
    if (!ofs) continue;
    // Row-major dump
    for (int r = 0; r < H; ++r) {
      for (int c = 0; c < W; ++c) {
        if (c) ofs << ",";
        ofs << static_cast<int>(cells[r * W + c]);
      }
      ofs << "\n";
    }
  }
}

} // namespace omo_dwa
