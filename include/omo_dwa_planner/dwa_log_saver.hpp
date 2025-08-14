#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <optional>

#include "omo_dwa_planner/types.hpp"
#include "omo_dwa_planner/config.hpp"

namespace omo_dwa {

/**
 * @brief Lightweight run logger for DWA, modeled after the Python DwaLogSaver.
 *
 * It aggregates per-tick data in memory and writes them to a timestamped
 * directory at end_run(). Output is JSON files to avoid external deps.
 *
 * Files:
 *  - params.json          : (optional) planner config snapshot if set
 *  - run_data.json        : run logs (per-tick samples, costs, etc.)
 *  - costmap_meta.json    : costmap size/resolution/origin (once)
 *  - costmap_*.csv        : optional costmap snapshots (H x W, int) per added map
 *  - meta.json / diag.json: run metadata & diagnostics
 *
 * Thread-safety: public methods are mutex-protected; safe to call from callbacks.
 */
class DwaLogSaver {
public:
  /// Construct with a base directory. If empty, logging is disabled (no-op).
  explicit DwaLogSaver(const std::string& base_dir, bool compress = true);

  /// Optional: snapshot planner config to be written into params.json in begin_run().
  void set_config(const Config& cfg);

  /// Begin a new run: create timestamped subdir and write params.json.
  void begin_run();

  /// End the run: flush all buffered data to disk.
  void end_run();

  /**
   * @brief Log one DWA tick (minimal variant, matches current dwa_node.cpp usage).
   *
   * @param tick_idx           increasing integer tick (for bookkeeping)
   * @param samples            vector of (v,w) pairs (size S)
   * @param total_costs        combined critic per sample (size S); inf for skipped
   * @param best_cost_terms    normalized terms for the best sample (size 5: obst,path,align,goal,center)
   * @param normalized_terms   optional Nx5 normalized matrix; if provided it will be stored
   */
  void log_iteration(int tick_idx,
                     const std::vector<VelPair>& samples,
                     const std::vector<double>& total_costs,
                     const std::vector<double>& best_cost_terms,
                     const std::optional<std::vector<std::vector<double>>>& normalized_terms = std::nullopt);

  // ------------------- Optional costmap logging (generic, no ROS headers) -------------------

  struct CostmapMeta {
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    int size_x{0};
    int size_y{0};
  };

  /**
   * @brief Add a costmap snapshot (H x W) with a timestamp (seconds).
   * Data is written as CSV files on end_run(), and meta is saved once.
   *
   * @param cells_row_major  H*W cells, row-major, 16-bit signed ints preferred
   * @param size_x           W (columns)
   * @param size_y           H (rows)
   * @param stamp_sec        wall/ROS time in seconds
   * @param meta             resolution & origin & sizes. First call fixes meta.
   */
  void add_costmap(const std::vector<int16_t>& cells_row_major,
                   int size_x, int size_y,
                   double stamp_sec,
                   const CostmapMeta& meta);

private:
  // Helpers
  static std::string now_timestamp();                 // "YYYYMMDD_HHMMSS"
  static bool ensure_dir(const std::string& path);    // mkdir -p

  // Writers
  void write_params_json_unlocked();
  void write_run_data_json_unlocked();
  void write_diag_json_unlocked();
  void write_meta_json_unlocked();
  void write_costmap_meta_unlocked();
  void write_costmaps_unlocked();

private:
  mutable std::mutex mtx_;

  // Configuration / state
  bool enabled_{false};
  bool compress_{true};              // reserved (JSON not compressed)
  bool started_{false};
  bool saved_{false};

  std::optional<Config> cfg_snapshot_;
  std::string base_dir_;             // e.g., "/tmp/dwa_logs"
  std::string run_dir_;              // base_dir_ + "/" + timestamp

  // -------- per-run buffers (will be serialized) --------

  // Per-tick arrays (T grows with time)
  std::vector<int> ticks_;
  std::vector<std::vector<VelPair>> vel_pairs_T_;         // [T][S]{v,w}
  std::vector<std::vector<double>> total_costs_T_;        // [T][S]
  std::vector<std::vector<double>> best_terms_T_;         // [T][5]
  std::vector<std::vector<std::vector<double>>> norm_TCS_; // [T][S][5] (optional; empty if not provided)

  // Shapes
  int S_{-1};   // samples per tick
  int C_{5};    // number of cost terms (fixed to 5: obst,path,align,goal,center)

  // Costmap buffers
  bool cm_meta_written_{false};
  CostmapMeta cm_meta_{};
  std::vector<double> cm_stamps_;               // seconds
  std::vector<std::vector<int16_t>> cm_maps_;   // each is H*W row-major

  // Diagnostics
  int log_ok_{0};
  int log_skipped_{0};
};

} // namespace omo_dwa
