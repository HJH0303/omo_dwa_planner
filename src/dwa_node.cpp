// src/dwa_node.cpp
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "omo_dwa_planner/dwa_node.hpp"
#include "omo_dwa_planner/types.hpp"
#include "omo_dwa_planner/config.hpp"
#include "omo_dwa_planner/data_provider.hpp"
#include "omo_dwa_planner/trajectory_generator.hpp"
#include "omo_dwa_planner/distance_costs.hpp"
#include "omo_dwa_planner/obstacle_cost.hpp"
#include "omo_dwa_planner/cost_evaluator.hpp"
#include "omo_dwa_planner/dwa_log_saver.hpp"

namespace omo_dwa {

// ===== Constructor ===========================================================
DwaNodeImpl::DwaNodeImpl() : rclcpp::Node("dwa_node")
{
  // ---- Parameters ----
  cfg_.sim_period_hz        = this->declare_parameter<double>("sim_period_hz", 20.0);
  cfg_.dt                   = this->declare_parameter<double>("dt", 0.2);
  cfg_.sim_time             = this->declare_parameter<double>("sim_time", 6.0);
  cfg_.v_samples            = this->declare_parameter<int>("v_samples", 15);
  cfg_.w_samples            = this->declare_parameter<int>("w_samples", 15);

  cfg_.acc_lim_v            = this->declare_parameter<double>("acc_lim_v", 2.0);
  cfg_.acc_lim_w            = this->declare_parameter<double>("acc_lim_w", 4.0);
  cfg_.v_min                = this->declare_parameter<double>("v_min", 0.0);
  cfg_.v_max                = this->declare_parameter<double>("v_max", 0.6);
  cfg_.w_min                = this->declare_parameter<double>("w_min", -3.0);
  cfg_.w_max                = this->declare_parameter<double>("w_max",  3.0);

  cfg_.map_width            = this->declare_parameter<double>("map_width", 4.0);
  cfg_.map_height           = this->declare_parameter<double>("map_height", 4.0);
  cfg_.cell_resolution      = this->declare_parameter<double>("cell_resolution", 0.1);
  cfg_.min_z_threshold      = this->declare_parameter<double>("min_z_threshold", -0.2);
  cfg_.max_z_threshold      = this->declare_parameter<double>("max_z_threshold",  0.40);
  cfg_.robot_radius         = this->declare_parameter<double>("robot_radius", 0.6);
  cfg_.inflation_radius     = this->declare_parameter<double>("inflation_radius", 0.35);
  cfg_.cost_scaling_factor  = this->declare_parameter<double>("cost_scaling_factor", 7.0);
  cfg_.robot_base_frame     = this->declare_parameter<std::string>("robot_base_frame", "base_link");
  cfg_.scan_offset_x        = this->declare_parameter<double>("scan_offset_x", -0.3);
  cfg_.scan_offset_y        = this->declare_parameter<double>("scan_offset_y",  0.0);
  cfg_.goal_corridor_radius = this->declare_parameter<double>("goal_corridor_radius", 0.30);
  cfg_.align_xshift         = this->declare_parameter<double>("align_xshift", -0.30);
  cfg_.align_yshift         = this->declare_parameter<double>("align_yshift",  0.00);

  cfg_.w_obstacle           = this->declare_parameter<double>("w_obstacle", 0.5);
  cfg_.w_path               = this->declare_parameter<double>("w_path", 0.2);
  cfg_.w_alignment          = this->declare_parameter<double>("w_alignment", 0.2);
  cfg_.w_goal               = this->declare_parameter<double>("w_goal", 0.3);
  cfg_.w_goal_center        = this->declare_parameter<double>("w_goal_center", 0.2);

  // Visualization params
  publish_all_trajs_        = this->declare_parameter<bool>("publish_all_trajs", true);
  all_trajs_publish_hz_     = this->declare_parameter<double>("all_trajs_publish_hz", 10.0);
  all_trajs_alpha_          = this->declare_parameter<double>("all_trajs_alpha", 1.0);
  all_trajs_thickness_      = this->declare_parameter<double>("all_trajs_thickness", 0.02);
  all_trajs_stride_         = this->declare_parameter<int>("all_trajs_stride", 1);

  publish_costmap_          = this->declare_parameter<bool>("publish_costmap", true);
  costmap_publish_hz_       = this->declare_parameter<double>("costmap_publish_hz", 5.0);
  costmap_frame_            = this->declare_parameter<std::string>("costmap_frame", "base_link");

  // ---- Modules ----
  dp_         = std::make_unique<DataProvider>(this);
  trj_gen_    = std::make_unique<TrajectoryGenerator>(cfg_);
  dist_costs_ = std::make_unique<DistanceCosts>(cfg_);
  obs_cost_   = std::make_unique<ObstacleCost>(cfg_);
  evaluator_  = std::make_unique<CostEvaluator>(cfg_);

  // Logger
  const std::string log_dir = this->declare_parameter<std::string>("log_dir", "/root/omo_ws/src/omo_dwa_planner/logs");
  logger_     = std::make_unique<DwaLogSaver>(log_dir); // "" -> disabled (no-op)
  logger_->set_config(cfg_);
  logger_->begin_run();

  // ---- Publishers ----
  cmd_pub_     = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_dwa", 10);
  marker_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("dwa/best_traj", 10);\
  trajs_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("dwa/all_trajs", rclcpp::SystemDefaultsQoS());
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("dwa/local_costmap", rclcpp::SystemDefaultsQoS());

  // ---- Subscribers ----
  auto sensor_qos = rclcpp::SensorDataQoS(); // BEST_EFFORT by default

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&DwaNodeImpl::odom_cb, this, std::placeholders::_1));

  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/voxel_cloud", sensor_qos,
      std::bind(&DwaNodeImpl::pc_cb, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos,
      std::bind(&DwaNodeImpl::scan_cb, this, std::placeholders::_1));

  wp_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/target_point", 10,
      std::bind(&DwaNodeImpl::wp_cb, this, std::placeholders::_1));

  gpath_param_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/global_path_params", 10,
      std::bind(&DwaNodeImpl::gpath_param_cb, this, std::placeholders::_1));

  // ---- Timer (control loop) ----
  const double hz = std::max(1.0, cfg_.sim_period_hz);
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&DwaNodeImpl::timer_cb, this));

  RCLCPP_INFO(this->get_logger(), "DWA node started (%.1f Hz)", cfg_.sim_period_hz);
  rclcpp::on_shutdown([this](){
  if (logger_) logger_->end_run();
  });
}

DwaNodeImpl::~DwaNodeImpl() {
  if (logger_) {
    logger_->end_run();   
  }
}



// ===== Callbacks =============================================================
void DwaNodeImpl::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  dp_->set_odometry(msg);
}

void DwaNodeImpl::pc_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  dp_->set_pointcloud(msg);
}

void DwaNodeImpl::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  dp_->set_laserscan(msg);
}

void DwaNodeImpl::wp_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  dp_->set_waypoint(msg);
}

void DwaNodeImpl::gpath_param_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  dp_->set_gpath_params(msg);
}

// ===== Control loop ==========================================================
void DwaNodeImpl::timer_cb()
{
  // 1) Ready check
  auto odom = dp_->get_odometry();
  auto wp   = dp_->get_waypoint();
  if (!odom || !wp) {
    return;
  }
  obs_cost_->clear();
  if (auto scan = dp_->get_laserscan()) {
    obs_cost_->updateFromLaserScan(*scan);
  }
  if (auto cloud = dp_->get_pointcloud()) {
    obs_cost_->updateFromPointCloud2(*cloud);
  }
  // 2) Read current state (odom frame assumed)

  const double v_now = odom->twist.twist.linear.x;
  const double w_now = odom->twist.twist.angular.z;

  // 3) Sample & simulate
  auto samples = trj_gen_->sample_window(v_now, w_now);
  if (samples.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No velocity samples.");
    return;
  }
  auto trjs = trj_gen_->simulate(samples);
  publish_all_trajs_markers(trjs);

  // 4) Global path line params (Ax+By+C=0), already in the same frame as trjs
  double A = 0.0, B = 0.0, C = 0.0;
  bool have_line = false;
  if (auto gpath_params = dp_->get_gpath_params()) {
    A = static_cast<double>(gpath_params->vector.x);
    B = static_cast<double>(gpath_params->vector.y);
    C = static_cast<double>(gpath_params->vector.z);
    have_line = true;
  }
  Pose2D local_goal{wp->point.x, wp->point.y, 0.0};

  // 5) Cost terms
  std::vector<double> c_obst = obs_cost_->evaluate(trjs);

  std::vector<double> c_path, c_align;
  if (have_line) {
    c_path  = dist_costs_->path_cost(trjs, A, B, C);
    c_align = dist_costs_->alignment_cost(trjs, A, B, C, cfg_.align_xshift, cfg_.align_yshift);
  } else {
    c_path.assign(trjs.size(),  std::numeric_limits<double>::infinity());
    c_align.assign(trjs.size(), std::numeric_limits<double>::infinity());
  }

  auto c_goal   = dist_costs_->goal_cost(trjs, local_goal);
  auto c_center = dist_costs_->goal_center_cost(trjs, local_goal, cfg_.goal_corridor_radius);

  // 6) Assemble term matrix: [N x 5] = [obst, path, align, goal, center]
  const std::size_t N = samples.size();
  const int T = 5;
  std::vector<std::vector<double>> terms(N, std::vector<double>(T, 0.0));
  for (std::size_t i = 0; i < N; ++i) {
    terms[i][0] = (i < c_obst.size())   ? c_obst[i]   : 0.0;
    terms[i][1] = (i < c_path.size())   ? c_path[i]   : 0.0;
    terms[i][2] = (i < c_align.size())  ? c_align[i]  : 0.0;
    terms[i][3] = (i < c_goal.size())   ? c_goal[i]   : 0.0;
    terms[i][4] = (i < c_center.size()) ? c_center[i] : 0.0;
  }

  // 7) Evaluate & pick best
  auto eval = evaluator_->evaluate(samples, terms);

  // 8) Publish cmd_vel
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = eval.best_cmd.v;
  cmd.angular.z = eval.best_cmd.w;
  cmd_pub_->publish(cmd);

  // 9) Visualize best traj (optional)
if (eval.best_index >= 0 && static_cast<std::size_t>(eval.best_index) < trjs.size()){
    publish_best_traj_marker(trjs[eval.best_index]);
  }

  // 10) Publish local costmap (optional, throttled)
  if (publish_costmap_ && costmap_pub_) {
    const rclcpp::Time now = this->now();
    const double period = (costmap_publish_hz_ > 0.0) ? (1.0 / costmap_publish_hz_) : 0.0;
    if (period <= 0.0 ||
        (last_costmap_pub_.nanoseconds() == 0) ||
        (now - last_costmap_pub_) >= rclcpp::Duration::from_seconds(period))
    {
      auto grid = obs_cost_->getCostmapMsg();   // bottom-center origin handled inside
      grid.header.stamp = now;
      grid.header.frame_id = costmap_frame_;    // usually "base_link"
      costmap_pub_->publish(grid);
      last_costmap_pub_ = now;
      if (logger_) {
        std::vector<int16_t> cells;
        cells.reserve(grid.data.size());
        for (int8_t v : grid.data) cells.push_back(static_cast<int16_t>(v));

        DwaLogSaver::CostmapMeta meta;
        meta.resolution = grid.info.resolution;
        meta.origin_x   = grid.info.origin.position.x;
        meta.origin_y   = grid.info.origin.position.y;
        meta.size_x     = static_cast<int>(grid.info.width);
        meta.size_y     = static_cast<int>(grid.info.height);

        logger_->add_costmap(cells, meta.size_x, meta.size_y, now.seconds(), meta);
      }
    }
  }
  // 11) Logging
  logger_->log_iteration(
      static_cast<int>(tick_), samples,
      eval.total_costs, eval.best_cost_terms,
      std::optional<std::vector<std::vector<double>>>{eval.normalized_terms}
  );

  ++tick_;
}

// ===== Visualization =========================================================
void DwaNodeImpl::publish_all_trajs_markers(const TrajSet& trjs)
{
  if (!publish_all_trajs_ || !trajs_pub_) return;

  const rclcpp::Time now = this->now();
  const double period = (all_trajs_publish_hz_ > 0.0) ? (1.0 / all_trajs_publish_hz_) : 0.0;
  if (period > 0.0 && last_trajs_pub_.nanoseconds() != 0 &&
      (now - last_trajs_pub_) < rclcpp::Duration::from_seconds(period)) {
    return;
  }

  visualization_msgs::msg::Marker m;
  m.header.frame_id = cfg_.robot_base_frame;   
  m.header.stamp    = now;
  m.ns              = "dwa/all_trajs";
  m.id              = 0;                       
  m.type            = visualization_msgs::msg::Marker::LINE_LIST;
  m.action          = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;

  m.scale.x = all_trajs_thickness_;
  m.color.r = 0.0; m.color.g = 1.0f; m.color.b = 0.0f;
  m.color.a = static_cast<float>(all_trajs_alpha_);

  const int stride = std::max(1, all_trajs_stride_);

  for (const auto& traj : trjs) {
    if (traj.size() < 2) continue;

    std::size_t last = 0;

    for (std::size_t k = stride; k < traj.size(); k += stride) {
      geometry_msgs::msg::Point p0, p1;
      p0.x = traj[last].x; p0.y = traj[last].y; p0.z = 0.0;
      p1.x = traj[k].x;    p1.y = traj[k].y;    p1.z = 0.0;
      m.points.push_back(p0);
      m.points.push_back(p1);
      last = k;
    }

    if (last < traj.size() - 1) {
      geometry_msgs::msg::Point p0, p1;
      p0.x = traj[last].x;        p0.y = traj[last].y;        p0.z = 0.0;
      p1.x = traj.back().x;       p1.y = traj.back().y;       p1.z = 0.0;
      m.points.push_back(p0);
      m.points.push_back(p1);
    }
  }

  trajs_pub_->publish(m);
  last_trajs_pub_ = now;
}

void DwaNodeImpl::publish_best_traj_marker(const Trajectory& traj)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = cfg_.robot_base_frame;
  m.header.stamp = this->now();
  m.ns = "dwa";
  m.id = 1;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.02; // line width
  m.color.a = 1.0;
  m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0;

  m.points.reserve(traj.size());
  for (const auto& s : traj) {
    geometry_msgs::msg::Point pt;
    pt.x = s.x;
    pt.y = s.y;
    pt.z = 0.0;
    m.points.push_back(pt);
  }
  marker_pub_->publish(m);
}

} // namespace omo_dwa

// ===== main =================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omo_dwa::DwaNodeImpl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
