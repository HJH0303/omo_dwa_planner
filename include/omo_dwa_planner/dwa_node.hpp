#pragma once

#include <memory>
#include <string>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>   // publish all sampled trajectories
#include <nav_msgs/msg/occupancy_grid.hpp>           // publish local costmap

#include "omo_dwa_planner/config.hpp"
#include "omo_dwa_planner/types.hpp"
#include "omo_dwa_planner/data_provider.hpp"
#include "omo_dwa_planner/trajectory_generator.hpp"
#include "omo_dwa_planner/distance_costs.hpp"
#include "omo_dwa_planner/obstacle_cost.hpp"
#include "omo_dwa_planner/cost_evaluator.hpp"
#include "omo_dwa_planner/dwa_log_saver.hpp"

namespace omo_dwa {

/**
  DWA planner ROS2 node.
 */
class DwaNodeImpl : public rclcpp::Node {
public:
  DwaNodeImpl();
  ~DwaNodeImpl() override;

private:
  // ---- Main control loop ----
  void timer_cb();

  // ---- Subscriptions ----
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr);
  void pc_cb(const sensor_msgs::msg::PointCloud2::SharedPtr);
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr);
  void wp_cb(const geometry_msgs::msg::PointStamped::SharedPtr);
  void gpath_param_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr);

  // ---- Visualization helpers ----
  void publish_best_traj_marker(const Trajectory& traj);
  void publish_all_trajs_markers(const TrajSet& trjs);

private:
  // ---- Core modules ----
  Config cfg_;
  std::unique_ptr<DataProvider>        dp_;
  std::unique_ptr<TrajectoryGenerator> trj_gen_;
  std::unique_ptr<DistanceCosts>       dist_costs_;
  std::unique_ptr<ObstacleCost>        obs_cost_;
  std::unique_ptr<CostEvaluator>       evaluator_;
  std::unique_ptr<DwaLogSaver>         logger_;

  // ---- Publishers ----
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   marker_pub_;   // best traj
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajs_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      costmap_pub_;  // local costmap

  // ---- Subscribers ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr wp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gpath_param_sub_; // /global_path_params

  // ---- Timers ----
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Visualization publish throttling ----
  bool        publish_all_trajs_{true};
  double      all_trajs_publish_hz_{10.0};
  double      all_trajs_alpha_{0.25};
  double      all_trajs_thickness_{0.01};
  int         all_trajs_stride_{1};
  rclcpp::Time last_trajs_pub_{rclcpp::Time(0,0,RCL_ROS_TIME)};

  bool        publish_costmap_{true};
  double      costmap_publish_hz_{5.0};
  std::string costmap_frame_{"base_link"};
  rclcpp::Time last_costmap_pub_{rclcpp::Time(0,0,RCL_ROS_TIME)};

  // ---- Bookkeeping ----
  std::size_t tick_{0};
};

} // namespace omo_dwa
