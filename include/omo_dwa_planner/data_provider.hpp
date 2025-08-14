// include/omo_dwa_planner/data_provider.hpp
#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace omo_dwa {

/**
 * @brief Thread-safe container for latest ROS messages used by the DWA pipeline.
 *
 * This mirrors the Python DataProvider role:
 * - Store the latest Odometry, PointCloud2, LaserScan, local waypoint (PointStamped),
 *   and global-path params (Vector3Stamped) provided by subscribers.
 * - Expose simple getters for planner modules (traj gen, costs, evaluator).
 *
 * All setters/getters take/return SharedPtr and are protected by a single mutex.
 * If the pipeline needs per-field granularity later, split the mutexes per field.
 */
class DataProvider {
public:
  explicit DataProvider(rclcpp::Node* node);

  // --- Setters (called by subscriber callbacks) ---
  void set_odometry(const nav_msgs::msg::Odometry::SharedPtr& msg);
  void set_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
  void set_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
  void set_waypoint(const geometry_msgs::msg::PointStamped::SharedPtr& msg);
  void set_gpath_params(const geometry_msgs::msg::Vector3Stamped::SharedPtr& msg);

  // --- Getters (called by planner/evaluator) ---
  nav_msgs::msg::Odometry::SharedPtr get_odometry() const;
  sensor_msgs::msg::PointCloud2::SharedPtr get_pointcloud() const;
  sensor_msgs::msg::LaserScan::SharedPtr get_laserscan() const;
  geometry_msgs::msg::PointStamped::SharedPtr get_waypoint() const;
  geometry_msgs::msg::Vector3Stamped::SharedPtr get_gpath_params() const;

  // --- Utilities ---
  /// @brief Clear all stored messages (optional; use if you want to reset state).
  void clear();

private:
  rclcpp::Node* node_;  // not owned

  // Latest messages (shared across the DWA pipeline)
  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::PointCloud2::SharedPtr pc_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  geometry_msgs::msg::PointStamped::SharedPtr waypoint_;
  geometry_msgs::msg::Vector3Stamped::SharedPtr gpath_params_;

  // Single mutex guarding all fields (simple & safe).
  // Split into per-field mutexes if contention becomes an issue.
  mutable std::mutex mtx_;
};

} // namespace omo_dwa
