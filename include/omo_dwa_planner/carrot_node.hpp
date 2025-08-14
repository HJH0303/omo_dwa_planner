// include/omo_dwa_planner/carrot_node.hpp
#pragma once

#include <vector>
#include <utility>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace omo_dwa {

/**
 * @brief "Carrot" waypoint generator node.
 *
 * This node tracks a list of world-frame waypoints ("corners") and periodically
 * publishes a lookahead target in the robot local frame (base_link) so that a
 * local planner (DWA) can follow it. It also publishes a short global path
 * segment and the (A,B,C) parameters of the current goal line in the odom frame.
 *
 * Parameters (mirror of the Python version):
 * - corners: std::vector<double> flattened as [x0,y0, x1,y1, ...] in odom/world
 * - goal_tolerance: double, radius [m] to consider a waypoint reached
 * - lookahead_max: double, maximum carrot distance [m]
 * - lookahead_min: double, minimum carrot distance [m] (0 disables)
 * - timer_period:  double, control period [s]
 * - advance_when_reached: bool, if true move to the next goal when reached
 * - loop_corners: bool, if true loop back to the first goal at the end
 *
 * Subscriptions:
 * - /odometry/filtered  (nav_msgs/Odometry)
 *
 * Publications:
 * - /target_point         (geometry_msgs/PointStamped, frame_id=base_link)
 * - /global_path_segment  (nav_msgs/Path, frame_id=odom)
 * - /global_path_params   (geometry_msgs/Vector3Stamped, vector=(A,B,C), frame_id=odom)
 */
class CarrotNode : public rclcpp::Node {
public:
  CarrotNode();

private:
  // --- Callbacks ---
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_cb();

  // --- Helpers (publishers) ---
  void publish_target(double x, double y);
  void publish_path_seg(double x0, double y0, double xg, double yg);
  void publish_path_params(double x0, double y0, double xg, double yg);

  // --- Parameters ---
  double tol_{0.2};     // goal_tolerance [m]
  double Lmax_{5.0};    // lookahead_max [m]
  double Lmin_{0.05};   // lookahead_min [m] (0 to disable)
  double period_{0.05}; // timer period [s]
  bool   advance_{true};
  bool   loop_{false};

  // --- Waypoints (odom/world frame) ---
  std::vector<std::pair<double,double>> goals_;
  std::size_t idx_{0};

  // --- Robot state (odom) ---
  struct State {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool   valid{false};
  } state_;

  // --- ROS I/O ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gpath_seg_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gpath_param_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace omo_dwa
