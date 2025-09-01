// include/omo_dwa_planner/carrot_node_vlm.hpp
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
 * @brief "Carrot" waypoint generator (VLM-driven, anchor unified).
 *
 * 입력:
 *  - /odometry/filtered (nav_msgs/Odometry)
 *  - /vlm_selected_point (geometry_msgs/PointStamped, base_link, z 무시)
 *
 * 출력:
 *  - /target_point (geometry_msgs/PointStamped, base_link)
 *  - /target_center_point (geometry_msgs/PointStamped, base_link)
 *  - /global_path_segment (nav_msgs/Path, odom)
 *  - /global_path_params (geometry_msgs/Vector3Stamped, odom; vector=(A,B,C))
 *
 * 파라미터:
 *  - goal_tolerance, lookahead_max, lookahead_min, timer_period
 *  - advance_when_reached, loop_corners
 *  - center_offset (anchor→target 방향으로 적용)
 */
class CarrotNode : public rclcpp::Node {
public:
  CarrotNode();

private:
  // --- Callbacks ---
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void vlm_target_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void set_anchor_to_current_pose();
  void advance_to_next_goal();  // kept for compatibility
  void timer_cb();

  // --- Publishers (helpers) ---
  void publish_target(double x, double y);
  void publish_center_target(double tx_b, double ty_b, double* world_x_out, double* world_y_out);
  void publish_path_seg(double x0, double y0, double xg, double yg);
  void publish_path_params(double x0, double y0, double xg, double yg);

  // --- Parameters ---
  double tol_{0.2};
  double Lmax_{5.0};
  double Lmin_{0.05};
  double period_{0.05};
  bool   advance_{true};
  bool   loop_{false};
  bool   has_goal_{false};

  double center_offset_{-0.3};
  double goal_xb_{0.0};
  double goal_yb_{0.0};

  // Base-link relative targets (single element used)

  size_t idx_{0};

  // Anchor pose (공통 기준) — 오차계산 + center_offset 방향 둘 다 여기에 통합
  double anchor_x_{0.0}, anchor_y_{0.0}, anchor_yaw_{0.0};
  bool   anchor_valid_{false};
  bool   has_active_goal_{false};

  // Robot state (odom)
  struct State {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    bool   valid{false};
  } state_;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vlm_target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_center_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gpath_seg_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gpath_param_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace omo_dwa
