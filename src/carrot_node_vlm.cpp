// src/carrot_node_vlm.cpp
#include "omo_dwa_planner/carrot_node_vlm.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

namespace {

// REP-103 yaw from quaternion
inline double yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

} // namespace

namespace omo_dwa {

CarrotNode::CarrotNode()
: rclcpp::Node("carrot_vlm_node")
{
  // Parameters
  this->declare_parameter<double>("goal_tolerance", 0.2);   // [m]
  this->declare_parameter<double>("lookahead_max", 5.0);     // [m]
  this->declare_parameter<double>("lookahead_min", 0.05);    // [m] (0 disables)
  this->declare_parameter<double>("timer_period", 0.05);     // [s]
  this->declare_parameter<double>("center_offset", -0.3);
  // legacy (compat)
  this->declare_parameter<bool>("advance_when_reached", true);
  this->declare_parameter<bool>("loop_corners", false);

  tol_            = this->get_parameter("goal_tolerance").as_double();
  Lmax_           = this->get_parameter("lookahead_max").as_double();
  Lmin_           = this->get_parameter("lookahead_min").as_double();
  period_         = this->get_parameter("timer_period").as_double();
  center_offset_  = this->get_parameter("center_offset").as_double();
  advance_        = this->get_parameter("advance_when_reached").as_bool();
  loop_           = this->get_parameter("loop_corners").as_bool();

  // ROS I/O
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, std::bind(&CarrotNode::odom_cb, this, _1));

  // VLM-selected point is in base_link (z ignored)
  vlm_target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/vlm_selected_point", 10, std::bind(&CarrotNode::vlm_target_cb, this, _1));

  target_pub_        = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
  target_center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_center_point", 10);
  gpath_seg_pub_     = this->create_publisher<nav_msgs::msg::Path>("/global_path_segment", 10);
  gpath_param_pub_   = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/global_path_params", 10);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_),
      std::bind(&CarrotNode::timer_cb, this));

  RCLCPP_INFO(this->get_logger(),
              "CarrotNode (VLM single target): tol=%.2fm, Lmax=%.2fm, Lmin=%.2fm, center_offset=%.2f",
              tol_, Lmax_, Lmin_, center_offset_);
}

void CarrotNode::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto &p = msg->pose.pose.position;
  const auto &q = msg->pose.pose.orientation;
  state_.x   = p.x;
  state_.y   = p.y;
  state_.yaw = yaw_from_quat(q);
  state_.valid = true;
}

void CarrotNode::vlm_target_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  if (!state_.valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Ignoring /vlm_selected_point until odometry is valid.");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != "base_link") {
    RCLCPP_WARN(this->get_logger(),
                "Expected base_link frame for /vlm_selected_point but got '%s'. "
                "Interpreting as base_link (z ignored).",
                msg->header.frame_id.c_str());
  }

  // (재쿼리 포함) 새 타깃 수신 시점 = anchor를 현재 포즈로 갱신 (통합 기준)
  set_anchor_to_current_pose();

  // 단일 타깃 갱신 (base_link, z 무시)
  goal_xb_   = msg->point.x;
  goal_yb_   = msg->point.y;
  has_goal_  = true;

  RCLCPP_INFO(this->get_logger(),
              "VLM base_link target set to (%.3f, %.3f). Anchor updated.", goal_xb_, goal_yb_);
}

void CarrotNode::set_anchor_to_current_pose()
{
  anchor_x_   = state_.x;
  anchor_y_   = state_.y;
  anchor_yaw_ = state_.yaw;
  anchor_valid_ = true;
}

void CarrotNode::timer_cb()
{
  if (!state_.valid) return;
  if (!has_goal_) {
    // 목표가 없으면 (새 VLM 올 때까지) 계속 0,0 퍼블리시해서 DWA가 멈추도록 유지
    publish_target(0.0, 0.0);
    return;
  }

  // 초기 앵커 세팅
  if (!anchor_valid_) set_anchor_to_current_pose();

  // 1) 현재 odom을 앵커 프레임(B_A)로 역변환: p_k^{B_A} = R(-θ_A) * (p_k^W - p_A^W)
  const double dxA = state_.x - anchor_x_;
  const double dyA = state_.y - anchor_y_;
  const double cA  = std::cos(anchor_yaw_), sA = std::sin(anchor_yaw_);
  const double xk_A =  cA * dxA + sA * dyA;
  const double yk_A = -sA * dxA + cA * dyA;

  // 2) 앵커 프레임 오차: e^{B_A} = g^{B_A} - p_k^{B_A}
  double ex_A = goal_xb_ - xk_A;
  double ey_A = goal_yb_ - yk_A;
  double d = std::hypot(ex_A, ey_A);

  // 3) 도달 판정 (단일 타깃: 도달 시 anchor 갱신 + 0 타깃 퍼블리시 후 리턴)
  if (d <= tol_) {
    set_anchor_to_current_pose();
    has_goal_ = false;           
    publish_target(0.0, 0.0);
    return; 
  }
  if (d < 1e-9) { publish_target(0.0, 0.0); return; }

  // 4) 현재 base_link로 회전만 적용: e^{B_k} = R(θ_k - θ_A) * e^{B_A}
  const double dth = state_.yaw - anchor_yaw_;
  const double cd  = std::cos(dth), sd = std::sin(dth);
  const double ex_B =  cd * ex_A + sd * ey_A;
  const double ey_B = -sd * ex_A + cd * ey_A;

  const double r = std::hypot(ex_B, ey_B);
  if (r < 1e-9) { publish_target(0.0, 0.0); return; }

  // 5) lookahead 클램프 → base_link 카롯
  double L = std::min(d, Lmax_);
  if (Lmin_ > 0.0) L = std::max(Lmin_, L);
  L = std::min(L, d);
  const double k = L / r;
  const double tx_b = ex_B * k;
  const double ty_b = ey_B * k;

  double wx = 0.0, wy = 0.0; // center-offset 후 월드 좌표

  // 6) Publish
  publish_target(tx_b, ty_b);
  publish_center_target(tx_b, ty_b, &wx, &wy);  // anchor→target 방향으로 보정
  publish_path_seg(state_.x, state_.y, wx, wy);
  publish_path_params(state_.x, state_.y, wx, wy);
}

void CarrotNode::publish_target(double x, double y)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = this->now();
  msg.point.x = x;
  msg.point.y = y;
  msg.point.z = 0.0;
  target_pub_->publish(msg);
}

void CarrotNode::publish_center_target(double tx_b, double ty_b, double* world_x_out, double* world_y_out)
{
  // 1) base_link → world (보정 전)
  const double cy = std::cos(state_.yaw);
  const double sy = std::sin(state_.yaw);
  const double px = state_.x + (cy * tx_b - sy * ty_b);
  const double py = state_.y + (sy * tx_b + cy * ty_b);

  // 2) anchor → (px,py) 방향 단위벡터 (anchor 통합 기준)
  const double dx0 = px - anchor_x_;
  const double dy0 = py - anchor_y_;
  const double r0  = std::hypot(dx0, dy0);

  double nx_w = px, ny_w = py;  // 기본: 보정 없음
  if (r0 > 1e-9) {
    const double ux0 = dx0 / r0;
    const double uy0 = dy0 / r0;
    // 3) center_offset 적용
    nx_w = px + center_offset_ * ux0;
    ny_w = py + center_offset_ * uy0;
  }

  if (world_x_out) *world_x_out = nx_w;
  if (world_y_out) *world_y_out = ny_w;

  // 4) world → base_link (보정된 포인트를 base_link로 퍼블리시)
  const double dx = nx_w - state_.x;
  const double dy = ny_w - state_.y;
  const double nx_b =  cy * dx + sy * dy;
  const double ny_b = -sy * dx + cy * dy;

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.point.x = nx_b;
  msg.point.y = ny_b;
  msg.point.z = 0.0;
  target_center_pub_->publish(msg);
}

void CarrotNode::publish_path_seg(double x0, double y0, double xg, double yg)
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "odom";

  auto mk_pose = [&](double x, double y) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    return ps;
  };

  path.poses.clear();
  path.poses.reserve(2);
  path.poses.push_back(mk_pose(x0, y0)); // 현재 로봇 위치
  path.poses.push_back(mk_pose(xg, yg)); // 보정된 월드 목표
  gpath_seg_pub_->publish(path);
}

void CarrotNode::publish_path_params(double x0, double y0, double xg, double yg)
{
  const double dx = xg - x0;
  const double dy = yg - y0;
  const double L  = std::hypot(dx, dy);

  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";

  if (L < 1e-9) {
    msg.vector.x = 0.0;
    msg.vector.y = 0.0;
    msg.vector.z = 0.0;
  } else {
    const double A =  dy / L;
    const double B = -dx / L;
    const double C = (dx * y0 - dy * x0) / L;  // Ax + By + C = 0
    msg.vector.x = A;
    msg.vector.y = B;
    msg.vector.z = C;
  }
  gpath_param_pub_->publish(msg);
}

} // namespace omo_dwa

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omo_dwa::CarrotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
