// src/carrot_node.cpp
#include <memory>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

using std::placeholders::_1;

namespace {

// REP-103 yaw from quaternion
inline double yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

struct State {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool   valid{false};
};

} // namespace

namespace omo_dwa {

class CarrotNode : public rclcpp::Node
{
public:
  CarrotNode()
  : rclcpp::Node("carrot_node")
  {
    // --- Parameters (mirrors the Python node) ---
    // corners: flattened [x0, y0, x1, y1, ...] in odom/world frame
    this->declare_parameter<std::vector<double>>("relative_targets", {10.0,0.0});

    this->declare_parameter<double>("goal_tolerance", 0.3);   // [m]
    this->declare_parameter<double>("lookahead_max", 4.0);     // [m]
    this->declare_parameter<double>("lookahead_min", 0.05);    // [m] set 0.0 to disable
    this->declare_parameter<double>("timer_period", 0.05);     // [s]
    this->declare_parameter<bool>("advance_when_reached", true);
    this->declare_parameter<bool>("loop_corners", false);
    this->declare_parameter<double>("center_offset", -0.3);


    // Load params
    std::vector<double> rel_flat = this->get_parameter("relative_targets").as_double_array();
    if (rel_flat.empty() || (rel_flat.size() % 2) != 0) {
      throw std::runtime_error("relative_targets must be [xb0, yb0, xb1, yb1, ...] in base_link");
    }
    rel_targets_b_.reserve(rel_flat.size() / 2);
    for (size_t i = 0; i + 1 < rel_flat.size(); i += 2) {
      rel_targets_b_.emplace_back(rel_flat[i], rel_flat[i + 1]);
    }


    tol_      = this->get_parameter("goal_tolerance").as_double();
    Lmax_     = this->get_parameter("lookahead_max").as_double();
    Lmin_     = this->get_parameter("lookahead_min").as_double();
    period_   = this->get_parameter("timer_period").as_double();
    advance_  = this->get_parameter("advance_when_reached").as_bool();
    loop_     = this->get_parameter("loop_corners").as_bool();
    center_offset_  = this->get_parameter("center_offset").as_double();

    // --- ROS I/O ---
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, std::bind(&CarrotNode::odom_cb, this, _1));

    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
    target_center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_center_point", 10);

    gpath_seg_pub_   = this->create_publisher<nav_msgs::msg::Path>("/global_path_segment", 10);
    gpath_param_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/global_path_params", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_),
      std::bind(&CarrotNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(),
      "CarrotNode: tol=%.2fm, Lmax=%.2fm, Lmin=%.2fm, goals=%zu, advance=%s, loop=%s",
      tol_, Lmax_, Lmin_, rel_targets_b_.size(),
      (advance_ ? "true" : "false"), (loop_ ? "true" : "false"));
  }

private:
  // --- Subscriptions ---
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    state_.x = p.x;
    state_.y = p.y;
    state_.yaw = yaw_from_quat(q);
    state_.valid = true;
  }
  void set_anchor_to_current_pose()
  {
    anchor_x_ = state_.x;
    anchor_y_ = state_.y;
    anchor_yaw_ = state_.yaw;
    anchor_valid_ = true;
  }
  void advance_to_next_goal()
  {
    ++idx_;
    if (idx_ >= rel_targets_b_.size()) {
      if (loop_ && !rel_targets_b_.empty()) idx_ = 0;
      else { has_active_goal_ = false; return; }
    }
    set_anchor_to_current_pose();
    has_active_goal_ = true;
  }


  // --- Timer loop ---
  void timer_cb()
  {
    if (!state_.valid) return;
    if (rel_targets_b_.empty()) return;

    // 1) 초기 앵커 세팅
    if (!anchor_valid_) set_anchor_to_current_pose();
    if (!has_active_goal_) { has_active_goal_ = true; }

    // 2) 현재 상대 타겟 (전환 당시 base_link = B_A 기준 좌표)
    const auto [xb, yb] = rel_targets_b_[idx_];

    // 3) 현재 odom을 앵커 프레임(B_A)로 역변환: p_k^{B_A} = R(-θ_A) * (p_k^W - p_A^W)
    const double dxA = state_.x - anchor_x_;
    const double dyA = state_.y - anchor_y_;
    const double cA  = std::cos(anchor_yaw_), sA = std::sin(anchor_yaw_);
    const double xk_A =  cA * dxA + sA * dyA;
    const double yk_A = -sA * dxA + cA * dyA;

    // 4) 앵커 프레임 오차: e^{B_A} = g^{B_A} - p_k^{B_A}
    double ex_A = xb - xk_A;
    double ey_A = yb - yk_A;
    double d = std::hypot(ex_A, ey_A);

    // 5) 도달 판정 (앵커 프레임/현재 프레임 어느 쪽이든 길이는 동일)
    if (d <= tol_) {
      if (advance_) {
        advance_to_next_goal();                   // 앵커를 지금으로 갱신
        if (!has_active_goal_) { publish_target(0.0, 0.0); return; }

        // 새 타겟으로 오차 재계산
        const auto [xb2, yb2] = rel_targets_b_[idx_];
        const double dxA2 = state_.x - anchor_x_;
        const double dyA2 = state_.y - anchor_y_;
        const double cA2  = std::cos(anchor_yaw_), sA2 = std::sin(anchor_yaw_);
        const double xk_A2 =  cA2 * dxA2 + sA2 * dyA2;
        const double yk_A2 = -sA2 * dxA2 + cA2 * dyA2;
        ex_A = xb2 - xk_A2;
        ey_A = yb2 - yk_A2;
        d = std::hypot(ex_A, ey_A);
      } else {
        publish_target(0.0, 0.0);
        return;
      }
    }

    if (d < 1e-9) { publish_target(0.0, 0.0); return; }

    // 6) 현재 base_link로 회전만 적용: e^{B_k} = R(θ_k - θ_A) * e^{B_A}
    const double dth = state_.yaw - anchor_yaw_;
    const double cd  = std::cos(dth), sd = std::sin(dth);
    const double ex_B =  cd * ex_A + sd * ey_A;
    const double ey_B = -sd * ex_A + cd * ey_A;

    const double r = std::hypot(ex_B, ey_B);
    if (r < 1e-9) { publish_target(0.0, 0.0); return; }

    // 7) 카롯 길이 클램프 → base_link 카롯
    double L = std::min(d, Lmax_);
    if (Lmin_ > 0.0) L = std::max(Lmin_, L);
    L = std::min(L, d);
    const double k = L / r;
    const double tx_b = ex_B * k;
    const double ty_b = ey_B * k;

    double px_w = 0.0, py_w = 0.0; 

    // 5) Publish outputs
    publish_target(tx_b, ty_b);
    publish_center_target(tx_b, ty_b, &px_w, &py_w);
    publish_path_seg(0., 0., px_w, py_w);       // segment in odom/world
    publish_path_params(0., 0., px_w, py_w);    // A,B,C for local straight segment
  }

  // --- Publishers (helpers) ---

  // Single carrot target for DWA: base_link frame
  void publish_target(double x, double y)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = "base_link";                         // IMPORTANT: relative to robot
    msg.header.stamp = this->now();
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = 0.0;
    target_pub_->publish(msg);
  }

  void publish_center_target(double tx_b, double ty_b, double* px_out, double* py_out)
    {
      // 1) base_link -> world(odom)
      const double cy = std::cos(state_.yaw);
      const double sy = std::sin(state_.yaw);
      const double px = state_.x + (cy * tx_b - sy * ty_b);
      const double py = state_.y + (sy * tx_b + cy * ty_b);
      if (px_out) *px_out = px;
      if (py_out) *py_out = py;
      // 2) unit vector from origin to (px,py) in world
      const double r = std::hypot(px, py);
      double nx_w = px, ny_w = py;
      if (r > 1e-9) {
        const double ux = px / r;
        const double uy = py / r;
        // 3) center_offset 적용 (world)
        nx_w = px + center_offset_ * ux;
        ny_w = py + center_offset_ * uy;
      }

      // 4) world -> base_link (되돌리기)
      const double dx = nx_w - state_.x;
      const double dy = ny_w - state_.y;
      const double nx_b =  cy * dx + sy * dy;
      const double ny_b = -sy * dx + cy * dy;

      // 5) publish in base_link
      geometry_msgs::msg::PointStamped msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "base_link";
      msg.point.x = nx_b;
      msg.point.y = ny_b;
      msg.point.z = 0.0;
      target_center_pub_->publish(msg);
    }


  // Publish a 2-point global path segment in odom frame
  void publish_path_seg(double x0, double y0, double xg, double yg)
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
    path.poses.push_back(mk_pose(x0, y0));
    path.poses.push_back(mk_pose(xg, yg));
    gpath_seg_pub_->publish(path);
  }

  // Publish (A,B,C) parameters for the current line in odom frame
  // If the line length is near-zero, publish zeros.
  void publish_path_params(double x0, double y0, double xg, double yg)
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
      const double C = (dx * y0 - dy * x0) / L;  // line: Ax + By + C = 0
      msg.vector.x = A;
      msg.vector.y = B;
      msg.vector.z = C;
    }
    gpath_param_pub_->publish(msg);
  }

private:
  // Parameters
  double tol_{0.3};
  double Lmax_{4.0};
  double Lmin_{0.05};
  double period_{0.05};
  bool   advance_{true};
  bool   loop_{false};
  double   center_offset_{-0.3};

  // RELATIVE base_link offsets sequence
  std::vector<std::pair<double,double>> rel_targets_b_;
  size_t idx_{0};

  // Anchor pose at the switching moment
  double anchor_x_{0.0}, anchor_y_{0.0}, anchor_yaw_{0.0};
  bool   anchor_valid_{false};

  bool   has_active_goal_{false};

  // Robot state (odom)
  State state_;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_center_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gpath_seg_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gpath_param_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace omo_dwa

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omo_dwa::CarrotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
