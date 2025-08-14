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
    this->declare_parameter<std::vector<double>>("corners", {10.0, 0.0});
    this->declare_parameter<double>("goal_tolerance", 0.2);   // [m]
    this->declare_parameter<double>("lookahead_max", 5.0);     // [m]
    this->declare_parameter<double>("lookahead_min", 0.05);    // [m] set 0.0 to disable
    this->declare_parameter<double>("timer_period", 0.05);     // [s]
    this->declare_parameter<bool>("advance_when_reached", true);
    this->declare_parameter<bool>("loop_corners", false);

    // Load params
    std::vector<double> pts = this->get_parameter("corners").as_double_array();
    if (pts.size() < 2 || (pts.size() % 2) != 0) {
      throw std::runtime_error("corners must be a flat list of [x0, y0, x1, y1, ...]");
    }
    goals_.reserve(pts.size() / 2);
    for (size_t i = 0; i + 1 < pts.size(); i += 2) {
      goals_.emplace_back(pts[i], pts[i + 1]);
    }

    tol_      = this->get_parameter("goal_tolerance").as_double();
    Lmax_     = this->get_parameter("lookahead_max").as_double();
    Lmin_     = this->get_parameter("lookahead_min").as_double();
    period_   = this->get_parameter("timer_period").as_double();
    advance_  = this->get_parameter("advance_when_reached").as_bool();
    loop_     = this->get_parameter("loop_corners").as_bool();

    // --- ROS I/O ---
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10, std::bind(&CarrotNode::odom_cb, this, _1));

    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_point", 10);
    gpath_seg_pub_   = this->create_publisher<nav_msgs::msg::Path>("/global_path_segment", 10);
    gpath_param_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/global_path_params", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_),
      std::bind(&CarrotNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(),
      "CarrotNode: tol=%.2fm, Lmax=%.2fm, Lmin=%.2fm, goals=%zu, advance=%s, loop=%s",
      tol_, Lmax_, Lmin_, goals_.size(),
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

  // --- Timer loop ---
  void timer_cb()
  {
    // Require odometry and at least one goal
    if (!state_.valid || goals_.empty()) {
      return;
    }

    // Current goal (in odom/world frame)
    const auto &goal = goals_[idx_];
    const double gx = goal.first;
    const double gy = goal.second;

    // 1) World offset to goal and distance
    const double dx_w = gx - state_.x;
    const double dy_w = gy - state_.y;
    const double dist = std::hypot(dx_w, dy_w);

    // 2) Reached goal?
    if (dist <= tol_) {
      if (advance_) {
        if (idx_ + 1 < goals_.size()) {
          ++idx_;
          RCLCPP_INFO(this->get_logger(), "Reached goal %zu, advancing to %zu", idx_ - 1, idx_);
        } else if (loop_ && goals_.size() > 1) {
          idx_ = 0;
          RCLCPP_INFO(this->get_logger(), "Reached last goal, looping to index 0");
        } else {
          // Stop at last goal: publish (0,0) and idle
          publish_target(0.0, 0.0);
          return;
        }
      } else {
        publish_target(0.0, 0.0);
        return;
      }
    }

    // 3) Transform vector into base_link: (ex, ey) = R(-yaw) * [dx_w, dy_w]
    const double cy = std::cos(state_.yaw);
    const double sy = std::sin(state_.yaw);
    const double ex =  cy * dx_w + sy * dy_w;
    const double ey = -sy * dx_w + cy * dy_w;

    const double r = std::hypot(ex, ey);
    if (r < 1e-6) {
      publish_target(0.0, 0.0);
      return;
    }

    // 4) Carrot length L = clamp(dist, Lmin, Lmax), but do not exceed remaining distance
    double L = std::min(dist, Lmax_);
    if (Lmin_ > 0.0) {
      L = std::max(Lmin_, L);
      L = std::min(L, dist);
    }

    const double k = L / r;
    const double tx = ex * k;   // target in base_link
    const double ty = ey * k;

    // 5) Publish outputs
    publish_target(tx, ty);
    publish_path_seg(state_.x, state_.y, gx, gy);       // segment in odom/world
    publish_path_params(state_.x, state_.y, tx, ty);    // A,B,C for local straight segment
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
  double tol_{0.2};
  double Lmax_{5.0};
  double Lmin_{0.05};
  double period_{0.05};
  bool   advance_{true};
  bool   loop_{false};

  // Goals in odom/world frame
  std::vector<std::pair<double,double>> goals_;
  size_t idx_{0};

  // Robot state (odom)
  State state_;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
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
