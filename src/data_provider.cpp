// src/data_provider.cpp
#include "omo_dwa_planner/data_provider.hpp"

namespace omo_dwa {

DataProvider::DataProvider(rclcpp::Node* node)
: node_(node)
{
  // Nothing to initialize beyond null shared pointers.
}

void DataProvider::set_odometry(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  odom_ = msg;
}

void DataProvider::set_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  pc_ = msg;
}

void DataProvider::set_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  scan_ = msg;
}

void DataProvider::set_waypoint(const geometry_msgs::msg::PointStamped::SharedPtr& msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  waypoint_ = msg;
}

void DataProvider::set_gpath_params(const geometry_msgs::msg::Vector3Stamped::SharedPtr& msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  gpath_params_ = msg;
}

nav_msgs::msg::Odometry::SharedPtr DataProvider::get_odometry() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return odom_;
}

sensor_msgs::msg::PointCloud2::SharedPtr DataProvider::get_pointcloud() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return pc_;
}

sensor_msgs::msg::LaserScan::SharedPtr DataProvider::get_laserscan() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return scan_;
}

geometry_msgs::msg::PointStamped::SharedPtr DataProvider::get_waypoint() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return waypoint_;
}

geometry_msgs::msg::Vector3Stamped::SharedPtr DataProvider::get_gpath_params() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return gpath_params_;
}

void DataProvider::clear()
{
  std::lock_guard<std::mutex> lk(mtx_);
  odom_.reset();
  pc_.reset();
  scan_.reset();
  waypoint_.reset();
  gpath_params_.reset();
}

} // namespace omo_dwa
