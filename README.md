# omo_dwa_planner
DWA local planner ROS2 package for a two-wheeled mobile robot.

## Getting Started
It uses data from two sensors(2D LiDAR, Depth camera):

```
sensor_msgs/msg/LaserScan
sensor_msgs/msg/PointCloud2
```

It uses odometry data.
```
nav_msgs/msg/Odometry
```

## Installing
```
cd ~/your_ws/src
git clone https://github.com/HJH0303/omo_dwa_planner.git
cd ..
colcon build
```

## Running DWA node and carrot node
```
ros2 run omo_dwa_planner dwa_node
ros2 run omo_dwa_planner carrot_node
```
## Usage
You don’t need a global path for testing — you can simply use the carrot node with a defined set of corners.

You can use the corners parameter in the carrot node to define waypoints.

For example, in /src/carrot_node.cpp:
```
this->declare_parameter<std::vector<double>>("corners", {10.0, 0.0});
```
Example – Rectangle Path

To define a rectangular path (10 m by 5 m), you can set the corners parameter like this:

```
this->declare_parameter<std::vector<double>>(
    "corners",
    {10.0, 0.0,    // First corner
     10.0, 5.0,    // Second corner
     0.0, 5.0,     // Third corner
     0.0, 0.0}     // Fourth corner
);
```
## Algorithm
<img width="4002" height="2099" alt="sdw" src="https://github.com/user-attachments/assets/8f8970af-b0a1-41d5-959d-c35dfc57b7c5" />

## Results
TBD

## References
[1] Fox, Dieter, Wolfram Burgard, and Sebastian Thrun. "The dynamic window approach to collision avoidance." IEEE robotics & automation magazine 4.1 (2002): 23-33.

[2] https://github.com/ros-planning/navigation

