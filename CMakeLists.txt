# CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project(omo_dwa_planner)

# --- Build options ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
# 선택: TF를 쓸 계획이면 활성화
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

# --- Core library ---
add_library(omo_dwa_core
  src/trajectory_generator.cpp
  src/distance_costs.cpp
  src/obstacle_cost.cpp
  src/cost_evaluator.cpp
  src/dwa_log_saver.cpp
  src/data_provider.cpp
  src/local_costmap2d.cpp

)

target_include_directories(omo_dwa_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(omo_dwa_core
  rclcpp
  message_filters
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  builtin_interfaces
  # tf2 tf2_ros tf2_geometry_msgs
)

# --- Nodes ---
add_executable(dwa_node src/dwa_node.cpp)
target_link_libraries(dwa_node omo_dwa_core)
ament_target_dependencies(dwa_node rclcpp)

add_executable(carrot_node src/carrot_node.cpp)
target_link_libraries(carrot_node omo_dwa_core)
ament_target_dependencies(carrot_node rclcpp)

# --- Install ---
install(TARGETS omo_dwa_core dwa_node carrot_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# (선택) 설정/런치 설치 시
# install(DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
