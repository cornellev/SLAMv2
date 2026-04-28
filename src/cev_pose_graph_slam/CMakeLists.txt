cmake_minimum_required(VERSION 3.8)
project(cev_pose_graph_slam)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Use the cevicp library from the lidar_odometry submodule
option(LIB_ONLY "" ON)
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../cev_lidar_odometry_ros2/icp
    ${CMAKE_CURRENT_BINARY_DIR}/cevicp_ext
    EXCLUDE_FROM_ALL)

add_executable(pgo_node src/SLAM_backend_math.cpp)

target_link_libraries(pgo_node cevicp Eigen3::Eigen)

ament_target_dependencies(pgo_node
  rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS pgo_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
