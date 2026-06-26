cmake_minimum_required(VERSION 3.8)
project(bmhs_planner_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ─── Core Algorithm Library (No ROS dependency) ─────────────────────────────
add_library(bmhs_core_lib SHARED
  src/map_processor.cpp
  src/vehicle_kinematics.cpp
  src/dijkstra.cpp
  src/bmhs_planner.cpp
  src/path_smoother.cpp
)
target_include_directories(bmhs_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bmhs_core_lib ${OpenCV_LIBS} pthread)

# ─── Planner Node (Action Server: ComputePathToPose) ────────────────────────
add_executable(bmhs_planner_node src/bmhs_planner_node.cpp)
target_link_libraries(bmhs_planner_node bmhs_core_lib)
ament_target_dependencies(bmhs_planner_node
  rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs
  std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs
)

# ─── Path Follower Node (Action Server: FollowPath) ─────────────────────────
add_executable(path_follower_node src/path_follower_node.cpp)
ament_target_dependencies(path_follower_node
  rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

# ─── Navigator Node (Coordinator / Action Client) ───────────────────────────
add_executable(bmhs_navigator_node src/bmhs_navigator_node.cpp)
ament_target_dependencies(bmhs_navigator_node
  rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs
  tf2 tf2_geometry_msgs
)

# ─── Install ─────────────────────────────────────────────────────────────────
install(TARGETS
  bmhs_planner_node
  path_follower_node
  bmhs_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bmhs_core_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
