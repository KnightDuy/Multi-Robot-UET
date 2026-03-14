cmake_minimum_required(VERSION 3.5)
project(nav2_simple_astar_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/simple_astar_planner.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  std_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_costmap_2d
  nav2_core
  nav2_smac_planner
  pluginlib
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES simple_astar_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core simple_astar_planner_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  std_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_costmap_2d
  nav2_core
  nav2_smac_planner
  pluginlib
)

ament_package()
