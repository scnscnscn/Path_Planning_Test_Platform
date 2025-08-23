cmake_minimum_required(VERSION 3.8)
project(path_planning_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_astar_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Add gtest executable
  ament_add_gtest(test_astar_planner test_astar_planner.cpp)
  
  target_link_libraries(test_astar_planner
    # Note: In a real build environment, we would link against the nav2_astar_planner library
    # For now, this serves as a test structure template
  )
  
  ament_target_dependencies(test_astar_planner
    rclcpp
    nav2_costmap_2d
    geometry_msgs
    nav_msgs
  )
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()