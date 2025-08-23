cmake_minimum_required(VERSION 3.8)
project(nav2_astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)

add_library(
  ${PROJECT_NAME}_plugin SHARED
  src/astar_planner.cpp  
)

ament_target_dependencies(
  ${PROJECT_NAME}_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  nav2_util
)

target_include_directories(
  ${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(
  nav2_core  
  global_planner_plugin.xml  
)

install(
  TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(
  rclcpp
  nav2_core
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  
  # Add unit tests
  ament_add_gtest(test_astar_planner_unit
    ../test/test_astar_planner.cpp
  )
  
  if(TARGET test_astar_planner_unit)
    target_link_libraries(test_astar_planner_unit
      ${PROJECT_NAME}_plugin
    )
    ament_target_dependencies(test_astar_planner_unit
      rclcpp
      nav2_core
      nav2_costmap_2d
      geometry_msgs
      nav_msgs
    )
  endif()
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
