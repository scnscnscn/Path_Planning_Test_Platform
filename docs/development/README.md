# Development Guidelines

This guide outlines the development standards and practices for contributing to the Path Planning Test Platform.

## Project Structure

```
Path_Planning_Test_Platform/
├── .github/                 # GitHub configuration and workflows
│   └── copilot-instructions.md
├── docs/                    # Project documentation
│   ├── api/                 # API documentation
│   ├── development/         # Development guides
│   ├── testing/             # Testing documentation
│   └── tutorials/           # User tutorials
├── custom_planners/         # Custom path planning algorithms
├── test/                    # Unit tests and testing framework
├── .clang-format           # C++ formatting configuration
├── .gitignore              # Git ignore rules
├── CONTRIBUTING.md         # Contribution guidelines
├── LICENSE                 # Apache 2.0 license
└── README.md               # Project overview
```

## Code Style Requirements

### C++ Formatting
- **Tool:** clang-format with provided `.clang-format` configuration
- **Style Base:** Google with customizations
- **Key Rules:**
  - 2-space indentation
  - Allman brace style
  - Left-aligned pointers
  - No line length limit
  - Namespace indentation

**Format Commands:**
```bash
# Format single file
clang-format -i src/my_planner.cpp

# Check formatting (dry run)
clang-format --dry-run --Werror src/my_planner.cpp

# Format all C++ files
find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

### Python Formatting
- **Standard:** PEP 8
- **Tools:** black (formatting), flake8 (checking)

**Format Commands:**
```bash
# Format Python files
black scripts/

# Check Python style
flake8 scripts/

# Fix common issues
autopep8 --in-place --aggressive scripts/my_script.py
```

## Navigation Planner Plugin Development

### Overview

Navigation planners in this platform must implement the `nav2_core::GlobalPlanner` interface to integrate with the Navigation2 framework. This section provides a complete guide for developing custom path planning algorithms.

### Plugin Development Workflow

#### 1. Project Setup

Create a new ROS2 package for your planner:

```bash
# Create package with dependencies
ros2 pkg create --build-type ament_cmake my_custom_planner \
  --dependencies rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d \
  pluginlib geometry_msgs nav_msgs nav2_util

# Navigate to package directory
cd my_custom_planner
```

#### 2. Package Configuration

**package.xml** - Define package metadata and dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_custom_planner</name>
  <version>1.0.0</version>
  <description>Custom path planner implementation</description>
  <maintainer email="WLQVincent@gmail.com">scnscnscn</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- Core ROS2 dependencies -->
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  
  <!-- Navigation2 dependencies -->
  <depend>nav2_core</depend>
  <depend>nav2_costmap_2d</depend>
  <depend>nav2_util</depend>
  <depend>pluginlib</depend>
  
  <!-- Message dependencies -->
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  
  <!-- Testing -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>gtest</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/planner_plugin.xml" />
  </export>
</package>
```

#### 3. CMake Configuration

**CMakeLists.txt** - Build configuration:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_custom_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Build planner library
add_library(${PROJECT_NAME}_plugin SHARED
  src/my_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  geometry_msgs
  nav_msgs
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Register plugin
pluginlib_export_plugin_description_file(nav2_core planner_plugin.xml)

# Install
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_dependencies(
  rclcpp
  nav2_core
  pluginlib
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_my_planner test/test_my_planner.cpp)
  target_link_libraries(test_my_planner ${PROJECT_NAME}_plugin)
  ament_target_dependencies(test_my_planner gtest)
endif()

ament_package()
```

#### 4. Plugin Declaration

**planner_plugin.xml** - Plugin metadata for ROS2 plugin system:

```xml
<library path="my_custom_planner_plugin">
  <class 
    type="my_custom_planner::MyPlanner"
    base_class_type="nav2_core::GlobalPlanner">
    <description>
      Custom path planner implementing [algorithm description]
    </description>
  </class>
</library>
```

#### 5. Header Implementation

**include/my_custom_planner/my_planner.hpp**:

```cpp
#ifndef MY_CUSTOM_PLANNER__MY_PLANNER_HPP_
#define MY_CUSTOM_PLANNER__MY_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace my_custom_planner
{
  /**
   * @brief Custom path planner class
   * 
   * Implements [algorithm name] for path planning in 2D grid environments.
   * [Brief algorithm description and key features]
   */
  class MyPlanner : public nav2_core::GlobalPlanner
  {
  public:
    MyPlanner() = default;
    ~MyPlanner() = default;

    /**
     * @brief Configure the planner
     * @param parent Weak pointer to parent lifecycle node
     * @param name Planner name for parameter namespacing
     * @param tf Shared pointer to TF buffer
     * @param costmap_ros Shared pointer to costmap
     */
    void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief Cleanup planner resources
     */
    void cleanup() override;

    /**
     * @brief Activate the planner
     */
    void activate() override;

    /**
     * @brief Deactivate the planner
     */
    void deactivate() override;

    /**
     * @brief Create a path from start to goal
     * @param start Starting pose
     * @param goal Goal pose
     * @return nav_msgs::Path Generated path (empty if no path found)
     */
    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal) override;

  private:
    // ROS2 components
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    
    // Configuration
    std::string planner_name_;
    std::string global_frame_;
    rclcpp::Logger logger_{rclcpp::get_logger("MyPlanner")};
    rclcpp::Clock::SharedPtr clock_;
    
    // Algorithm-specific parameters
    double algorithm_param1_;
    bool algorithm_param2_;
    
    // Helper methods
    bool isValidPosition(double x, double y) const;
    std::vector<geometry_msgs::msg::PoseStamped> generatePath(/* params */);
  };
}

#endif // MY_CUSTOM_PLANNER__MY_PLANNER_HPP_
```

#### 6. Implementation Template

**src/my_planner.cpp**:

```cpp
#include "my_custom_planner/my_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_custom_planner
{
  void MyPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    // Store references
    node_ = parent.lock();
    planner_name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    clock_ = node_->get_clock();

    // Declare and get parameters
    nav2_util::declare_parameter_if_not_declared(
      node_, planner_name_ + ".algorithm_param1", 
      rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
      node_, planner_name_ + ".algorithm_param2", 
      rclcpp::ParameterValue(false));

    node_->get_parameter(planner_name_ + ".algorithm_param1", algorithm_param1_);
    node_->get_parameter(planner_name_ + ".algorithm_param2", algorithm_param2_);

    RCLCPP_INFO(logger_, "Configured %s with param1=%.2f, param2=%s", 
                planner_name_.c_str(), algorithm_param1_, 
                algorithm_param2_ ? "true" : "false");
  }

  void MyPlanner::cleanup()
  {
    RCLCPP_INFO(logger_, "Cleaning up %s", planner_name_.c_str());
    // Clean up resources
  }

  void MyPlanner::activate()
  {
    RCLCPP_INFO(logger_, "Activating %s", planner_name_.c_str());
    // Activate planner
  }

  void MyPlanner::deactivate()
  {
    RCLCPP_INFO(logger_, "Deactivating %s", planner_name_.c_str());
    // Deactivate planner
  }

  nav_msgs::msg::Path MyPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();

    // Input validation
    if (start.header.frame_id != global_frame_) {
      RCLCPP_ERROR(logger_, "Start pose frame_id does not match global frame");
      return path;
    }
    
    if (goal.header.frame_id != global_frame_) {
      RCLCPP_ERROR(logger_, "Goal pose frame_id does not match global frame");
      return path;
    }

    // Implement your path planning algorithm here
    // 1. Convert world coordinates to grid coordinates
    // 2. Validate start and goal positions
    // 3. Run planning algorithm
    // 4. Convert result back to world coordinates
    // 5. Create and return path message

    auto start_time = node_->now();
    
    // TODO: Implement your algorithm
    std::vector<geometry_msgs::msg::PoseStamped> waypoints = generatePath(/*params*/);
    
    auto end_time = node_->now();
    auto planning_time = (end_time - start_time).seconds();
    
    path.poses = waypoints;
    
    RCLCPP_INFO(logger_, "Generated path with %zu waypoints in %.3f seconds", 
                waypoints.size(), planning_time);
    
    return path;
  }

  bool MyPlanner::isValidPosition(double x, double y) const
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return false; // Outside map bounds
    }
    
    unsigned char cost = costmap_->getCost(mx, my);
    return cost != nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  std::vector<geometry_msgs::msg::PoseStamped> MyPlanner::generatePath(/* params */)
  {
    std::vector<geometry_msgs::msg::PoseStamped> path;
    
    // TODO: Implement your path generation logic
    
    return path;
  }
}

// Register plugin
PLUGINLIB_EXPORT_CLASS(my_custom_planner::MyPlanner, nav2_core::GlobalPlanner)
```

#### 7. Unit Testing

**test/test_my_planner.cpp**:

```cpp
#include <gtest/gtest.h>
#include <memory>
#include "my_custom_planner/my_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

class MyPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planner_ = std::make_shared<my_custom_planner::MyPlanner>();
    // Set up test fixtures (mock costmap, tf buffer, etc.)
  }

  void TearDown() override
  {
    planner_.reset();
  }

  std::shared_ptr<my_custom_planner::MyPlanner> planner_;
};

TEST_F(MyPlannerTest, BasicFunctionality) 
{
  // Test basic planner functionality
  EXPECT_NE(planner_, nullptr);
}

TEST_F(MyPlannerTest, ValidPathGeneration)
{
  // Test path generation with valid start and goal
  geometry_msgs::msg::PoseStamped start, goal;
  // Set up start and goal poses
  
  auto path = planner_->createPlan(start, goal);
  // Verify path properties
}

TEST_F(MyPlannerTest, InvalidInputHandling)
{
  // Test error handling with invalid inputs
  geometry_msgs::msg::PoseStamped invalid_start, goal;
  // Set up invalid start pose
  
  auto path = planner_->createPlan(invalid_start, goal);
  EXPECT_TRUE(path.poses.empty());
}
```

### Implementation Guidelines

#### Algorithm Design Patterns

1. **Grid-based Search**: A*, Dijkstra, RRT*
2. **Sampling-based**: RRT, PRM
3. **Optimization-based**: TEB, DWA
4. **Hybrid**: A* with post-processing

#### Parameter Management

Use meaningful parameter names with appropriate defaults:

```cpp
// Good parameter naming
nav2_util::declare_parameter_if_not_declared(
  node_, planner_name_ + ".max_planning_time", rclcpp::ParameterValue(5.0));
nav2_util::declare_parameter_if_not_declared(
  node_, planner_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.2));
```

#### Error Handling Best Practices

```cpp
// Comprehensive error checking
if (!costmap_->worldToMap(x, y, mx, my)) {
  RCLCPP_WARN(logger_, "Position (%.2f, %.2f) outside costmap bounds", x, y);
  return false;
}

if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
  RCLCPP_DEBUG(logger_, "Position (%.2f, %.2f) is in lethal obstacle", x, y);
  return false;
}
```

#### Performance Optimization

1. **Memory Management**: Use smart pointers, avoid memory leaks
2. **Computational Efficiency**: Profile critical paths, optimize algorithms
3. **Early Termination**: Check for timeout conditions
4. **Caching**: Store frequently computed values

### Integration and Testing

#### Local Testing

```bash
# Build package
colcon build --packages-select my_custom_planner

# Run unit tests
colcon test --packages-select my_custom_planner

# Source environment
source install/setup.bash

# Test with navigation stack
ros2 launch nav2_bringup tb3_simulation_launch.py
```

#### Navigation Configuration

Add your planner to navigation parameters:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["MyCustomPlanner"]
    MyCustomPlanner:
      plugin: "my_custom_planner/MyPlanner"
      algorithm_param1: 2.0
      algorithm_param2: true
```

### Common Pitfalls and Solutions

1. **Frame ID Mismatches**: Always validate coordinate frames
2. **Memory Leaks**: Use RAII and smart pointers
3. **Thread Safety**: Be aware of concurrent access to costmap
4. **Parameter Updates**: Handle dynamic parameter changes
5. **Infinite Loops**: Implement timeout mechanisms

### Performance Benchmarking

Use the testing framework to evaluate performance:

```cpp
// Timing measurement example
auto start_time = std::chrono::high_resolution_clock::now();
auto path = planner->createPlan(start_pose, goal_pose);
auto end_time = std::chrono::high_resolution_clock::now();

auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
  end_time - start_time).count();
RCLCPP_INFO(logger_, "Planning took %ld ms", duration);
```

## Dependency Injection Pattern

Use dependency injection for:
- Planner instance management
- Test framework decoupling
- Configuration management
- External service integration

**Example Implementation:**
```cpp
class PlannerManager 
{
public:
  PlannerManager(std::shared_ptr<PlannerBase> planner) 
    : planner_(planner) {}
    
  nav_msgs::Path planPath(const geometry_msgs::PoseStamped& start,
                         const geometry_msgs::PoseStamped& goal) 
  {
    return planner_->compute_path(start, goal, costmap_);
  }

private:
  std::shared_ptr<PlannerBase> planner_;
};
```

## Unit Testing Standards

### Test Requirements
- **Framework:** Google Test (gtest) for C++
- **Coverage:** All new features must include tests
- **Scope:** Core logic + edge cases + error conditions

### Test Structure
```cpp
#include <gtest/gtest.h>
#include "my_planner.hpp"

class MyPlannerTest : public ::testing::Test 
{
protected:
  void SetUp() override 
  {
    planner_ = std::make_shared<MyPlanner>();
  }
  
  std::shared_ptr<MyPlanner> planner_;
};

TEST_F(MyPlannerTest, ComputePathValidInput) 
{
  // Test with valid start and goal
  auto path = planner_->compute_path(start_pose, goal_pose, costmap);
  EXPECT_FALSE(path.poses.empty());
  EXPECT_GT(path.poses.size(), 1);
}

TEST_F(MyPlannerTest, ComputePathInvalidInput) 
{
  // Test error handling
  auto path = planner_->compute_path(invalid_pose, goal_pose, costmap);
  EXPECT_TRUE(path.poses.empty());
}
```

### Running Tests
```bash
# Build with tests
colcon build

# Run all tests
colcon test

# Run specific test package
colcon test --packages-select my_planner_tests

# Verbose test output
colcon test --event-handlers console_direct+
```

## Documentation Standards

### API Documentation
- Document all public functions
- Include parameter descriptions
- Specify return values
- Provide usage examples
- Document error conditions

### Code Comments
- Complex algorithms require detailed comments
- Public APIs need comprehensive documentation
- Include references for mathematical formulations
- Explain non-obvious design decisions

**Example Documentation:**
```cpp
/**
 * @brief Computes optimal path using A* algorithm
 * 
 * This function implements the A* search algorithm with configurable
 * heuristics for path planning in 2D grid environments.
 * 
 * @param start Starting pose with position and orientation
 * @param goal Target pose with position and orientation  
 * @param costmap 2D costmap containing obstacle information
 * @return nav_msgs::Path Sequence of poses from start to goal
 * 
 * @throws std::invalid_argument If start or goal is in obstacle
 * @throws std::runtime_error If no valid path exists
 * 
 * @note Algorithm complexity: O(b^d) where b is branching factor
 *       and d is solution depth
 */
nav_msgs::Path compute_path(const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal,
                           nav2_costmap_2d::Costmap2D* costmap);
```

## Git Workflow

### Branch Management
- Create feature branches from main: `git checkout -b feature/my-feature`
- Keep branches focused on single features
- Delete branches after merge

### Commit Guidelines
- Use descriptive commit messages with prefixes
- Make atomic commits (one logical change per commit)
- Reference issues: `[FEATURE] Add RRT planner - fixes #123`

### Pre-commit Checklist
- [ ] Code formatted with clang-format/black
- [ ] All tests pass locally
- [ ] Documentation updated
- [ ] No compiler warnings
- [ ] Dependency injection used appropriately

## Build System

### ROS2 Package Structure
```xml
<!-- package.xml -->
<package format="3">
  <name>my_planner</name>
  <version>1.0.0</version>
  <description>Custom path planner implementation</description>
  <maintainer email="WLQVincent@gmail.com">scnscnscn</maintainer>
  <license>Apache-2.0</license>
  
  <depend>rclcpp</depend>
  <depend>nav2_core</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  
  <test_depend>gtest</test_depend>
</package>
```

### CMake Configuration
- Follow ROS2 CMake patterns
- Include proper dependencies
- Configure test targets
- Install targets correctly

## Performance Considerations

### Algorithm Testing
- Implement timing measurements
- Compare against baseline algorithms
- Document performance characteristics
- Test with various map sizes and complexities

### Memory Management
- Use smart pointers for automatic memory management
- Avoid memory leaks in long-running processes
- Profile memory usage for large maps

## Contact

For questions about development practices:
- **Maintainer:** scnscnscn
- **Email:** WLQVincent@gmail.com
- **Issues:** Use GitHub issue tracker for technical discussions