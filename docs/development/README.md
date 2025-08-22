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