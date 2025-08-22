# Testing Framework Documentation

This document describes the testing framework and guidelines for the Path Planning Test Platform.

## Overview

The testing framework provides comprehensive validation for path planning algorithms including:
- Unit tests for individual components
- Integration tests for complete workflows
- Performance benchmarking and comparison
- Automated testing pipelines

## Test Structure

### Directory Organization
```
test/
├── unit/                    # Unit tests for individual components
│   ├── planners/           # Planner-specific tests
│   ├── utils/              # Utility function tests
│   └── framework/          # Framework component tests
├── integration/            # Integration and system tests
├── benchmarks/             # Performance testing
├── data/                   # Test data and scenarios
│   ├── maps/              # Test maps
│   ├── scenarios/         # Test scenarios
│   └── expected/          # Expected results
└── CMakeLists.txt         # Test build configuration
```

## Unit Testing

### Framework: Google Test (gtest)

**Test Naming Convention:**
- File: `test_<component>_<function>.cpp`
- Class: `<Component>Test`
- Method: `<Function><Condition>`

**Example Test File Structure:**
```cpp
#include <gtest/gtest.h>
#include "my_planner/my_planner.hpp"

class MyPlannerTest : public ::testing::Test 
{
protected:
  void SetUp() override 
  {
    // Initialize test fixtures
    planner_ = std::make_shared<MyPlanner>();
    setupTestMap();
  }
  
  void TearDown() override 
  {
    // Cleanup after tests
    planner_.reset();
  }
  
  void setupTestMap() 
  {
    // Create test costmap with known obstacles
  }
  
  std::shared_ptr<MyPlanner> planner_;
  nav2_costmap_2d::Costmap2D test_costmap_;
};

// Test valid path computation
TEST_F(MyPlannerTest, ComputePathValidScenario) 
{
  geometry_msgs::PoseStamped start = createPose(1.0, 1.0, 0.0);
  geometry_msgs::PoseStamped goal = createPose(5.0, 5.0, 0.0);
  
  nav_msgs::Path path = planner_->compute_path(start, goal, &test_costmap_);
  
  EXPECT_FALSE(path.poses.empty());
  EXPECT_GE(path.poses.size(), 2);
  EXPECT_NEAR(path.poses.front().pose.position.x, 1.0, 0.1);
  EXPECT_NEAR(path.poses.back().pose.position.x, 5.0, 0.1);
}

// Test obstacle avoidance
TEST_F(MyPlannerTest, ComputePathWithObstacles) 
{
  // Add obstacles to test map
  addObstacle(2.5, 2.5, 1.0);  // Block direct path
  
  geometry_msgs::PoseStamped start = createPose(1.0, 1.0, 0.0);
  geometry_msgs::PoseStamped goal = createPose(4.0, 4.0, 0.0);
  
  nav_msgs::Path path = planner_->compute_path(start, goal, &test_costmap_);
  
  EXPECT_FALSE(path.poses.empty());
  // Verify path avoids obstacles
  for (const auto& pose : path.poses) 
  {
    EXPECT_FALSE(isInObstacle(pose.pose.position));
  }
}

// Test error handling
TEST_F(MyPlannerTest, ComputePathImpossibleScenario) 
{
  // Completely surround goal with obstacles
  surroundWithObstacles(5.0, 5.0, 2.0);
  
  geometry_msgs::PoseStamped start = createPose(1.0, 1.0, 0.0);
  geometry_msgs::PoseStamped goal = createPose(5.0, 5.0, 0.0);
  
  nav_msgs::Path path = planner_->compute_path(start, goal, &test_costmap_);
  
  EXPECT_TRUE(path.poses.empty());  // Should return empty path
}
```

### Test Coverage Requirements

**Mandatory Test Categories:**
1. **Happy Path Tests** - Normal operation with valid inputs
2. **Edge Case Tests** - Boundary conditions and corner cases  
3. **Error Handling Tests** - Invalid inputs and failure scenarios
4. **Performance Tests** - Timing and resource usage validation

**Coverage Targets:**
- Core logic functions: 100%
- Public API methods: 100%
- Error handling paths: 90%
- Edge cases: 80%

## Performance Testing

### Timing Framework

**Benchmark Structure:**
```cpp
#include <benchmark/benchmark.h>
#include "my_planner/my_planner.hpp"

class PlannerBenchmark 
{
public:
  static void BM_PlannerPerformance(benchmark::State& state) 
  {
    auto planner = std::make_shared<MyPlanner>();
    setupLargeMap();
    
    for (auto _ : state) 
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      
      nav_msgs::Path path = planner->compute_path(start_pose, goal_pose, &large_costmap);
      
      auto end_time = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      
      state.SetIterationTime(elapsed.count() / 1000000.0);  // Convert to seconds
    }
    
    state.SetItemsProcessed(state.iterations());
  }
};

BENCHMARK(PlannerBenchmark::BM_PlannerPerformance)
  ->Unit(benchmark::kMillisecond)
  ->Iterations(100)
  ->UseManualTime();
```

### Performance Metrics

**Key Measurements:**
- **Planning Time** - Algorithm execution time
- **Path Quality** - Path length, smoothness, safety margin
- **Memory Usage** - Peak memory consumption
- **Success Rate** - Percentage of successful planning attempts

**Comparison Framework:**
```cpp
struct PerformanceResults 
{
  double mean_time_ms;
  double std_dev_ms;
  double min_time_ms;
  double max_time_ms;
  double path_length;
  double path_smoothness;
  int success_count;
  int total_attempts;
};

class AlgorithmComparison 
{
public:
  PerformanceResults benchmarkPlanner(std::shared_ptr<PlannerBase> planner,
                                     const std::vector<TestScenario>& scenarios);
  
  void compareAlgorithms(const std::vector<std::shared_ptr<PlannerBase>>& planners);
  
  void generateReport(const std::string& output_file);
};
```

## Integration Testing

### Test Scenarios

**Scenario Categories:**
1. **Simple Environments** - Open spaces with minimal obstacles
2. **Complex Environments** - Dense obstacle fields, narrow passages
3. **Dynamic Scenarios** - Moving obstacles, changing goals
4. **Stress Tests** - Large maps, time constraints, resource limits

**Test Data Organization:**
```
test/data/
├── maps/
│   ├── simple_room.yaml
│   ├── office_environment.yaml
│   ├── warehouse.yaml
│   └── maze_complex.yaml
├── scenarios/
│   ├── basic_navigation.json
│   ├── obstacle_avoidance.json
│   └── stress_test.json
└── expected/
    ├── reference_paths/
    └── timing_baselines/
```

### Automated Testing

**Test Pipeline:**
1. Build all planner packages
2. Run unit tests for each component
3. Execute integration tests with standard scenarios
4. Perform performance benchmarks
5. Generate comparison reports
6. Validate against baseline metrics

**CI/CD Integration:**
```yaml
# Example GitHub Actions workflow
name: Path Planner Tests
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup ROS2
        uses: ros-tooling/setup-ros@v0.6
      - name: Build packages
        run: colcon build
      - name: Run unit tests
        run: colcon test --packages-select-regex ".*_test"
      - name: Run integration tests
        run: colcon test --packages-select integration_tests
      - name: Performance benchmarks
        run: ./scripts/run_benchmarks.sh
      - name: Upload results
        uses: actions/upload-artifact@v2
        with:
          name: test-results
          path: test_results/
```

## Test Data Management

### Test Maps
- **Format:** ROS2 occupancy grid maps (.yaml + .pgm)
- **Variety:** Different complexity levels and obstacle patterns
- **Validation:** Known optimal paths for verification

### Scenario Definitions
```json
{
  "scenario_name": "narrow_corridor",
  "description": "Test navigation through narrow passages",
  "map_file": "maps/corridor_map.yaml",
  "test_cases": [
    {
      "start": {"x": 1.0, "y": 1.0, "theta": 0.0},
      "goal": {"x": 10.0, "y": 1.0, "theta": 0.0},
      "expected_success": true,
      "max_planning_time": 5.0,
      "reference_path_length": 9.5
    }
  ]
}
```

## Running Tests

### Command Line Interface
```bash
# Run all tests
colcon test

# Run specific test package
colcon test --packages-select my_planner_tests

# Run with verbose output
colcon test --event-handlers console_direct+

# Run performance benchmarks
./scripts/benchmark_planners.sh

# Generate test report
./scripts/generate_test_report.sh
```

### Test Configuration
```cmake
# CMakeLists.txt for test package
find_package(gtest REQUIRED)
find_package(benchmark REQUIRED)

# Unit test executable
add_executable(test_my_planner
  test/unit/test_my_planner.cpp
  test/unit/test_utils.cpp
)

target_link_libraries(test_my_planner
  ${gtest_LIBRARIES}
  my_planner_lib
)

# Performance benchmark
add_executable(benchmark_my_planner
  test/benchmarks/benchmark_my_planner.cpp
)

target_link_libraries(benchmark_my_planner
  benchmark::benchmark
  my_planner_lib
)

# Register tests with CTest
add_test(NAME my_planner_unit_tests COMMAND test_my_planner)
add_test(NAME my_planner_benchmarks COMMAND benchmark_my_planner)
```

## Best Practices

### Test Design
- Write tests before implementing features (TDD)
- Keep tests independent and isolated
- Use descriptive test names and documentation
- Mock external dependencies when possible
- Include both positive and negative test cases

### Maintenance
- Update tests when APIs change
- Review test coverage regularly
- Remove obsolete tests
- Keep test data current and relevant
- Document test requirements and setup procedures

## Contact

For testing framework questions:
- **Maintainer:** scnscnscn
- **Email:** WLQVincent@gmail.com
- **Issues:** Report testing issues on GitHub tracker