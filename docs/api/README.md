# API Documentation

This directory contains detailed API documentation for the Path Planning Test Platform.

## Path Planners API

### Base Planner Interface

All custom planners should inherit from the base planner interface and implement the following core methods:

#### `compute_path` Function

**Purpose:** Compute a path from start to goal position

**Parameters:**
- `start` (geometry_msgs::PoseStamped) - Starting position and orientation
- `goal` (geometry_msgs::PoseStamped) - Target position and orientation  
- `costmap` (nav2_costmap_2d::Costmap2D*) - Navigation costmap for obstacle information

**Returns:**
- `nav_msgs::Path` - Computed path as a sequence of poses
- Returns empty path if no valid path found

**Example Usage:**
```cpp
auto planner = std::make_shared<CustomPlanner>();
nav_msgs::Path path = planner->compute_path(start_pose, goal_pose, costmap);
```

### Error Handling

All planners should implement proper error handling:
- Return empty path for invalid inputs
- Log warnings for planning failures
- Throw exceptions for critical errors only

## Testing Framework API

### Performance Testing

#### `measure_planning_time` Function

**Purpose:** Measure algorithm execution time for performance comparison

**Parameters:**
- `planner` (std::shared_ptr<PlannerBase>) - Planner instance to test
- `test_scenario` (TestScenario) - Test case with start/goal positions
- `iterations` (int) - Number of test iterations for averaging

**Returns:**
- `TimingResults` - Statistical timing data (mean, std, min, max)

## Documentation Guidelines

When documenting new APIs:
1. Include function purpose and behavior
2. Document all parameters with types
3. Specify return values and possible states
4. Provide usage examples
5. Document error conditions and handling
6. Include performance considerations where relevant

**Maintainer:** WLQVincent@gmail.com