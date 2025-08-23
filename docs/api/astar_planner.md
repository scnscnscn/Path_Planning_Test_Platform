# A* Global Planner API Reference

## Overview

The `nav2_astar_planner` package provides a custom A* global path planning algorithm for ROS2 navigation. This implementation is optimized for TurtleBot3 platforms and provides efficient pathfinding with configurable parameters.

## Class: AStarPlanner

### Inheritance
```cpp
class AStarPlanner : public nav2_core::GlobalPlanner
```

### Public Methods

#### configure()
```cpp
void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
```

**Purpose:** Initialize and configure the A* planner with ROS2 parameters.

**Parameters:**
- `parent`: Weak pointer to the lifecycle node
- `name`: Name of the planner instance
- `tf`: TF buffer for coordinate transformations
- `costmap_ros`: ROS wrapper for the costmap

**Configurable Parameters:**
- `heuristic_weight`: Weight for the heuristic function (default: 1.0)
- `allow_unknown`: Whether to allow planning through unknown areas (default: false)
- `cost_threshold`: Maximum cost value for traversable cells (default: 250)

#### createPlan()
```cpp
nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) override;
```

**Purpose:** Generate an optimal path from start to goal using A* algorithm.

**Parameters:**
- `start`: Starting pose in global frame
- `goal`: Target pose in global frame

**Returns:** `nav_msgs::msg::Path` containing the planned route

**Algorithm Features:**
- Uses Euclidean distance heuristic
- Supports 8-directional movement
- Includes path smoothing to remove collinear points
- Collision checking with configurable cost threshold

#### cleanup()
```cpp
void cleanup() override;
```

**Purpose:** Clean up resources when the planner is shut down.

#### activate()
```cpp
void activate() override;
```

**Purpose:** Activate the planner for use.

#### deactivate()
```cpp
void deactivate() override;
```

**Purpose:** Deactivate the planner.

### Private Methods

#### calculateHeuristic()
```cpp
double calculateHeuristic(int x1, int y1, int x2, int y2) const;
```

**Purpose:** Calculate the heuristic cost between two grid points using Euclidean distance.

**Parameters:**
- `x1, y1`: Starting grid coordinates
- `x2, y2`: Target grid coordinates

**Returns:** Weighted Euclidean distance in map units

#### isNodeValid()
```cpp
bool isNodeValid(int x, int y, const std::unordered_set<std::pair<int, int>, CoordHash>& closed_set) const;
```

**Purpose:** Validate if a grid cell is traversable.

**Parameters:**
- `x, y`: Grid coordinates to check
- `closed_set`: Set of already processed nodes

**Returns:** `true` if the node is valid for path planning

**Validation Criteria:**
- Within costmap boundaries
- Not already in closed set
- Cost below threshold
- Not a lethal obstacle
- Handles unknown areas based on `allow_unknown` parameter

#### getMoveCost()
```cpp
double getMoveCost(int x1, int y1, int x2, int y2) const;
```

**Purpose:** Calculate the movement cost between adjacent grid cells.

**Parameters:**
- `x1, y1`: Starting grid coordinates
- `x2, y2`: Target grid coordinates

**Returns:** Movement cost (resolution for straight, resolution * √2 for diagonal)

#### smoothPath()
```cpp
nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& raw_path) const;
```

**Purpose:** Remove unnecessary waypoints from the path by eliminating collinear points.

**Parameters:**
- `raw_path`: Original path from A* algorithm

**Returns:** Smoothed path with reduced waypoints

## Configuration Example

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_astar_planner/AStarPlanner"
      tolerance: 0.5
      heuristic_weight: 1.0      # Standard A* (1.0), > 1.0 more greedy, < 1.0 more like Dijkstra
      allow_unknown: true        # Allow planning through unknown areas
      cost_threshold: 250        # Maximum traversable cost (0-255)
```

## Performance Characteristics

- **Time Complexity:** O(b^d) where b is branching factor, d is depth
- **Space Complexity:** O(b^d) for storing open and closed sets
- **Optimality:** Guarantees shortest path when heuristic_weight ≤ 1.0
- **Admissible Heuristic:** Uses Euclidean distance which never overestimates

## Integration Notes

The planner integrates seamlessly with the Nav2 navigation stack and supports:
- Dynamic reconfiguration of parameters
- Real-time costmap updates
- Multiple robot footprint shapes
- Lifecycle management

## Maintainer

- **Name:** scnscnscn
- **Email:** WLQVincent@gmail.com
- **GitHub:** scnscnscn