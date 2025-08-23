# Using the Custom A* Global Planner

This tutorial explains how to use the custom A* global planner that has been integrated into the Path Planning Test Platform.

## Overview

The custom A* planner (`nav2_astar_planner`) provides an efficient pathfinding algorithm optimized for TurtleBot3 navigation. It offers several advantages over the default Nav2 planner:

- Configurable heuristic weighting
- Optimized for grid-based navigation
- Built-in path smoothing
- Flexible obstacle handling

## Configuration

The A* planner is already configured as the default global planner in all TurtleBot3 parameter files. The configuration can be found in:

- `src/turtlebot3_navigation2/param/humble/burger.yaml`
- `src/turtlebot3_navigation2/param/humble/waffle.yaml`
- `src/turtlebot3_navigation2/param/humble/waffle_pi.yaml`
- `src/turtlebot3_navigation2/param/humble/burger_cam.yaml`

### Parameter Explanation

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_astar_planner/AStarPlanner"
      tolerance: 0.5              # Goal tolerance in meters
      heuristic_weight: 1.0       # A* heuristic weight
      allow_unknown: true         # Allow planning through unknown areas
      cost_threshold: 250         # Maximum traversable cost (0-255)
```

### Parameter Tuning Guide

#### `heuristic_weight`
- **1.0** - Standard A* algorithm (optimal paths)
- **> 1.0** - More greedy behavior (faster planning, potentially sub-optimal)
- **< 1.0** - More like Dijkstra (slower but very thorough)

#### `cost_threshold`
- **Lower values (100-200)** - More conservative, avoids areas near obstacles
- **Higher values (200-254)** - More aggressive, allows closer proximity to obstacles
- **255** - Allows planning through any non-lethal area

#### `allow_unknown`
- **true** - Planner can navigate through unmapped areas
- **false** - Planner treats unknown areas as obstacles

## Usage

### 1. Launch Navigation

```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Launch navigation stack
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

### 2. Set Initial Pose

Use RViz2 to set the robot's initial pose using the "2D Pose Estimate" tool.

### 3. Send Navigation Goals

Use RViz2's "2D Nav Goal" tool to send navigation goals. The A* planner will automatically:

1. Generate an optimal path from current position to goal
2. Apply path smoothing to remove unnecessary waypoints
3. Handle dynamic obstacle avoidance through costmap updates

### 4. Monitor Performance

You can monitor the planner's performance through ROS2 logs:

```bash
# View planner logs
ros2 topic echo /planner_server/transition_event

# Monitor planning time
ros2 run rqt_plot rqt_plot /planner_server/planning_time
```

## Advanced Usage

### Dynamic Parameter Reconfiguration

You can modify planner parameters at runtime:

```bash
# Change heuristic weight for more greedy behavior
ros2 param set /planner_server GridBased.heuristic_weight 1.5

# Adjust cost threshold for more conservative planning
ros2 param set /planner_server GridBased.cost_threshold 200

# Toggle unknown area handling
ros2 param set /planner_server GridBased.allow_unknown false
```

### Performance Comparison

To compare the A* planner with other planners, you can temporarily switch back to the default:

```bash
# Switch to NavFn planner
ros2 param set /planner_server GridBased.plugin "nav2_navfn_planner/NavfnPlanner"

# Switch back to A* planner
ros2 param set /planner_server GridBased.plugin "nav2_astar_planner/AStarPlanner"
```

## Troubleshooting

### Common Issues

1. **No path found**
   - Check if goal is reachable
   - Lower `cost_threshold` if path goes through high-cost areas
   - Enable `allow_unknown` if path crosses unmapped regions

2. **Slow planning**
   - Increase `heuristic_weight` for faster (but potentially sub-optimal) planning
   - Check costmap resolution and size

3. **Jerky paths**
   - The path smoothing algorithm should handle most cases
   - Consider adjusting local planner parameters for smoother following

### Debug Information

The A* planner provides detailed logging:

```bash
# Enable debug logging
ros2 param set /planner_server use_sim_time True
ros2 run rqt_console rqt_console
```

Look for messages from `AStarPlanner` node for planning statistics and warnings.

## Performance Characteristics

- **Planning Time**: Typically 1-50ms for maps up to 1000x1000 cells
- **Memory Usage**: O(map_size) for open/closed sets
- **Path Quality**: Optimal when `heuristic_weight = 1.0`
- **Robustness**: Handles dynamic obstacles through costmap updates

## Integration with Testing Framework

The A* planner is designed to work with the platform's testing framework:

```cpp
// Example test usage
auto planner = std::make_shared<nav2_astar_planner::AStarPlanner>();
auto path = planner->createPlan(start_pose, goal_pose);
// Analyze path quality and performance
```

For detailed API usage, see the [A* Planner API Documentation](../api/astar_planner.md).

---

**Next Steps**: 
- Try different parameter configurations for your specific use case
- Monitor performance metrics during navigation
- Compare with other available planners using the testing framework