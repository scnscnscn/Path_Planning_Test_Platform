# nav2_astar_planner

## 概述

nav2_astar_planner 是一个基于 ROS2 的 A* 算法全局路径规划器插件，专为 Navigation2 框架设计。该规划器实现了经典的 A* 搜索算法，可在二维栅格地图中计算从起始点到目标点的最优路径，支持避障和代价地图集成。

## A* 算法原理

### 算法基础

A* (A-star) 是一种图搜索算法，结合了 Dijkstra 算法的精确性和贪心最佳优先搜索的效率。它通过评估函数 f(n) = g(n) + h(n) 来选择下一个要扩展的节点：

- **g(n)**: 从起始节点到当前节点 n 的实际代价
- **h(n)**: 从节点 n 到目标节点的启发式代价估计（必须是可容许的）
- **f(n)**: 总估计代价，表示通过节点 n 到达目标的预期总代价

### 算法特性

1. **完备性**: 如果存在解，A* 保证能找到解
2. **最优性**: 当启发函数可容许时（h(n) ≤ 实际代价），A* 保证找到最优解
3. **效率**: 相比 Dijkstra 算法，A* 通过启发函数引导搜索方向，减少了搜索空间

### 启发函数

本实现使用**欧几里得距离**作为启发函数：

```
h(n) = heuristic_weight × √((x_goal - x_current)² + (y_goal - y_current)²) × resolution
```

其中 `heuristic_weight` 是可配置的权重参数，用于平衡算法的最优性和计算速度。

### 搜索策略

1. **8方向搜索**: 支持水平、垂直和对角线移动
2. **代价计算**: 
   - 水平/垂直移动: `resolution`
   - 对角线移动: `resolution × √2`
3. **优先级排序**: 使用最小堆维护开放集合，优先扩展 f 值最小的节点

## 代码架构

### 核心组件

#### 1. AStarNode 结构体
```cpp
struct AStarNode {
    int x, y;                           // 网格坐标
    double g_cost;                      // 起始点到当前点的实际代价
    double h_cost;                      // 启发式代价
    double f_cost;                      // 总代价 = g_cost + h_cost
    std::shared_ptr<AStarNode> parent;  // 父节点指针
};
```

**功能**: 表示搜索图中的一个节点，包含位置信息、代价计算和父节点链接。

#### 2. CoordHash 结构体
```cpp
struct CoordHash {
    std::size_t operator()(const std::pair<int, int>& coord) const;
};
```

**功能**: 为坐标对提供哈希函数，用于高效的集合和映射操作。

#### 3. AStarPlanner 类
主要的规划器类，继承自 `nav2_core::GlobalPlanner`。

### 关键方法

#### 生命周期管理
- `configure()`: 初始化规划器参数和资源
- `activate()`: 激活规划器
- `deactivate()`: 停用规划器  
- `cleanup()`: 清理资源

#### 核心规划函数
```cpp
nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal);
```

**算法流程**:
1. 坐标转换（世界坐标 → 网格坐标）
2. 起点和终点有效性检查
3. 初始化开放集合、封闭集合和代价映射
4. A* 主循环搜索
5. 路径重构和坐标转换（网格坐标 → 世界坐标）
6. 路径平滑处理（移除共线点优化路径）

#### 辅助功能函数

- `calculateHeuristic()`: 计算启发式代价（欧几里得距离）
- `isNodeValid()`: 检查节点是否可通行（边界、障碍物、代价阈值）
- `getNeighbors()`: 获取当前节点的8个邻居
- `getMoveCost()`: 计算移动代价（考虑对角线移动）
- `worldToGrid()` / `gridToWorld()`: 坐标系转换
- `smoothPath()`: 路径平滑处理，移除共线点优化路径

### 数据结构设计

1. **开放集合**: `std::priority_queue` 实现最小堆，优先处理 f 值最小的节点
2. **封闭集合**: `std::unordered_set` 快速查找已访问的节点
3. **代价映射**: `std::unordered_map` 存储每个节点的最佳 g 值

### 类依赖关系

```
AStarPlanner
├── nav2_core::GlobalPlanner (基类)
├── nav2_costmap_2d::Costmap2D (代价地图)
├── tf2_ros::Buffer (坐标变换)
└── rclcpp_lifecycle::LifecycleNode (ROS2 节点)
```

### 路径平滑

算法包含路径平滑功能，通过移除共线点来优化最终路径：

```cpp
// 路径平滑算法
nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& raw_path) const;
```

**平滑策略**:
- 检测三个连续点的叉积
- 移除共线点（叉积接近零的点）
- 保留起点和终点
- 减少路径点数量，提高执行效率

**数学原理**:
使用叉积检测共线性：`cross = (c.x - p.x) * (n.y - c.y) - (c.y - p.y) * (n.x - c.x)`
当 |cross| < ε 时，认为三点共线并移除中间点。

## 配置参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `heuristic_weight` | double | 1.0 | 启发函数权重，影响搜索效率和最优性平衡 |
| `allow_unknown` | bool | false | 是否允许通过未知区域 |
| `cost_threshold` | int | 250 | 代价阈值，超过此值的区域视为不可通行 |

### 参数调优指导

- **heuristic_weight**:
  - = 1.0: 保证最优解（A*）
  - > 1.0: 加快搜索速度，可能牺牲最优性（Weighted A*）
  - < 1.0: 更保守的搜索，接近 Dijkstra 算法

- **allow_unknown**: 
  - `true`: 允许规划通过未探索区域
  - `false`: 只在已知安全区域规划路径

- **cost_threshold**: 根据代价地图的代价分布调整，平衡安全性和通行性

## 构建和使用

### 依赖项

- ROS2 Humble
- Navigation2 框架
- nav2_core
- nav2_costmap_2d
- geometry_msgs
- nav_msgs

### 构建

```bash
# 在工作空间根目录
colcon build --packages-select nav2_astar_planner

# 安装环境
source install/setup.bash
```

### 配置使用

在 Navigation2 配置文件中注册插件：

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_astar_planner/AStarPlanner"
      heuristic_weight: 1.0
      allow_unknown: false
      cost_threshold: 250
```

### 运行示例

```bash
# 启动 Navigation2 栈
ros2 launch nav2_bringup tb3_simulation_launch.py

# 或在自定义配置中使用
ros2 launch your_navigation_package navigation_launch.py
```

## 性能特性

### 时间复杂度
- **最坏情况**: O(b^d)，其中 b 是分支因子，d 是解的深度
- **平均情况**: 取决于启发函数质量和地图复杂度

### 空间复杂度
- O(b^d)，主要用于存储开放集合和封闭集合

### 性能优化
1. 使用高效的数据结构（优先队列、哈希集合）
2. 早期终止条件检查
3. 内存池优化（可选）
4. 路径平滑减少执行点数

## 与其他规划器对比

| 特性 | A* | Dijkstra | RRT | RRT* |
|------|----|---------|----|------|
| 最优性 | 保证（可容许启发函数） | 保证 | 不保证 | 渐近最优 |
| 完备性 | 保证 | 保证 | 概率完备 | 概率完备 |
| 计算效率 | 高 | 中 | 高 | 中 |
| 内存使用 | 高 | 高 | 低 | 中 |
| 动态环境适应性 | 差 | 差 | 好 | 好 |

## 已知限制

1. **静态环境**: 适用于静态或缓慢变化的环境
2. **内存消耗**: 在大型地图上可能消耗较多内存
3. **重规划**: 环境变化时需要完全重新规划
4. **分辨率依赖**: 性能与地图分辨率密切相关

## 故障排除

### 常见问题

1. **规划失败**:
   - 检查起点和终点是否在地图范围内
   - 确认路径不被障碍物完全阻挡
   - 调整 `cost_threshold` 参数

2. **性能问题**:
   - 增大 `heuristic_weight` 加快搜索
   - 降低地图分辨率
   - 检查代价地图更新频率

3. **路径质量**:
   - 调整启发函数权重
   - 检查代价地图质量
   - 考虑后处理平滑算法

## 开发扩展

### 自定义启发函数

可以通过修改 `calculateHeuristic()` 方法实现不同的启发函数：

```cpp
// 曼哈顿距离
double manhattan_heuristic = heuristic_weight_ * (std::abs(x2-x1) + std::abs(y2-y1)) * resolution;

// 切比雪夫距离  
double chebyshev_heuristic = heuristic_weight_ * std::max(std::abs(x2-x1), std::abs(y2-y1)) * resolution;
```

### 添加路径平滑

在路径重构后添加平滑算法：

```cpp
// 在 createPlan() 方法中路径生成后
path = smoothPath(path);
```

## 维护者信息

- **包维护者**: scnscnscn
- **联系邮箱**: WLQVincent@gmail.com
- **项目地址**: [Path_Planning_Test_Platform](https://github.com/scnscnscn/Path_Planning_Test_Platform)

## 许可证

Apache License 2.0

## 参考文献

1. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths. IEEE transactions on Systems Science and Cybernetics, 4(2), 100-107.
2. Russell, S., & Norvig, P. (2020). Artificial Intelligence: A Modern Approach (4th ed.). Pearson.
3. Navigation2 Documentation: [https://navigation.ros.org/](https://navigation.ros.org/)