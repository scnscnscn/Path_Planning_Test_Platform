# nav2\_astar\_planner 测试文档（修订版）

## 一、测试误差与边界问题总结

本项目在 nav2\_astar\_planner 单元测试与集成测试开发中，针对典型误差与边界问题形成以下解决方案，供后续开发维护参考：

### 1. 浮点误差导致的断言失败



*   **现象**：路径 / 坐标类测试中，实际世界坐标（world coordinate）与理论值存在 `0.01~0.05` 的微小偏差，导致 `EXPECT_DOUBLE_EQ` 断言触发失败。

*   **根本原因**：浮点数运算累积误差、地图分辨率与原点偏移叠加、数值舍入规则差异。

*   **解决方案**：

1.  替换断言方式：使用 `EXPECT_NEAR(actual, expected, tolerance)` 替代精确比较，推荐容差设置为 `5e-2` 或 `6e-2`（需略大于最大观测误差，避免误判）。

2.  注释规范：所有使用 `EXPECT_NEAR` 的断言需注明容差依据，例如 `// 容差6e-2：覆盖分辨率0.05+浮点误差0.01`。

### 2. 代价地图边界与障碍物设置问题

#### （1）典型案例：不可达目标点测试（UnreachableGoal 用例）



*   **问题现象**：期望目标点被 8 个障碍物包围时，`blocked_neighbors` 统计值应为 8，但实际仅为 2，断言失败。

*   **排查过程**：

1.  障碍物写入范围与目标点网格坐标（grid coordinate）不匹配：调试输出显示，目标点的 8 个邻居中仅 2 个标记为障碍（cost≠0），其余为自由空间（cost=0）。

2.  坐标映射逻辑偏差：`createPose(4.6, 4.6)` 实际映射为 grid `(45,45)`，而非预期的 `(46,46)`—— 因世界坐标到网格索引的转换默认采用**向下取整**规则（如分辨率 0.1 时，4.6÷0.1=46，向下取整为 45）。

*   **解决方案**：

1.  坐标对齐：明确目标点 grid 坐标（如 `(45,45)`），障碍物写入范围扩展至 `(44,44)-(46,46)`（仅排除中心点，确保 8 个邻居均被覆盖）。

2.  调试辅助：打印邻居坐标、cost 值、有效性（valid）状态，快速定位未覆盖区域。

*   **验证结果**：修正后 `blocked_neighbors=8`，测试用例通过。

#### （2）共性问题与解决方案



*   **其他现象**：目标点被误判为障碍物、边界障碍物设置未生效。

*   **原因**：

1.  代价地图（costmap）边界点设置障碍时，超出地图范围的点未被过滤，导致设置失效。

2.  障碍物与自由空间设置顺序倒置，目标点被后续障碍物覆盖。

*   **解决方案**：

1.  显式保障目标点状态：障碍物设置完成后，必须调用 `setFree(goal_x, goal_y)`，强制确保目标点为自由空间。

2.  边界处理优化：统计 `blocked_neighbors` 时，将 “超出地图边界的邻居” 也判定为 “被阻挡”，符合实际导航逻辑。

### 3. 未知区域与代价阈值冲突



*   **现象**：开启 “允许未知区域通行”（allow\_unknown=true）后，NO\_INFORMATION 区域仍被判定为不可通行。

*   **原因**：`isNodeValid` 函数逻辑顺序错误 —— 先判断 `cost > cost_threshold_`，再判断是否为 NO\_INFORMATION。若 `cost_threshold_ < 255`（NO\_INFORMATION 对应的 cost 值），即使允许未知区域，该区域仍会因 cost 超限被拒绝。

*   **解决方案**：

    优化 `isNodeValid` 逻辑优先级：



```
if (cost == NO\_INFORMATION) {

&#x20; return allow\_unknown\_;  // 优先判断未知区域，允许则直接返回true

}

return (cost <= cost\_threshold\_);  // 再判断代价阈值
```

### 4. 测试用例与 costmap 状态同步



*   **现象**：多次运行测试、或障碍物 / 自由空间设置顺序混乱，导致目标点状态异常（如时而为自由、时而为障碍）。

*   **解决方案**：

1.  状态重置：每个测试用例执行前，调用 `costmap->resetMap()` 清空历史状态，确保环境一致性。

2.  即时断言：关键操作（如设置障碍物、标记自由空间）后，立即断言目标点的 cost 值与可通行性，例如：



```
EXPECT\_EQ(costmap->getCost(goal\_x, goal\_y), FREE\_SPACE);

EXPECT\_TRUE(planner->isNodeValid(goal\_x, goal\_y, closed\_set));
```

## 二、A\* 规划器 gtest 单元测试

### 1. 概述

本目录提供 nav2\_astar\_planner 包的完整 gtest 测试套件，覆盖**功能正确性验证**、**边界条件处理**、**核心算法逻辑**三大维度，确保 A\* 路径规划算法的可靠性。

### 2. 测试文件结构



```
test/

├── test\_astar\_planner.cpp          # 核心单元测试（数据结构、算法方法）

├── test\_astar\_integration.cpp      # 集成测试（复杂场景、参数影响）

├── test\_astar\_planner\_helper.hpp   # 测试辅助类（友元访问、轻量costmap）

└── README.md                       # 测试文档（本文档）
```

### 3. 测试覆盖范围

#### 3.1 核心数据结构测试（test\_astar\_planner.cpp）

##### （1）AStarNode 结构体测试



*   **构造函数验证**：检查节点坐标（x/y）、代价值（g\_cost/f\_cost/h\_cost）、父节点指针初始化的正确性。

*   **比较操作符测试**：验证优先队列排序逻辑 —— 先比较 f\_cost（总代价），f\_cost 相等时比较 h\_cost（启发代价）。

*   **父子关系验证**：构建节点链表，断言子节点的父指针指向正确，且路径回溯时能还原完整路径。

##### （2）CoordHash 哈希函数测试



*   **哈希一致性**：相同坐标（x1,y1）多次哈希生成相同值。

*   **哈希分布性**：不同坐标生成不同哈希值（抽样 100 组坐标，无碰撞），确保容器（unordered\_set/map）高效查找。

*   **容器兼容性**：验证坐标作为 unordered\_set 元素、unordered\_map 键时的插入 / 查找 / 删除功能正常。

#### 3.2 算法核心方法测试

##### （1）启发函数测试（testCalculateHeuristic）



*   **距离类型验证**：


    *   欧几里得距离：验证直角三角形（3,4,5）、轴向移动（如 x 差 5,y 差 0）的计算结果。

    *   权重影响：测试不同启发权重（heuristic\_weight）对 h\_cost 的影响（如权重 1.5 时，h\_cost=1.5× 基础距离）。

*   **边界情况**：起点与终点重合时，h\_cost=0。

##### （2）邻居获取测试（testGetNeighbors）



*   **完整性验证**：非边界节点返回 8 个邻居（上下左右 + 四个对角线），无遗漏 / 重复。

*   **边界适配**：地图边缘节点（如 x=0 或 y=map\_height-1）仅返回有效范围内的邻居（如左上角节点返回 3 个邻居）。

##### （3）移动代价测试（testGetMoveCost）



*   **基础代价**：水平 / 垂直移动代价 = 地图分辨率（如分辨率 0.1 时，代价 = 0.1）。

*   **对角线代价**：对角线移动代价 = 分辨率 × √2（如分辨率 0.1 时，代价≈0.1414）。

*   **方向一致性**：相同类型移动（如左移、右移）的代价一致，不受方向影响。

##### （4）节点有效性测试（testIsNodeValid）

覆盖 5 类关键场景：



1.  边界检查：超出地图范围的坐标（x<0 或 x≥width）返回 false。

2.  障碍物检测：cost=LETHAL\_OBSTACLE（致命障碍）的节点返回 false。

3.  已访问检查：closed\_set 中已存在的节点返回 false。

4.  代价阈值：cost>cost\_threshold\_ 的节点返回 false（如阈值 100 时，cost=150 的节点被拒绝）。

5.  未知区域：allow\_unknown=true 时，NO\_INFORMATION 节点返回 true；反之返回 false。

##### （5）坐标转换测试



*   **双向一致性**：网格坐标→世界坐标→网格坐标，结果与原始坐标一致（误差≤0.5× 分辨率）。

*   **边界处理**：超出地图范围的世界坐标，转换为网格坐标时返回无效标记（如 - 1）。

##### （6）路径平滑测试（testSmoothPath）



*   **共线点移除**：三点共线（如 (0,0)→(1,1)→(2,2)）时，中间点 (1,1) 被移除。

*   **拐点保留**：非共线路径（如 (0,0)→(1,0)→(1,1)）的拐点 (1,0) 被保留。

*   **边界保护**：空路径、单点路径、两点路径的处理逻辑无异常（不崩溃、不篡改起终点）。

#### 3.3 集成测试（test\_astar\_integration.cpp）

##### （1）复杂环境测试



*   **L 形障碍物场景**：验证算法能绕过凹形障碍物，生成最短路径（非绕远路）。

*   **狭窄通道场景**：通道宽度 = 1 个网格单元时，算法能找到唯一可行路径。

*   **高代价区域场景**：路径优先规避高代价区域（如 cost=200），仅在无其他路径时才穿过。

##### （2）边界条件测试



*   **地图边界导航**：起点 / 终点位于地图边缘（如 x=0,y=5）时，路径规划正常。

*   **不可达目标处理**：目标点被完全包围（如 8 个邻居均为障碍）时，算法返回 “无路径”，无死循环。

*   **起终点有效性**：起点 / 终点为障碍时，算法立即返回无效，不执行冗余计算。

##### （3）参数影响测试



*   **启发权重**：权重 = 1.0 时为标准 A\*；权重 > 1.0（如 2.0）时，路径更接近直线（启发占比更高）；权重 < 1.0（如 0.5）时，搜索范围扩大（更接近 Dijkstra 算法）。

*   **代价阈值**：阈值从 50 提升至 200 时，原本被拒绝的高代价区域（如 cost=150）变为可通行。

*   **未知区域开关**：allow\_unknown=true 时，路径可穿过未知区域；反之则必须绕开。

### 4. 测试辅助工具

#### 4.1 AStarPlannerTestHelper 类

继承自 AStarPlanner，通过友元权限暴露私有方法，支持单元测试直接调用：



```
class AStarPlannerTestHelper : public AStarPlanner

{

public:

&#x20; // 测试启发函数

&#x20; double testCalculateHeuristic(int x1, int y1, int x2, int y2) const&#x20;

&#x20; { return calculateHeuristic(x1, y1, x2, y2); }

&#x20;&#x20;

&#x20; // 测试节点有效性

&#x20; bool testIsNodeValid(int x, int y, const std::unordered\_set\<std::pair\<int, int>, CoordHash>& closed\_set) const&#x20;

&#x20; { return isNodeValid(x, y, closed\_set); }

&#x20;&#x20;

&#x20; // 其他私有方法的测试接口...

};
```

#### 4.2 TestCostmap 类

轻量级 Costmap2D 子类，简化测试环境搭建：



```
class TestCostmap : public nav2\_costmap\_2d::Costmap2D

{

public:

&#x20; // 快速设置障碍

&#x20; void setObstacle(unsigned int x, unsigned int y)&#x20;

&#x20; { setCost(x, y, LETHAL\_OBSTACLE); }

&#x20;&#x20;

&#x20; // 快速设置自由空间

&#x20; void setFree(unsigned int x, unsigned int y)&#x20;

&#x20; { setCost(x, y, FREE\_SPACE); }

&#x20;&#x20;

&#x20; // 快速设置未知区域

&#x20; void setUnknown(unsigned int x, unsigned int y)&#x20;

&#x20; { setCost(x, y, NO\_INFORMATION); }

&#x20;&#x20;

&#x20; // 预定义测试场景（如L形障碍、狭窄通道）

&#x20; void createSimpleTestScene();

};
```

### 5. 构建与运行测试

#### 5.1 构建测试



```
\# 在ROS工作空间根目录执行

colcon build --packages-select nav2\_astar\_planner
```

#### 5.2 运行测试



```
\# 1. 运行所有测试

colcon test --packages-select nav2\_astar\_planner

\# 2. 运行指定测试（如核心单元测试）

colcon test --packages-select nav2\_astar\_planner --ctest-args -R "test\_astar\_planner"

\# 3. 查看详细输出（含调试信息）

colcon test --packages-select nav2\_astar\_planner --event-handlers console\_direct+
```

#### 5.3 查看测试结果



```
\# 查看所有测试结果摘要

colcon test-result --all

\# 查看指定包的详细结果（含失败日志路径）

colcon test-result --test-result-base build/nav2\_astar\_planner
```

### 6. 测试最佳实践

#### 6.1 测试独立性原则



*   每个测试用例（TEST/TEST\_F）需独立初始化环境，不依赖其他测试的执行结果。

*   禁止在测试间共享全局变量或 costmap 状态。

#### 6.2 断言设计规范



*   优先使用精准断言：如 `EXPECT_EQ` 替代 `EXPECT_TRUE`（例：`EXPECT_EQ(cost, 0)` 比 `EXPECT_TRUE(cost==0)` 更清晰）。

*   浮点比较必用 `EXPECT_NEAR`，并注明容差依据（见 1.1 节）。

*   断言失败时添加提示：如 `EXPECT_EQ(blocked_cnt, 8) << "目标点8个邻居未完全被障碍覆盖";`。

#### 6.3 边界覆盖要求

每个核心方法需覆盖以下边界场景：



*   空输入（如空路径、空 closed\_set）

*   极值输入（如 x=0、y=map\_max、cost=255）

*   临界状态（如 cost=cost\_threshold\_、起点 = 终点）

#### 6.4 性能优化



*   单元测试：单个用例执行时间控制在 1 秒内，避免大尺寸地图（推荐≤50×50 网格）。

*   集成测试：复杂场景测试时间控制在 5 秒内，如需性能测试可单独创建性能用例（如 `TEST_P(PerformanceTest, LargeMap)`）。

### 7. 故障排除

#### 7.1 常见失败原因


| 失败类型  | 排查方向                                                                              |
| ----- | --------------------------------------------------------------------------------- |
| 编译错误  | 1. 头文件包含是否完整（如 test\_astar\_planner\_helper.hpp）2. 依赖包是否正确声明（如 nav2\_costmap\_2d） |
| 链接错误  | 1. 测试目标是否链接 gtest 库2. 辅助类是否正确继承父类                                                 |
| 断言失败  | 1. 查看调试输出（如邻居 cost 值、坐标转换结果）2. 验证 costmap 状态是否符合预期                                |
| 运行时崩溃 | 1. 是否访问空指针（如未初始化的 costmap）2. 坐标是否越界（如 x=-1）                                       |


## 三、联系方式



*   **维护者**：scnscnscn

*   **邮箱**：WLQVincent@gmail.com

*   **项目仓库**：[Path\_Planning\_Test\_Platform](https://github.com/scnscnscn/Path_Planning_Test_Platform)

如有测试相关问题或优化建议，可通过 GitHub Issues 或邮件反馈。

