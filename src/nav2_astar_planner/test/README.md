# A* 规划器 gtest 单元测试

## 概述

本目录包含 nav2_astar_planner 包的完整 gtest 单元测试套件。测试套件设计用于验证 A* 路径规划算法的功能正确性、边界条件处理和性能特性。

## 测试文件结构

```
test/
├── test_astar_planner.cpp          # 核心单元测试
├── test_astar_integration.cpp      # 集成测试
├── test_astar_planner_helper.hpp   # 测试辅助类
└── README.md                       # 本文档
```

## 测试覆盖范围

### 1. 核心数据结构测试 (`test_astar_planner.cpp`)

#### AStarNode 结构体测试
- **构造函数测试**: 验证节点正确初始化坐标、代价值和父节点指针
- **比较操作符测试**: 验证优先队列排序逻辑（f_cost 优先，h_cost 次之）
- **父子关系测试**: 验证节点链表构建的正确性

#### CoordHash 哈希函数测试  
- **哈希一致性**: 相同坐标生成相同哈希值
- **哈希分布**: 不同坐标生成不同哈希值（概率性验证）
- **容器兼容性**: 在 unordered_set/unordered_map 中的正确使用

### 2. 算法核心方法测试

#### 启发函数测试 (`testCalculateHeuristic`)
- **欧几里得距离**: 验证 3-4-5 直角三角形等标准距离计算
- **边界情况**: 相同点距离为0，轴向距离计算
- **权重影响**: 不同启发权重对距离计算的影响

#### 邻居获取测试 (`testGetNeighbors`)
- **8方向邻居**: 验证返回正确的8个相邻网格坐标
- **邻居完整性**: 确保没有遗漏或重复的邻居

#### 移动代价测试 (`testGetMoveCost`)
- **水平/垂直移动**: 验证基础移动代价 = 分辨率
- **对角线移动**: 验证对角线代价 = 分辨率 × √2
- **方向一致性**: 不同方向的相同类型移动有相同代价

#### 节点有效性测试 (`testIsNodeValid`)
- **边界检查**: 验证超出地图边界的点被正确拒绝
- **障碍物检测**: 验证致命障碍物被正确识别
- **已访问检查**: 验证 closed_set 中的节点被正确排除
- **代价阈值**: 验证高代价区域的通行判断
- **未知区域**: 验证 allow_unknown 参数的正确作用

#### 坐标转换测试
- **网格到世界**: 验证网格索引正确转换为世界坐标
- **世界到网格**: 验证世界坐标正确转换为网格索引
- **转换一致性**: 双向转换的一致性验证
- **边界处理**: 超出边界的坐标转换失败处理

#### 路径平滑测试 (`testSmoothPath`)
- **共线点移除**: 验证直线上的中间点被正确移除
- **拐点保留**: 验证关键转向点被正确保留
- **起终点保护**: 验证起点和终点始终保留
- **边界情况**: 空路径、单点路径、两点路径的处理

### 3. 集成测试 (`test_astar_integration.cpp`)

#### 复杂环境测试
- **L形障碍物**: 测试绕过复杂障碍物的路径规划
- **狭窄通道**: 测试通过狭窄通道的导航能力
- **高代价区域**: 测试在高代价但可通过区域的路径规划

#### 边界条件测试
- **地图边界**: 测试地图边缘附近的路径规划
- **不可达目标**: 测试被完全包围的目标点处理
- **起终点验证**: 测试起点和终点的有效性检查

#### 参数影响测试
- **启发权重**: 测试不同启发权重对算法行为的影响
- **代价阈值**: 测试不同代价阈值的过滤效果
- **未知区域**: 测试允许/禁止未知区域的影响

## 测试辅助工具

### AStarPlannerTestHelper 类
继承自 AStarPlanner 的测试友元类，提供对私有方法的测试访问：

```cpp
class AStarPlannerTestHelper : public AStarPlanner
{
public:
  // 测试专用的公共接口
  double testCalculateHeuristic(int x1, int y1, int x2, int y2) const;
  bool testIsNodeValid(int x, int y, const std::unordered_set<std::pair<int, int>, CoordHash>& closed_set) const;
  // ... 其他测试接口
};
```

### TestCostmap 类
轻量级的 Costmap2D 子类，专门用于单元测试：

```cpp
class TestCostmap : public nav2_costmap_2d::Costmap2D
{
public:
  void setObstacle(unsigned int x, unsigned int y);
  void setFree(unsigned int x, unsigned int y);
  void setUnknown(unsigned int x, unsigned int y);
  void createSimpleTestScene();  // 创建预定义的测试场景
};
```

## 构建和运行测试

### 构建测试
```bash
# 在工作空间根目录
colcon build --packages-select nav2_astar_planner
```

### 运行单元测试
```bash
# 运行所有测试
colcon test --packages-select nav2_astar_planner

# 运行特定测试
colcon test --packages-select nav2_astar_planner --ctest-args -R test_astar_planner

# 查看详细输出
colcon test --packages-select nav2_astar_planner --event-handlers console_direct+
```

### 查看测试结果
```bash
# 查看测试报告
colcon test-result --all

# 查看特定包的测试结果
colcon test-result --test-result-base build/nav2_astar_planner
```

## 测试数据和场景

### 标准测试场景
1. **开放环境**: 无障碍物的简单直线规划
2. **障碍物环境**: 包含各种形状障碍物的复杂环境
3. **狭窄通道**: 测试精确导航能力
4. **高代价区域**: 测试代价敏感的路径选择
5. **边界环境**: 测试地图边缘的特殊情况

### 测试地图配置
- **小型环境**: 10×10 网格，用于基础功能测试
- **中型环境**: 20×20 网格，用于算法逻辑测试  
- **大型环境**: 50×50 网格，用于集成和性能测试

## 测试最佳实践

### 1. 测试独立性
每个测试用例都是独立的，不依赖其他测试的状态或结果。

### 2. 明确的断言
使用具体的断言验证预期行为：
```cpp
EXPECT_EQ(actual_value, expected_value);
EXPECT_TRUE(condition);
EXPECT_DOUBLE_EQ(actual_double, expected_double);
EXPECT_NEAR(actual_value, expected_value, tolerance);
```

### 3. 边界条件覆盖
针对每个功能测试边界条件：
- 空输入、单元素输入
- 最大值、最小值
- 边界坐标、超出边界

### 4. 错误条件测试
验证错误输入的正确处理：
- 无效坐标
- 不可达目标
- 超出边界的输入

## 性能考虑

### 测试执行时间
- 单元测试设计为快速执行（< 1秒）
- 集成测试可能需要更长时间（1-5秒）
- 避免在测试中使用过大的数据集

### 内存使用
- 使用适当大小的测试数据
- 正确清理测试资源
- 避免内存泄漏

## 扩展测试

### 添加新测试用例
1. 在适当的测试类中添加新的 `TEST_F` 方法
2. 遵循现有的命名约定和测试结构
3. 包含充分的注释说明测试目的

### 添加新测试类
1. 继承 `::testing::Test` 或使用现有的测试基类
2. 在 `SetUp()` 方法中初始化测试环境
3. 在 `TearDown()` 方法中清理资源（如需要）

## 故障排除

### 常见测试失败原因
1. **编译错误**: 检查头文件包含和依赖项
2. **链接错误**: 确保测试目标正确链接到所需库
3. **运行时错误**: 检查测试数据的有效性
4. **断言失败**: 验证测试逻辑和预期值

### 调试技巧
1. 使用 `--gtest_filter` 运行特定测试
2. 添加调试输出查看中间结果
3. 使用调试器逐步执行测试代码
4. 检查测试环境设置

## 贡献指南

### 代码风格
- 遵循项目的 clang-format 配置
- 使用有意义的变量和函数名
- 添加充分的注释

### 测试质量
- 确保新测试有明确的目的
- 覆盖正常情况和边界条件
- 包含错误处理测试
- 验证测试的可重复性

### 文档更新
- 更新此 README 文档以反映新增的测试
- 添加必要的代码注释
- 更新测试覆盖范围说明

---

## 联系方式

- **维护者**: scnscnscn
- **邮箱**: WLQVincent@gmail.com
- **项目**: [Path_Planning_Test_Platform](https://github.com/scnscnscn/Path_Planning_Test_Platform)

如有问题或建议，请通过 GitHub Issues 或邮件联系。