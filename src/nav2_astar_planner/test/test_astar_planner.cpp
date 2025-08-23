/*
 * Copyright (c) 2024 scnscnscn
 * Licensed under the Apache License, Version 2.0
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <utility>
#include <unordered_set>

#include "nav2_astar_planner/astar_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "test_astar_planner_helper.hpp"

namespace nav2_astar_planner
{

/**
 * @brief AStarNode结构体测试类
 * 测试A*节点的构造函数、成员变量和比较操作符
 */
class AStarNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 创建测试节点
    node1_ = std::make_shared<AStarNode>(5, 10, 15.0, 20.0);
    node2_ = std::make_shared<AStarNode>(3, 8, 12.0, 18.0);
    node3_ = std::make_shared<AStarNode>(7, 12, 25.0, 10.0);  // 相同f_cost的节点
  }

  std::shared_ptr<AStarNode> node1_;
  std::shared_ptr<AStarNode> node2_;
  std::shared_ptr<AStarNode> node3_;
};

/**
 * @brief 测试AStarNode构造函数
 */
TEST_F(AStarNodeTest, Constructor)
{
  EXPECT_EQ(node1_->x, 5);
  EXPECT_EQ(node1_->y, 10);
  EXPECT_DOUBLE_EQ(node1_->g_cost, 15.0);
  EXPECT_DOUBLE_EQ(node1_->h_cost, 20.0);
  EXPECT_DOUBLE_EQ(node1_->f_cost, 35.0);  // g_cost + h_cost
  EXPECT_EQ(node1_->parent, nullptr);
}

/**
 * @brief 测试AStarNode带父节点的构造函数
 */
TEST_F(AStarNodeTest, ConstructorWithParent)
{
  auto child_node = std::make_shared<AStarNode>(6, 11, 18.0, 22.0, node1_);
  
  EXPECT_EQ(child_node->x, 6);
  EXPECT_EQ(child_node->y, 11);
  EXPECT_DOUBLE_EQ(child_node->g_cost, 18.0);
  EXPECT_DOUBLE_EQ(child_node->h_cost, 22.0);
  EXPECT_DOUBLE_EQ(child_node->f_cost, 40.0);
  EXPECT_EQ(child_node->parent, node1_);
}

/**
 * @brief 测试AStarNode比较操作符
 */
TEST_F(AStarNodeTest, ComparisonOperator)
{
  // node1: f_cost = 35.0, h_cost = 20.0
  // node2: f_cost = 30.0, h_cost = 18.0
  // node3: f_cost = 35.0, h_cost = 10.0
  
  EXPECT_TRUE(*node1_ > *node2_);   // 更大的f_cost
  EXPECT_FALSE(*node2_ > *node1_);  // 更小的f_cost
  EXPECT_TRUE(*node1_ > *node3_);   // 相同f_cost，更大的h_cost
  EXPECT_FALSE(*node3_ > *node1_);  // 相同f_cost，更小的h_cost
}

/**
 * @brief CoordHash哈希函数测试类
 */
class CoordHashTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hash_func_ = CoordHash();
  }

  CoordHash hash_func_;
};

/**
 * @brief 测试CoordHash哈希函数
 */
TEST_F(CoordHashTest, HashFunction)
{
  std::pair<int, int> coord1(5, 10);
  std::pair<int, int> coord2(5, 10);
  std::pair<int, int> coord3(10, 5);
  
  // 相同坐标应该有相同哈希值
  EXPECT_EQ(hash_func_(coord1), hash_func_(coord2));
  
  // 不同坐标应该有不同哈希值（概率上）
  EXPECT_NE(hash_func_(coord1), hash_func_(coord3));
}

/**
 * @brief 测试CoordHash在unordered_set中的使用
 */
TEST_F(CoordHashTest, HashInUnorderedSet)
{
  std::unordered_set<std::pair<int, int>, CoordHash> coord_set;
  
  coord_set.insert({5, 10});
  coord_set.insert({3, 8});
  coord_set.insert({5, 10});  // 重复插入
  
  EXPECT_EQ(coord_set.size(), 2);  // 只有2个唯一元素
  EXPECT_TRUE(coord_set.count({5, 10}));
  EXPECT_TRUE(coord_set.count({3, 8}));
  EXPECT_FALSE(coord_set.count({7, 12}));
}

/**
 * @brief AStarPlanner算法方法测试类
 * 使用测试助手类测试私有方法
 */
class AStarPlannerMethodTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 创建测试costmap (20x20，分辨率0.05m)
    test_costmap_ = std::make_unique<TestCostmap>(20, 20, 0.05);
    test_costmap_->createSimpleTestScene();
    
    // 创建测试助手
    planner_helper_ = std::make_unique<AStarPlannerTestHelper>();
    planner_helper_->setCostmapForTesting(test_costmap_.get(), 0.05, "map");
    planner_helper_->setTestParameters(1.0, false, 250);
  }

  std::unique_ptr<TestCostmap> test_costmap_;
  std::unique_ptr<AStarPlannerTestHelper> planner_helper_;
};

/**
 * @brief 测试启发函数计算
 */
TEST_F(AStarPlannerMethodTest, CalculateHeuristic)
{
  // 测试欧几里得距离计算
  // 点(0,0)到点(3,4)的距离应该是5，乘以分辨率0.05
  double expected_distance = 5.0 * 0.05;
  double actual_distance = planner_helper_->testCalculateHeuristic(0, 0, 3, 4);
  
  EXPECT_DOUBLE_EQ(actual_distance, expected_distance);
  
  // 测试相同点的距离（应该为0）
  EXPECT_DOUBLE_EQ(planner_helper_->testCalculateHeuristic(5, 5, 5, 5), 0.0);
  
  // 测试水平距离
  double horizontal_distance = planner_helper_->testCalculateHeuristic(0, 0, 5, 0);
  EXPECT_DOUBLE_EQ(horizontal_distance, 5.0 * 0.05);
  
  // 测试垂直距离
  double vertical_distance = planner_helper_->testCalculateHeuristic(0, 0, 0, 5);
  EXPECT_DOUBLE_EQ(vertical_distance, 5.0 * 0.05);
}

/**
 * @brief 测试启发函数权重影响
 */
TEST_F(AStarPlannerMethodTest, CalculateHeuristicWithWeight)
{
  // 设置不同的启发权重
  planner_helper_->setTestParameters(2.0, false, 250);  // 权重2.0
  
  double base_distance = 5.0 * 0.05;
  double weighted_distance = planner_helper_->testCalculateHeuristic(0, 0, 3, 4);
  
  EXPECT_DOUBLE_EQ(weighted_distance, base_distance * 2.0);
}

/**
 * @brief 测试邻居节点获取
 */
TEST_F(AStarPlannerMethodTest, GetNeighbors)
{
  // 测试中心位置的8方向邻居
  auto neighbors = planner_helper_->testGetNeighbors(5, 5);
  
  EXPECT_EQ(neighbors.size(), 8);
  
  std::vector<std::pair<int, int>> expected_neighbors = {
      {4, 4}, {4, 5}, {4, 6},
      {5, 4},         {5, 6},
      {6, 4}, {6, 5}, {6, 6}
  };
  
  // 验证所有预期的邻居都存在
  for (const auto& expected : expected_neighbors)
  {
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), expected) != neighbors.end())
        << "Missing neighbor: (" << expected.first << ", " << expected.second << ")";
  }
}

/**
 * @brief 测试移动代价计算
 */
TEST_F(AStarPlannerMethodTest, GetMoveCost)
{
  double resolution = 0.05;
  
  // 水平移动代价 (5,5) -> (5,6)
  double horizontal_cost = planner_helper_->testGetMoveCost(5, 5, 5, 6);
  EXPECT_DOUBLE_EQ(horizontal_cost, resolution);
  
  // 垂直移动代价 (5,5) -> (6,5)
  double vertical_cost = planner_helper_->testGetMoveCost(5, 5, 6, 5);
  EXPECT_DOUBLE_EQ(vertical_cost, resolution);
  
  // 对角线移动代价 (5,5) -> (6,6)
  double diagonal_cost = planner_helper_->testGetMoveCost(5, 5, 6, 6);
  EXPECT_DOUBLE_EQ(diagonal_cost, resolution * std::sqrt(2));
  
  // 反向对角线移动代价 (5,5) -> (4,4)
  double reverse_diagonal_cost = planner_helper_->testGetMoveCost(5, 5, 4, 4);
  EXPECT_DOUBLE_EQ(reverse_diagonal_cost, resolution * std::sqrt(2));
}

/**
 * @brief 测试节点有效性检查
 */
TEST_F(AStarPlannerMethodTest, IsNodeValid)
{
  std::unordered_set<std::pair<int, int>, CoordHash> empty_closed_set;
  std::unordered_set<std::pair<int, int>, CoordHash> closed_set_with_nodes;
  closed_set_with_nodes.insert({5, 5});
  
  // 测试正常的自由空间
  EXPECT_TRUE(planner_helper_->testIsNodeValid(1, 1, empty_closed_set));
  
  // 测试边界外的点
  EXPECT_FALSE(planner_helper_->testIsNodeValid(-1, 5, empty_closed_set));
  EXPECT_FALSE(planner_helper_->testIsNodeValid(5, -1, empty_closed_set));
  EXPECT_FALSE(planner_helper_->testIsNodeValid(25, 5, empty_closed_set));
  EXPECT_FALSE(planner_helper_->testIsNodeValid(5, 25, empty_closed_set));
  
  // 测试已访问的节点
  EXPECT_FALSE(planner_helper_->testIsNodeValid(5, 5, closed_set_with_nodes));
  
  // 测试障碍物（根据createSimpleTestScene的设置）
  // 在(20/3, y)位置有垂直墙
  unsigned int wall_x = 20 / 3;  // 约等于6
  EXPECT_FALSE(planner_helper_->testIsNodeValid(wall_x, 2, empty_closed_set));
  
  // 测试未知区域（右上角）
  planner_helper_->setTestParameters(1.0, false, 250);  // 不允许未知区域
  EXPECT_FALSE(planner_helper_->testIsNodeValid(18, 18, empty_closed_set));
  
  // 允许未知区域
  planner_helper_->setTestParameters(1.0, true, 250);
  EXPECT_TRUE(planner_helper_->testIsNodeValid(18, 18, empty_closed_set));
}

/**
 * @brief 测试坐标转换
 */
TEST_F(AStarPlannerMethodTest, CoordinateConversion)
{
  // 测试网格坐标转世界坐标
  auto world_pose = planner_helper_->testGridToWorld(10, 10);
  
  EXPECT_EQ(world_pose.header.frame_id, "map");
  EXPECT_NEAR(world_pose.pose.position.x, 10 * 0.05, 5e-2);  // 允许浮点误差
  EXPECT_NEAR(world_pose.pose.position.y, 10 * 0.05, 5e-2);
    EXPECT_NEAR(world_pose.pose.position.z, 0.0, 1e-6);
    EXPECT_NEAR(world_pose.pose.orientation.w, 1.0, 1e-6);
  
  // 测试世界坐标转网格坐标
  geometry_msgs::msg::PoseStamped test_pose;
  test_pose.header.frame_id = "map";
  test_pose.pose.position.x = 0.5;  // 对应网格坐标10
  test_pose.pose.position.y = 0.5;  // 对应网格坐标10
  
  int grid_x, grid_y;
  bool conversion_success = planner_helper_->testWorldToGrid(test_pose, grid_x, grid_y);
  
  EXPECT_TRUE(conversion_success);
  EXPECT_EQ(grid_x, 10);
  EXPECT_EQ(grid_y, 10);
}

/**
 * @brief 测试路径平滑功能
 */
TEST_F(AStarPlannerMethodTest, SmoothPath)
{
  // 创建包含共线点的原始路径
  nav_msgs::msg::Path raw_path;
  raw_path.header.frame_id = "map";
  
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  
  // 创建一条包含共线点的路径: (0,0) -> (1,1) -> (2,2) -> (3,3) -> (3,4)
  std::vector<std::pair<double, double>> path_points = {
      {0.0, 0.0},
      {1.0, 1.0},  // 共线点
      {2.0, 2.0},  // 共线点
      {3.0, 3.0},  // 拐点
      {3.0, 4.0}   // 终点
  };
  
  for (const auto& point : path_points)
  {
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    raw_path.poses.push_back(pose);
  }
  
  // 执行路径平滑
  auto smoothed_path = planner_helper_->testSmoothPath(raw_path);
  
  // 平滑后的路径应该移除共线点，只保留关键点
  EXPECT_LT(smoothed_path.poses.size(), raw_path.poses.size());
  EXPECT_GE(smoothed_path.poses.size(), 2);  // 至少保留起点和终点
  
  // 起点和终点应该保持不变
  EXPECT_DOUBLE_EQ(smoothed_path.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(smoothed_path.poses.front().pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(smoothed_path.poses.back().pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(smoothed_path.poses.back().pose.position.y, 4.0);
}

/**
 * @brief 测试空路径和边界情况的路径平滑
 */
TEST_F(AStarPlannerMethodTest, SmoothPathEdgeCases)
{
  // 测试空路径
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = "map";
  auto smoothed_empty = planner_helper_->testSmoothPath(empty_path);
  EXPECT_EQ(smoothed_empty.poses.size(), 0);
  
  // 测试单点路径
  nav_msgs::msg::Path single_point_path;
  single_point_path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 1.0;
  pose.pose.orientation.w = 1.0;
  single_point_path.poses.push_back(pose);
  
  auto smoothed_single = planner_helper_->testSmoothPath(single_point_path);
  EXPECT_EQ(smoothed_single.poses.size(), 1);
  
  // 测试两点路径
  nav_msgs::msg::Path two_point_path = single_point_path;
  pose.pose.position.x = 2.0;
  pose.pose.position.y = 2.0;
  two_point_path.poses.push_back(pose);
  
  auto smoothed_two = planner_helper_->testSmoothPath(two_point_path);
  EXPECT_EQ(smoothed_two.poses.size(), 2);
}

/**
 * @brief 测试边界条件和错误处理
 */
class AStarPlannerEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_costmap_ = std::make_unique<TestCostmap>(10, 10, 0.1);
    planner_helper_ = std::make_unique<AStarPlannerTestHelper>();
    planner_helper_->setCostmapForTesting(test_costmap_.get(), 0.1, "map");
  }

  std::unique_ptr<TestCostmap> test_costmap_;
  std::unique_ptr<AStarPlannerTestHelper> planner_helper_;
};

/**
 * @brief 测试代价阈值影响
 */
TEST_F(AStarPlannerEdgeCaseTest, CostThresholdTest)
{
  std::unordered_set<std::pair<int, int>, CoordHash> empty_closed_set;
  
  // 设置一个中等代价的区域
  test_costmap_->setCost(5, 5, 150);
  
  // 设置低代价阈值，应该拒绝通过
  planner_helper_->setTestParameters(1.0, false, 100);
  EXPECT_FALSE(planner_helper_->testIsNodeValid(5, 5, empty_closed_set));
  
  // 设置高代价阈值，应该允许通过
  planner_helper_->setTestParameters(1.0, false, 200);
  EXPECT_TRUE(planner_helper_->testIsNodeValid(5, 5, empty_closed_set));
}

/**
 * @brief 测试极端情况下的坐标转换
 */
TEST_F(AStarPlannerEdgeCaseTest, ExtremeCaseCoordinateConversion)
{
  // 测试边界点的坐标转换
  auto corner_pose = planner_helper_->testGridToWorld(0, 0);
    EXPECT_NEAR(corner_pose.pose.position.x, 0.0, 6e-2);
    EXPECT_NEAR(corner_pose.pose.position.y, 0.0, 6e-2);

  // 测试最大边界点
  auto max_corner_pose = planner_helper_->testGridToWorld(9, 9);
    EXPECT_NEAR(max_corner_pose.pose.position.x, 9 * 0.1, 6e-2);
    EXPECT_NEAR(max_corner_pose.pose.position.y, 9 * 0.1, 6e-2);

  // 测试超出边界的世界坐标转换
  geometry_msgs::msg::PoseStamped out_of_bounds_pose;
  out_of_bounds_pose.header.frame_id = "map";
  out_of_bounds_pose.pose.position.x = -1.0;  // 负坐标
  out_of_bounds_pose.pose.position.y = -1.0;
  
  int grid_x, grid_y;
  bool conversion_success = planner_helper_->testWorldToGrid(out_of_bounds_pose, grid_x, grid_y);
  EXPECT_FALSE(conversion_success);  // 应该转换失败
}

/**
 * @brief 测试特殊路径的平滑处理
 */
TEST_F(AStarPlannerEdgeCaseTest, SpecialPathSmoothing)
{
  // 测试Z字形路径
  nav_msgs::msg::Path zigzag_path;
  zigzag_path.header.frame_id = "map";
  
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  
  // 创建Z字形路径: (0,0) -> (1,0) -> (1,1) -> (2,1) -> (2,2)
  std::vector<std::pair<double, double>> zigzag_points = {
      {0.0, 0.0},
      {1.0, 0.0},
      {1.0, 1.0},
      {2.0, 1.0},
      {2.0, 2.0}
  };
  
  for (const auto& point : zigzag_points)
  {
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    zigzag_path.poses.push_back(pose);
  }
  
  auto smoothed_zigzag = planner_helper_->testSmoothPath(zigzag_path);
  
  // Z字形路径没有共线点，所以平滑后大小应该相同
  EXPECT_EQ(smoothed_zigzag.poses.size(), zigzag_path.poses.size());
}

}  // namespace nav2_astar_planner

/**
 * @brief 主测试函数
 */
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}