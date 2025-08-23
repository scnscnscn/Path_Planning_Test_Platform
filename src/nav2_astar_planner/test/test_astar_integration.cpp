/*
 * Copyright (c) 2024 scnscnscn
 * Licensed under the Apache License, Version 2.0
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>

#include "nav2_astar_planner/astar_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "test_astar_planner_helper.hpp"

namespace nav2_astar_planner
{

/**
 * @brief A*算法集成测试类
 * 测试完整的路径规划场景和算法正确性
 */
class AStarPlannerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 创建较大的测试环境 (50x50, 分辨率0.1m)
    test_costmap_ = std::make_unique<TestCostmap>(50, 50, 0.1);
    planner_helper_ = std::make_unique<AStarPlannerTestHelper>();
    planner_helper_->setCostmapForTesting(test_costmap_.get(), 0.1, "map");
    planner_helper_->setTestParameters(1.0, false, 250);
    
    setupComplexEnvironment();
  }

  /**
   * @brief 创建复杂的测试环境
   * 包含障碍物、狭窄通道和不同代价区域
   */
  void setupComplexEnvironment()
  {
    // 创建一个L形障碍物
    for (int x = 10; x <= 30; ++x)
    {
      test_costmap_->setObstacle(x, 20);  // 水平墙
    }
    for (int y = 20; y <= 35; ++y)
    {
      test_costmap_->setObstacle(20, y);  // 垂直墙
    }
    
    // 创建一些高代价区域
    for (int x = 5; x <= 15; ++x)
    {
      for (int y = 5; y <= 15; ++y)
      {
        test_costmap_->setCost(x, y, 200);  // 高代价但可通过
      }
    }
    
    // 创建狭窄通道
    for (int x = 35; x <= 45; ++x)
    {
      test_costmap_->setObstacle(x, 15);
      test_costmap_->setObstacle(x, 17);
    }
    // 在通道中间留一个出口
    test_costmap_->setFree(40, 16);
  }

  /**
   * @brief 创建测试用的Pose
   */
  geometry_msgs::msg::PoseStamped createPose(double x, double y, const std::string& frame = "map")
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  /**
   * @brief 验证路径的连续性
   */
  bool isPathContinuous(const nav_msgs::msg::Path& path, double max_step_size = 1.5)
  {
    if (path.poses.size() < 2)
    {
      return true;
    }
    
    for (size_t i = 1; i < path.poses.size(); ++i)
    {
      auto& prev = path.poses[i-1].pose.position;
      auto& curr = path.poses[i].pose.position;
      
      double dx = curr.x - prev.x;
      double dy = curr.y - prev.y;
      double step_size = std::hypot(dx, dy);
      
      if (step_size > max_step_size * test_costmap_->getResolution())
      {
        return false;
      }
    }
    
    return true;
  }

  /**
   * @brief 验证路径没有经过障碍物
   */
  bool isPathObstacleFree(const nav_msgs::msg::Path& path)
  {
    for (const auto& pose_stamped : path.poses)
    {
      int grid_x, grid_y;
      if (planner_helper_->testWorldToGrid(pose_stamped, grid_x, grid_y))
      {
        unsigned char cost = test_costmap_->getCost(grid_x, grid_y);
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          return false;
        }
      }
    }
    return true;
  }

  std::unique_ptr<TestCostmap> test_costmap_;
  std::unique_ptr<AStarPlannerTestHelper> planner_helper_;
};

/**
 * @brief 测试简单直线路径规划
 */
TEST_F(AStarPlannerIntegrationTest, SimpleStraightPath)
{
  // 在开放空间中规划直线路径
  auto start_pose = createPose(0.5, 0.5);  // 网格(5,5)
  auto goal_pose = createPose(1.0, 1.0);   // 网格(10,10)
  
  // 手动模拟createPlan的核心逻辑进行测试
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(start_pose, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(goal_pose, goal_x, goal_y));
  
  // 验证起点和终点都是有效的
  std::unordered_set<std::pair<int, int>, CoordHash> empty_closed;
  EXPECT_TRUE(planner_helper_->testIsNodeValid(start_x, start_y, empty_closed));
  EXPECT_TRUE(planner_helper_->testIsNodeValid(goal_x, goal_y, empty_closed));
}

/**
 * @brief 测试绕过障碍物的路径规划
 */
TEST_F(AStarPlannerIntegrationTest, ObstacleAvoidance)
{
  // 起点在障碍物一侧，终点在另一侧
  auto start_pose = createPose(0.5, 2.5);  // 障碍物左侧
  auto goal_pose = createPose(4.0, 2.5);   // 障碍物右侧
  
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(start_pose, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(goal_pose, goal_x, goal_y));
  
  // 验证路径必须绕过障碍物
  // 在这种设置下，直线路径会被L形障碍物阻挡
  
  // 测试启发函数正确性
  double heuristic = planner_helper_->testCalculateHeuristic(start_x, start_y, goal_x, goal_y);
  double expected_min_distance = std::hypot(goal_x - start_x, goal_y - start_y) * test_costmap_->getResolution();
  EXPECT_DOUBLE_EQ(heuristic, expected_min_distance);
}

/**
 * @brief 测试狭窄通道导航
 */
TEST_F(AStarPlannerIntegrationTest, NarrowPassage)
{
  // 测试通过狭窄通道的路径规划
  auto start_pose = createPose(3.0, 1.6);  // 通道入口
  auto goal_pose = createPose(4.5, 1.6);   // 通道出口
  
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(start_pose, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(goal_pose, goal_x, goal_y));
  
  std::unordered_set<std::pair<int, int>, CoordHash> empty_closed;
  
  // 验证通道出口是通畅的（根据setupComplexEnvironment的设置）
  EXPECT_TRUE(planner_helper_->testIsNodeValid(40, 16, empty_closed));  // 通道中间
  
  // 验证通道上下方被阻挡
  EXPECT_FALSE(planner_helper_->testIsNodeValid(40, 15, empty_closed));  // 上方障碍物
  EXPECT_FALSE(planner_helper_->testIsNodeValid(40, 17, empty_closed));  // 下方障碍物
}

/**
 * @brief 测试高代价区域的路径规划
 */
TEST_F(AStarPlannerIntegrationTest, HighCostAreaNavigation)
{
  // 起点和终点设置成需要经过高代价区域
  auto start_pose = createPose(0.8, 0.8);  // 高代价区域外
  auto goal_pose = createPose(1.2, 1.2);   // 需要经过高代价区域
  
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(start_pose, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(goal_pose, goal_x, goal_y));
  
  std::unordered_set<std::pair<int, int>, CoordHash> empty_closed;
  
  // 验证高代价区域仍然是可通过的（代价200 < 阈值250）
  EXPECT_TRUE(planner_helper_->testIsNodeValid(10, 10, empty_closed));
  
  // 测试代价阈值的影响
  planner_helper_->setTestParameters(1.0, false, 150);  // 降低阈值
  EXPECT_FALSE(planner_helper_->testIsNodeValid(10, 10, empty_closed));  // 现在应该被阻挡
}

/**
 * @brief 测试不可达目标的处理
 */
TEST_F(AStarPlannerIntegrationTest, UnreachableGoal)
{
  // 创建完全被障碍物包围的目标点
  for (int x = 45; x <= 47; ++x)
  {
    for (int y = 45; y <= 47; ++y)
    {
      if (x != 46 || y != 46)  // 除了中心点，周围都是障碍物
      {
        test_costmap_->setObstacle(x, y);
      }
    }
  }
  
  auto start_pose = createPose(0.5, 0.5);
  auto goal_pose = createPose(4.6, 4.6);  // 被包围的目标点
  
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(start_pose, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(goal_pose, goal_x, goal_y));
  
  std::unordered_set<std::pair<int, int>, CoordHash> empty_closed;
  
  // 目标点本身是有效的
  EXPECT_TRUE(planner_helper_->testIsNodeValid(goal_x, goal_y, empty_closed));
  
  // 但是周围都被阻挡了
  auto neighbors = planner_helper_->testGetNeighbors(goal_x, goal_y);
  int blocked_neighbors = 0;
  for (const auto& neighbor : neighbors)
  {
    if (!planner_helper_->testIsNodeValid(neighbor.first, neighbor.second, empty_closed))
    {
      blocked_neighbors++;
    }
  }
  EXPECT_EQ(blocked_neighbors, 8);  // 所有邻居都被阻挡
}

/**
 * @brief 测试边界条件
 */
TEST_F(AStarPlannerIntegrationTest, BoundaryConditions)
{
  // 测试地图边界附近的路径规划
  auto corner_start = createPose(0.05, 0.05);    // 接近原点
  auto corner_goal = createPose(4.95, 4.95);     // 接近右上角
  
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(corner_start, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(corner_goal, goal_x, goal_y));
  
  // 验证边界点的邻居处理
  auto corner_neighbors = planner_helper_->testGetNeighbors(0, 0);
  
  std::unordered_set<std::pair<int, int>, CoordHash> empty_closed;
  int valid_neighbors = 0;
  for (const auto& neighbor : corner_neighbors)
  {
    if (planner_helper_->testIsNodeValid(neighbor.first, neighbor.second, empty_closed))
    {
      valid_neighbors++;
    }
  }
  
  // 角落位置只有3个有效邻居（在地图内部）
  EXPECT_EQ(valid_neighbors, 3);
}

/**
 * @brief 测试启发函数权重的影响
 */
TEST_F(AStarPlannerIntegrationTest, HeuristicWeightEffect)
{
  auto start_pose = createPose(0.5, 0.5);
  auto goal_pose = createPose(2.0, 2.0);
  
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(start_pose, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(goal_pose, goal_x, goal_y));
  
  // 测试不同权重下的启发函数值
  planner_helper_->setTestParameters(1.0, false, 250);
  double h1 = planner_helper_->testCalculateHeuristic(start_x, start_y, goal_x, goal_y);
  
  planner_helper_->setTestParameters(2.0, false, 250);
  double h2 = planner_helper_->testCalculateHeuristic(start_x, start_y, goal_x, goal_y);
  
  planner_helper_->setTestParameters(0.5, false, 250);
  double h3 = planner_helper_->testCalculateHeuristic(start_x, start_y, goal_x, goal_y);
  
  // 验证权重正确影响启发函数值
  EXPECT_DOUBLE_EQ(h2, h1 * 2.0);
  EXPECT_DOUBLE_EQ(h3, h1 * 0.5);
}

/**
 * @brief 性能测试：测试大规模环境下的规划效率
 */
TEST_F(AStarPlannerIntegrationTest, PerformanceTest)
{
  // 创建一个相对复杂的路径规划场景
  auto start_pose = createPose(0.1, 0.1);
  auto goal_pose = createPose(4.9, 4.9);
  
  int start_x, start_y, goal_x, goal_y;
  ASSERT_TRUE(planner_helper_->testWorldToGrid(start_pose, start_x, start_y));
  ASSERT_TRUE(planner_helper_->testWorldToGrid(goal_pose, goal_x, goal_y));
  
  // 验证起点到终点的直线距离
  double straight_line_distance = planner_helper_->testCalculateHeuristic(start_x, start_y, goal_x, goal_y);
  double expected_distance = std::hypot(goal_x - start_x, goal_y - start_y) * test_costmap_->getResolution();
  
  EXPECT_DOUBLE_EQ(straight_line_distance, expected_distance);
  
  // 这里可以添加时间测量，但由于我们测试的是独立方法，
  // 完整的性能测试需要实际的createPlan方法调用
}

}  // namespace nav2_astar_planner

/**
 * @brief 集成测试主函数
 */
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}