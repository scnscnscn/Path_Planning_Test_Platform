/*
 * Copyright (c) 2024 scnscnscn
 * Licensed under the Apache License, Version 2.0
 */

#ifndef NAV2_ASTAR_PLANNER__TEST_ASTAR_PLANNER_HELPER_HPP_
#define NAV2_ASTAR_PLANNER__TEST_ASTAR_PLANNER_HELPER_HPP_

#include <memory>
#include <vector>
#include <utility>
#include <unordered_set>

#include "nav2_astar_planner/astar_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_astar_planner
{


/**
 * @brief 测试辅助类，用于访问AStarPlanner的私有方法
 * 
 * 这个类继承自AStarPlanner，并提供公共接口来测试私有方法。
 * 这是一种常见的单元测试模式，用于测试类的内部逻辑。
 */
class AStarPlannerTestHelper : public AStarPlanner
{
public:
  AStarPlannerTestHelper() = default;
  ~AStarPlannerTestHelper() = default;

  /**
   * @brief 设置测试用的costmap
   * @param costmap 测试用的costmap指针
   * @param resolution 地图分辨率
   * @param global_frame 全局坐标系名称
   */
  void setCostmapForTesting(nav2_costmap_2d::Costmap2D* costmap, 
                           double resolution, 
                           const std::string& global_frame = "map")
  {
    costmap_ = costmap;
    global_frame_ = global_frame;
    heuristic_weight_ = 1.0;  // 默认启发权重
  }

  /**
   * @brief 测试用的启发函数计算接口
   */
  double testCalculateHeuristic(int x1, int y1, int x2, int y2) const
  {
    return calculateHeuristic(x1, y1, x2, y2);
  }

  /**
   * @brief 测试用的节点有效性检查接口
   */
  bool testIsNodeValid(int x, int y, 
                      const std::unordered_set<std::pair<int, int>, CoordHash>& closed_set) const
  {
    return isNodeValid(x, y, closed_set);
  }

  /**
   * @brief 测试用的网格坐标转世界坐标接口
   */
  geometry_msgs::msg::PoseStamped testGridToWorld(int x, int y) const
  {
    // 为测试创建一个简单的mock时钟
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    
    double wx, wy;
    costmap_->mapToWorld(x, y, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    
    return pose;
  }

  /**
   * @brief 测试用的世界坐标转网格坐标接口
   */
  bool testWorldToGrid(const geometry_msgs::msg::PoseStamped& pose, int& mx, int& my) const
  {
    return worldToGrid(pose, mx, my);
  }

  /**
   * @brief 测试用的邻居获取接口
   */
  std::vector<std::pair<int, int>> testGetNeighbors(int x, int y) const
  {
    return getNeighbors(x, y);
  }

  /**
   * @brief 测试用的移动代价计算接口
   */
  double testGetMoveCost(int x1, int y1, int x2, int y2) const
  {
    return getMoveCost(x1, y1, x2, y2);
  }

  /**
   * @brief 测试用的路径平滑接口
   */
  nav_msgs::msg::Path testSmoothPath(const nav_msgs::msg::Path& raw_path) const
  {
    return smoothPath(raw_path);
  }

  /**
   * @brief 设置测试参数
   */
  void setTestParameters(double heuristic_weight = 1.0, 
                        bool allow_unknown = false,
                        unsigned char cost_threshold = 250)
  {
    heuristic_weight_ = heuristic_weight;
    allow_unknown_ = allow_unknown;
    cost_threshold_ = cost_threshold;
  }
};

/**
 * @brief 简化的模拟Costmap2D类，专门用于单元测试
 * 
 * 这个类提供了一个轻量级的costmap实现，
 * 不依赖ROS2运行时环境，适合单元测试使用。
 */
class TestCostmap : public nav2_costmap_2d::Costmap2D
{
public:
  /**
   * @brief 构造函数
   * @param cells_size_x X方向网格数量
   * @param cells_size_y Y方向网格数量  
   * @param resolution 网格分辨率(米/格)
   * @param origin_x 地图原点X坐标
   * @param origin_y 地图原点Y坐标
   */
  TestCostmap(unsigned int cells_size_x, 
              unsigned int cells_size_y, 
              double resolution,
              double origin_x = 0.0, 
              double origin_y = 0.0)
      : nav2_costmap_2d::Costmap2D(cells_size_x, cells_size_y, resolution, 
                                   origin_x, origin_y, nav2_costmap_2d::FREE_SPACE)
  {
    // 初始化为自由空间
  }

  /**
   * @brief 设置障碍物
   * @param x 网格X坐标
   * @param y 网格Y坐标
   * @param cost 代价值
   */
  void setObstacle(unsigned int x, unsigned int y, unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    if (x < getSizeInCellsX() && y < getSizeInCellsY())
    {
      setCost(x, y, cost);
    }
  }

  /**
   * @brief 设置自由空间
   * @param x 网格X坐标
   * @param y 网格Y坐标
   */
  void setFree(unsigned int x, unsigned int y)
  {
    if (x < getSizeInCellsX() && y < getSizeInCellsY())
    {
      setCost(x, y, nav2_costmap_2d::FREE_SPACE);
    }
  }

  /**
   * @brief 设置未知区域
   * @param x 网格X坐标
   * @param y 网格Y坐标
   */
  void setUnknown(unsigned int x, unsigned int y)
  {
    if (x < getSizeInCellsX() && y < getSizeInCellsY())
    {
      setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
    }
  }

  /**
   * @brief 创建一个简单的测试场景
   * 
   * 创建一个包含障碍物的简单环境：
   * - 周围是自由空间
   * - 中间有一些障碍物
   * - 右上角有未知区域
   */
  void createSimpleTestScene()
  {
    unsigned int size_x = getSizeInCellsX();
    unsigned int size_y = getSizeInCellsY();
    
    // 设置一些障碍物形成简单的迷宫
    // 垂直墙
    for (unsigned int y = 2; y < size_y - 2; ++y)
    {
      if (y != size_y / 2)  // 在中间留一个通道
      {
        setObstacle(size_x / 3, y);
      }
    }
    
    // 水平墙
    for (unsigned int x = 2; x < size_x - 2; ++x)
    {
      if (x != size_x / 2)  // 在中间留一个通道
      {
        setObstacle(x, size_y / 3);
      }
    }
    
    // 设置一些未知区域
    for (unsigned int x = size_x * 3 / 4; x < size_x; ++x)
    {
      for (unsigned int y = size_y * 3 / 4; y < size_y; ++y)
      {
        setUnknown(x, y);
      }
    }
  }
};

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__TEST_ASTAR_PLANNER_HELPER_HPP_