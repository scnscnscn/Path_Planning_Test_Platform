#ifndef NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <unordered_map>

namespace nav2_astar_planner
{
  struct AStarNode
  {
    int x;             
    int y;             
    double g_cost;   
    double h_cost;      
    double f_cost;      
    AStarNode* parent; 

    // 构造函数
    AStarNode(int x_, int y_, double g_, double h_, AStarNode* p_)
        : x(x_), y(y_), g_cost(g_), h_cost(h_), f_cost(g_ + h_), parent(p_) {}

    // 优先队列排序规则：F代价小的节点优先出队（若F相同，H小的优先）
    bool operator>(const AStarNode& other) const
    {
      if (f_cost != other.f_cost)
      {
        return f_cost > other.f_cost;
      }
      return h_cost > other.h_cost;
    }
  };

  // -------------------------- A*规划器类 --------------------------
  class AStarPlanner : public nav2_core::GlobalPlanner
  {
  public:
    // 构造/析构函数
    AStarPlanner() = default;
    ~AStarPlanner() = default;

    // -------------------------- 重写Nav2基类方法 --------------------------
    // 1. 初始化（读取参数、初始化代价地图）
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // 2. 激活规划器（暂无需操作）
    void activate() override {}

    // 3. 失活规划器（暂无需操作）
    void deactivate() override {}

    // 4. 清理资源（暂无需操作）
    void cleanup() override {}

    // 5. 核心：生成A*路径
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

  private:
    // -------------------------- 私有成员与工具函数 --------------------------
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;             // 节点指针
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 代价地图ROS包装类
    nav2_costmap_2d::Costmap2D* costmap_;                         // 代价地图核心对象（获取格子代价）
    std::string global_frame_;                                    // 全局坐标系（如"map"）
    std::string planner_name_;                                    // 规划器名称（用于参数命名）

    // A*参数（可从ROS参数服务器读取）
    double heuristic_weight_;  // 启发函数权重（H = weight * 距离，默认1.0）
    double allow_unknown_;     // 是否允许路径经过未知区域（默认false）
    double inflation_radius_;  // 障碍物膨胀半径（默认0.2m，避免贴墙）
    int cost_threshold_;       // 允许的最大代价（超过则视为障碍物，默认250）

    // 工具函数1：计算启发函数（H代价）- 欧几里得距离（适用于2D平面，更平滑）
    double calculateHeuristic(int x1, int y1, int x2, int y2);

    // 工具函数2：检查节点是否合法（在地图内、非障碍物、未被访问）
    bool isNodeValid(int x, int y, std::unordered_map<std::string, bool>& closed_set);

    // 工具函数3：将网格坐标转换为世界坐标（米）
    geometry_msgs::msg::PoseStamped gridToWorld(int x, int y);

    // 工具函数4：生成节点唯一键（用于closed_set哈希表）
    std::string getNodeKey(int x, int y) { return std::to_string(x) + "," + std::to_string(y); }
  };

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_