#ifndef NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_astar_planner
{
  // A*节点结构体
  struct AStarNode
  {
    int x, y;                           // 网格坐标
    double g_cost;                      // 起始点到当前点的实际代价
    double h_cost;                      // 启发式代价（当前点到目标点的估计代价）
    double f_cost;                      // 总代价 = g_cost + h_cost
    std::shared_ptr<AStarNode> parent;  // 父节点指针

    // 构造函数
    AStarNode(int x_, int y_, double g_, double h_, std::shared_ptr<AStarNode> p_ = nullptr)
        : x(x_), y(y_), g_cost(g_), h_cost(h_), f_cost(g_ + h_), parent(p_) {}

    // 优先队列排序规则（F代价小的节点优先）
    bool operator>(const AStarNode& other) const
    {
      if (std::abs(f_cost - other.f_cost) > 1e-6)
      {
        return f_cost > other.f_cost;
      }
      return h_cost > other.h_cost;  // F相等时，H小的优先
    }
  };

  // 坐标对哈希函数（用于unordered_set/unordered_map）
  struct CoordHash
  {
    std::size_t operator()(const std::pair<int, int>& coord) const
    {
      return std::hash<int>()(coord.first) ^ (std::hash<int>()(coord.second) << 1);
    }
  };

  // A*规划器类
  class AStarPlanner : public nav2_core::GlobalPlanner
  {
  public:
    AStarPlanner() = default;
    ~AStarPlanner() = default;

    // 禁用拷贝构造函数和赋值操作符以防止不必要的复制
    AStarPlanner(const AStarPlanner&) = delete;
    AStarPlanner& operator=(const AStarPlanner&) = delete;

    // 允许测试辅助类访问私有成员和方法
    friend class AStarPlannerTestHelper;

    // 初始化配置
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // 资源清理
    void cleanup() override;

    // 激活规划器
    void activate() override;

    // 停用规划器
    void deactivate() override;

    // 核心规划函数
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

  private:
    // ROS相关成员
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;             // 节点指针
    std::shared_ptr<tf2_ros::Buffer> tf_;                         // TF缓冲区
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // 代价地图ROS包装器
    nav2_costmap_2d::Costmap2D* costmap_;                         // 代价地图指针
    rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};   // 日志
    rclcpp::Clock::SharedPtr clock_;                              // 时钟
    std::string planner_name_;                                    // 规划器名称
    std::string global_frame_;                                    // 全局坐标系名称

    // A*算法参数
    double heuristic_weight_;       // 启发函数权重
    bool allow_unknown_;            // 是否允许未知区域
    double inflation_radius_;       // 膨胀半径
    unsigned char cost_threshold_;  // 代价阈值

    // 启发函数（欧几里得距离）
    double calculateHeuristic(int x1, int y1, int x2, int y2) const;

    // 检查节点是否有效
    bool isNodeValid(int x, int y, const std::unordered_set<std::pair<int, int>, CoordHash>& closed_set) const;

    // 网格坐标转世界坐标
    geometry_msgs::msg::PoseStamped gridToWorld(int x, int y) const;

    // 世界坐标转网格坐标
    bool worldToGrid(const geometry_msgs::msg::PoseStamped& pose, int& mx, int& my) const;

    // 获取相邻节点（8方向）
    std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;

    // 计算移动代价
    double getMoveCost(int x1, int y1, int x2, int y2) const;

    // 路径平滑处理
    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& raw_path) const;
  };

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_