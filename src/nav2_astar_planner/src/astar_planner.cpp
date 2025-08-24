#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include "../include/nav2_astar_planner/astar_planner.hpp"

namespace nav2_astar_planner
{

  void AStarPlanner::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    planner_name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    clock_ = node_->get_clock();

    // 声明参数
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".allow_unknown", rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".cost_threshold", rclcpp::ParameterValue(250));

    // 获取参数
    node_->get_parameter(planner_name_ + ".heuristic_weight", heuristic_weight_);
    node_->get_parameter(planner_name_ + ".allow_unknown", allow_unknown_);

    int temp_threshold;
    node_->get_parameter(planner_name_ + ".cost_threshold", temp_threshold);
    cost_threshold_ = static_cast<unsigned char>(std::min(255, std::max(0, temp_threshold)));

    RCLCPP_INFO(
        logger_,
        "Configured A* planner with parameters: "
        "heuristic_weight=%.2f, allow_unknown=%s, "
        "cost_threshold=%d",
        heuristic_weight_, allow_unknown_ ? "true" : "false",
        static_cast<int>(cost_threshold_));
  }

  void AStarPlanner::cleanup()
  {
    RCLCPP_INFO(logger_, "Cleaning up A* planner");
  }

  void AStarPlanner::activate()
  {
    RCLCPP_INFO(logger_, "Activating A* planner");
  }

  void AStarPlanner::deactivate()
  {
    RCLCPP_INFO(logger_, "Deactivating A* planner");
  }

  nav_msgs::msg::Path AStarPlanner::createPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();

    // 检查坐标系是否一致
    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          logger_,
          "Start and goal must be in global frame (%s), but got start=%s, goal=%s",
          global_frame_.c_str(), start.header.frame_id.c_str(), goal.header.frame_id.c_str());
      return path;
    }

    // 转换为网格坐标
    int start_x, start_y, goal_x, goal_y;
    if (!worldToGrid(start, start_x, start_y) || !worldToGrid(goal, goal_x, goal_y))
    {
      RCLCPP_ERROR(logger_, "Start or goal is outside costmap bounds");
      return path;
    }

    // 检查起点和终点有效性
    std::unordered_set<std::pair<int, int>, CoordHash> dummy_closed;
    if (!isNodeValid(start_x, start_y, dummy_closed))
    {
      RCLCPP_ERROR(logger_, "Start position is invalid (obstacle or out of bounds)");
      return path;
    }
    if (!isNodeValid(goal_x, goal_y, dummy_closed))
    {
      RCLCPP_ERROR(logger_, "Goal position is invalid (obstacle or out of bounds)");
      return path;
    }

    // A*算法初始化
    using NodePtr = std::shared_ptr<AStarNode>;
    std::priority_queue<NodePtr, std::vector<NodePtr>, std::greater<NodePtr>> open_set;
    std::unordered_set<std::pair<int, int>, CoordHash> closed_set;
    std::unordered_map<std::pair<int, int>, double, CoordHash> g_costs;

    // 起点入队
    auto start_node = std::make_shared<AStarNode>(
        start_x, start_y, 0.0,
        calculateHeuristic(start_x, start_y, goal_x, goal_y));
    open_set.push(start_node);
    g_costs[{start_x, start_y}] = 0.0;

    NodePtr goal_node = nullptr;

    // A*主循环
    while (!open_set.empty())
    {
      auto current = open_set.top();
      open_set.pop();
      std::pair<int, int> current_coord = {current->x, current->y};

      // 已到达目标
      if (current->x == goal_x && current->y == goal_y)
      {
        goal_node = current;
        break;
      }

      // 已处理过该节点
      if (closed_set.count(current_coord))
      {
        continue;
      }
      closed_set.insert(current_coord);

      // 遍历邻居
      for (const auto& neighbor : getNeighbors(current->x, current->y))
      {
        int nx = neighbor.first;
        int ny = neighbor.second;
        std::pair<int, int> neighbor_coord = {nx, ny};

        // 检查邻居有效性
        if (!isNodeValid(nx, ny, closed_set))
        {
          continue;
        }

        // 计算新代价
        double new_g = current->g_cost + getMoveCost(current->x, current->y, nx, ny);
        if (g_costs.count(neighbor_coord) && new_g >= g_costs[neighbor_coord])
        {
          continue;  // 已有更优路径
        }

        // 更新代价并加入开放集
        double h = calculateHeuristic(nx, ny, goal_x, goal_y);
        auto neighbor_node = std::make_shared<AStarNode>(nx, ny, new_g, h, current);
        open_set.push(neighbor_node);
        g_costs[neighbor_coord] = new_g;
      }
    }

    // 构建路径
    if (!goal_node)
    {
      RCLCPP_WARN(logger_, "No path found from start to goal");
      return path;
    }

    // 回溯路径
    std::vector<NodePtr> raw_path;
    for (auto node = goal_node; node != nullptr; node = node->parent)
    {
      raw_path.push_back(node);
    }
    std::reverse(raw_path.begin(), raw_path.end());

    // 转换为ROS路径消息
    for (const auto& node : raw_path)
    {
      path.poses.push_back(gridToWorld(node->x, node->y));
    }

    // 路径平滑
    path = smoothPath(path);

    RCLCPP_INFO(logger_, "Found path with %zu waypoints", path.poses.size());
    return path;
  }

  double AStarPlanner::calculateHeuristic(int x1, int y1, int x2, int y2) const
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return heuristic_weight_ * std::hypot(dx, dy) * costmap_->getResolution();
  }

  bool AStarPlanner::isNodeValid(
      int x, int y,
      const std::unordered_set<std::pair<int, int>, CoordHash>& closed_set) const
  {
    // 检查边界
    if (x < 0 || x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
        y < 0 || y >= static_cast<int>(costmap_->getSizeInCellsY()))
    {
      return false;
    }

    // 检查是否已访问
    if (closed_set.count({x, y}))
    {
      return false;
    }

    // 检查代价
    unsigned char cost = costmap_->getCost(x, y);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      return false;  // 致命障碍物
    }
    if (cost == nav2_costmap_2d::NO_INFORMATION)
    {
      // 未知区域
      return allow_unknown_;
    }
    if (cost > cost_threshold_)
    {
      return false;  // 超过代价阈值
    }

    return true;
  }

  geometry_msgs::msg::PoseStamped AStarPlanner::gridToWorld(int x, int y) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node_->now();

    double wx, wy;
    costmap_->mapToWorld(x, y, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;  // 默认朝向

    return pose;
  }

  bool AStarPlanner::worldToGrid(const geometry_msgs::msg::PoseStamped& pose, int& mx, int& my) const
  {
    unsigned int ux, uy;
    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, ux, uy))
    {
      return false;
    }
    mx = static_cast<int>(ux);
    my = static_cast<int>(uy);
    return true;
  }

  std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int x, int y) const
  {
    return {
        {x - 1, y - 1}, {x - 1, y}, {x - 1, y + 1}, {x, y - 1}, {x, y + 1}, {x + 1, y - 1}, {x + 1, y}, {x + 1, y + 1}};
  }

  double AStarPlanner::getMoveCost(int x1, int y1, int x2, int y2) const
  {
    double res = costmap_->getResolution();
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    return (dx == 1 && dy == 1) ? res * std::sqrt(2) : res;
  }

  nav_msgs::msg::Path AStarPlanner::smoothPath(const nav_msgs::msg::Path& raw_path) const
  {
    // 三次B样条平滑路径
    if (raw_path.poses.size() <= 3)
    {
      return raw_path;
    }

    nav_msgs::msg::Path smoothed;
    smoothed.header = raw_path.header;

    const double step = 0.05; // 更小步长，提升平滑度
    size_t n = raw_path.poses.size();

    // B样条基函数
    auto bspline = [](double t, double p0, double p1, double p2, double p3) {
      double t2 = t * t;
      double t3 = t2 * t;
      return (
        (-t3 + 3 * t2 - 3 * t + 1) * p0 +
        (3 * t3 - 6 * t2 + 4) * p1 +
        (-3 * t3 + 3 * t2 + 3 * t + 1) * p2 +
        (t3) * p3
      ) / 6.0;
    };

    // 边界点扩展，首尾各补一个点
    std::vector<geometry_msgs::msg::Pose> ctrl_pts;
    ctrl_pts.reserve(n + 2);
    ctrl_pts.push_back(raw_path.poses[0].pose); // 首点复制
    for (size_t i = 0; i < n; ++i)
      ctrl_pts.push_back(raw_path.poses[i].pose);
    ctrl_pts.push_back(raw_path.poses[n - 1].pose); // 尾点复制

    // B样条插值
    for (size_t i = 0; i < n; ++i)
    {
      const auto& p0 = ctrl_pts[i].position;
      const auto& p1 = ctrl_pts[i + 1].position;
      const auto& p2 = ctrl_pts[i + 2].position;
      const auto& p3 = ctrl_pts[i + 3].position;
      for (double t = 0; t < 1.0; t += step)
      {
        double x = bspline(t, p0.x, p1.x, p2.x, p3.x);
        double y = bspline(t, p0.y, p1.y, p2.y, p3.y);
        geometry_msgs::msg::PoseStamped pt;
        pt.header = raw_path.header;
        pt.pose.position.x = x;
        pt.pose.position.y = y;
        pt.pose.position.z = 0.0;
        pt.pose.orientation.w = 1.0;
        smoothed.poses.push_back(pt);
      }
    }
    // 保证终点加入
    smoothed.poses.push_back(raw_path.poses.back());
    return smoothed;
  }

}  // namespace nav2_astar_planner

// 注册为Nav2全局规划器插件
PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)