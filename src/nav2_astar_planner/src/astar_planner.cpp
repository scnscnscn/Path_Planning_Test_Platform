#include "nav2_astar_planner/astar_planner.hpp"
#include "pluginlib/class_list_macros.hpp"  // 插件导出宏

namespace nav2_astar_planner
{

  // -------------------------- 1. 实现configure()：初始化参数与代价地图 --------------------------
  void AStarPlanner::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    // 初始化成员变量
    node_ = parent.lock();
    planner_name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();             // 获取代价地图核心对象
    global_frame_ = costmap_ros->getGlobalFrameID();  // 全局坐标系（如"map"）

    // -------------------------- 从ROS参数服务器读取A*参数 --------------------------
    // 1. 启发函数权重（默认1.0，权重越大越偏向贪心，权重越小越偏向Dijkstra）
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
    node_->get_parameter(planner_name_ + ".heuristic_weight", heuristic_weight_);

    // 2. 是否允许经过未知区域（默认false，未知区域代价为255）
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".allow_unknown", rclcpp::ParameterValue(false));
    node_->get_parameter(planner_name_ + ".allow_unknown", allow_unknown_);

    // 3. 障碍物膨胀半径（默认0.2m，需与代价地图膨胀参数一致）
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".inflation_radius", rclcpp::ParameterValue(0.2));
    node_->get_parameter(planner_name_ + ".inflation_radius", inflation_radius_);

    // 4. 代价阈值（超过则视为障碍物，默认250；255为未知区域）
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".cost_threshold", rclcpp::ParameterValue(250));
    node_->get_parameter(planner_name_ + ".cost_threshold", cost_threshold_);

    RCLCPP_INFO(node_->get_logger(), "A* Planner configured successfully!");
  }

  // -------------------------- 2. 实现启发函数：欧几里得距离 --------------------------
  double AStarPlanner::calculateHeuristic(int x1, int y1, int x2, int y2)
  {
    // 网格分辨率（米/格子）：代价地图每个格子的实际尺寸
    double resolution = costmap_->getResolution();
    // 世界坐标下的欧几里得距离 = 网格距离 * 分辨率 * 权重
    double dx = (x2 - x1) * resolution;
    double dy = (y2 - y1) * resolution;
    return heuristic_weight_ * std::hypot(dx, dy);
  }

  // -------------------------- 3. 实现节点合法性检查 --------------------------
  bool AStarPlanner::isNodeValid(int x, int y, std::unordered_map<std::string, bool>& closed_set)
  {
    // 1. 检查是否在地图边界内（x：列数，y：行数）
    if (x < 0 || x >= costmap_->getSizeInCellsX() || y < 0 || y >= costmap_->getSizeInCellsY())
    {
      return false;
    }

    // 2. 检查是否已在关闭列表（已扩展过的节点）
    std::string key = getNodeKey(x, y);
    if (closed_set.find(key) != closed_set.end())
    {
      return false;
    }

    // 3. 检查格子代价（是否为障碍物/未知区域）
    unsigned char cost = costmap_->getCost(x, y);
    // 规则：代价>阈值 → 障碍物；未知区域（255）且不允许 → 无效
    if (cost > cost_threshold_ || (cost == 255 && !allow_unknown_))
    {
      return false;
    }

    return true;
  }

  // -------------------------- 4. 实现网格坐标→世界坐标转换 --------------------------
  geometry_msgs::msg::PoseStamped AStarPlanner::gridToWorld(int x, int y)
  {
    geometry_msgs::msg::PoseStamped world_pose;
    world_pose.header.frame_id = global_frame_;
    world_pose.header.stamp = node_->now();

    // 代价地图原点（世界坐标）：地图左下角的x/y坐标
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();
    // 网格分辨率（米/格子）
    double resolution = costmap_->getResolution();

    // 世界坐标 = 原点坐标 + 网格坐标 * 分辨率（注意：y是行数，对应世界坐标的y轴）
    world_pose.pose.position.x = origin_x + (x + 0.5) * resolution;  // +0.5是取格子中心
    world_pose.pose.position.y = origin_y + (y + 0.5) * resolution;
    world_pose.pose.position.z = 0.0;  // 2D规划，z=0

    // 姿态：默认无旋转（单位四元数，x=y=z=0, w=1）
    world_pose.pose.orientation.x = 0.0;
    world_pose.pose.orientation.y = 0.0;
    world_pose.pose.orientation.z = 0.0;
    world_pose.pose.orientation.w = 1.0;

    return world_pose;
  }

  // -------------------------- 5. 核心：实现createPlan()：A*路径搜索 --------------------------
  nav_msgs::msg::Path AStarPlanner::createPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal,
      const std::function<bool()>& cancel_checker)
  {
    nav_msgs::msg::Path global_path;  // 输出路径
    global_path.header.frame_id = global_frame_;
    global_path.header.stamp = node_->now();

    // -------------------------- 第一步：检查坐标系一致性 --------------------------
    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Start/Goal must be in %s frame!", global_frame_.c_str());
      return global_path;
    }

    // -------------------------- 第二步：世界坐标→网格坐标转换 --------------------------
    int start_x, start_y;  // 起点网格坐标
    int goal_x, goal_y;    // 终点网格坐标
    double resolution = costmap_->getResolution();

    // 代价地图的worldToMap()：将世界坐标（米）转换为网格坐标（整数）
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y))
    {
      RCLCPP_ERROR(node_->get_logger(), "Start pose is out of costmap bounds!");
      return global_path;
    }
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
    {
      RCLCPP_ERROR(node_->get_logger(), "Goal pose is out of costmap bounds!");
      return global_path;
    }

    // 检查起点/终点是否为障碍物（若无效，直接返回空路径）
    std::unordered_map<std::string, bool> empty_closed_set;
    if (!isNodeValid(start_x, start_y, empty_closed_set) || !isNodeValid(goal_x, goal_y, empty_closed_set))
    {
      RCLCPP_ERROR(node_->get_logger(), "Start/Goal is in obstacle or unknown area!");
      return global_path;
    }

    // -------------------------- 第三步：初始化A*搜索（开放列表/关闭列表） --------------------------
    // 开放列表：优先队列（按F代价从小到大排序），存储待扩展的节点
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<>> open_list;
    // 关闭列表：哈希表（存储已扩展的节点，避免重复访问）
    std::unordered_map<std::string, bool> closed_set;
    // 节点指针列表：用于后续内存释放（避免内存泄漏）
    std::vector<AStarNode*> node_list;

    // 初始化起点节点（G=0，H=启发函数，父节点=nullptr）
    AStarNode* start_node = new AStarNode(
        start_x, start_y,
        0.0,                                                   // G代价：起点到自身的距离为0
        calculateHeuristic(start_x, start_y, goal_x, goal_y),  // H代价
        nullptr                                                // 父节点
    );
    open_list.push(*start_node);
    node_list.push_back(start_node);

    // 8个搜索方向（上下左右+对角线，可改为4方向减少路径拐点）
    const std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},  // 上下左右
        {1, 1},
        {1, -1},
        {-1, 1},
        {-1, -1}  // 对角线
    };
    double diagonal_cost = resolution * std::sqrt(2);  // 对角线移动的代价（米）
    double straight_cost = resolution;                 // 直线移动的代价（米）

    // -------------------------- 第四步：A*核心搜索循环 --------------------------
    bool goal_found = false;
    AStarNode* goal_node = nullptr;

    while (!open_list.empty() && !cancel_checker())
    {  // cancel_checker：检查是否取消导航
      // 1. 取出开放列表中F代价最小的节点（当前最优节点）
      AStarNode current_node = open_list.top();
      open_list.pop();

      // 2. 检查当前节点是否为终点（若到达，跳出循环）
      if (current_node.x == goal_x && current_node.y == goal_y)
      {
        goal_found = true;
        // 复制终点节点（优先队列存储的是副本，需从node_list中找原指针）
        for (auto& node : node_list)
        {
          if (node->x == goal_x && node->y == goal_y && node->f_cost == current_node.f_cost)
          {
            goal_node = node;
            break;
          }
        }
        break;
      }

      // 3. 检查当前节点是否已在关闭列表（若已存在，跳过）
      std::string current_key = getNodeKey(current_node.x, current_node.y);
      if (closed_set.find(current_key) != closed_set.end())
      {
        continue;
      }
      // 将当前节点加入关闭列表（标记为已扩展）
      closed_set[current_key] = true;

      // 4. 扩展当前节点的8个邻居
      for (const auto& dir : directions)
      {
        int neighbor_x = current_node.x + dir.first;
        int neighbor_y = current_node.y + dir.second;

        // 4.1 检查邻居节点是否合法（边界、障碍物、未扩展）
        if (!isNodeValid(neighbor_x, neighbor_y, closed_set))
        {
          continue;
        }

        // 4.2 计算邻居节点的G代价（当前节点G + 移动代价）
        double move_cost = (dir.first != 0 && dir.second != 0) ? diagonal_cost : straight_cost;
        double neighbor_g = current_node.g_cost + move_cost;

        // 4.3 计算邻居节点的H代价和F代价
        double neighbor_h = calculateHeuristic(neighbor_x, neighbor_y, goal_x, goal_y);
        double neighbor_f = neighbor_g + neighbor_h;

        // 4.4 创建邻居节点并加入开放列表
        AStarNode* neighbor_node = new AStarNode(neighbor_x, neighbor_y, neighbor_g, neighbor_h, &current_node);
        open_list.push(*neighbor_node);
        node_list.push_back(neighbor_node);
      }
    }

    // -------------------------- 第五步：回溯路径（从终点→起点） --------------------------
    if (goal_found && goal_node != nullptr)
    {
      std::vector<geometry_msgs::msg::PoseStamped> path_poses;
      AStarNode* current_backtrack = goal_node;

      // 回溯：从终点遍历到起点
      while (current_backtrack != nullptr)
      {
        // 网格坐标→世界坐标
        geometry_msgs::msg::PoseStamped pose = gridToWorld(current_backtrack->x, current_backtrack->y);
        path_poses.push_back(pose);
        // 移动到父节点
        current_backtrack = current_backtrack->parent;
      }

      // 反转路径（起点→终点）
      std::reverse(path_poses.begin(), path_poses.end());
      // 将路径加入输出消息
      global_path.poses = path_poses;
      RCLCPP_INFO(node_->get_logger(), "A* Path found! Total waypoints: %lu", path_poses.size());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "A* Path not found!");
    }

    // -------------------------- 第六步：释放节点内存（避免内存泄漏） --------------------------
    for (auto& node : node_list)
    {
      delete node;
    }

    return global_path;
  }

}  // namespace nav2_astar_planner

// -------------------------- 导出A*插件（关键：让Nav2能加载） --------------------------
PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)