#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include "../include/rrt_star_connect_planner/rrt_star_connect_planner.hpp"

namespace nav2_rrt_star_connect_planner
{
  // 计算启发式代价（欧氏距离）
  double RRTStarConnectPlanner::calculateHeuristic(double x1, double y1, double x2, double y2) const
  {
    return std::hypot(x2 - x1, y2 - y1);
  }

  // 在自由空间采样点
  std::pair<double, double> RRTStarConnectPlanner::sampleFree()
  {
    if (!costmap_)
    {
      RCLCPP_ERROR(logger_, "Costmap not initialized");
      return {0.0, 0.0};
    }

    // 获取地图边界
    double min_x = costmap_->getOriginX();
    double max_x = min_x + costmap_->getSizeInMetersX();
    double min_y = costmap_->getOriginY();
    double max_y = min_y + costmap_->getSizeInMetersY();

    // RViz采样点可视化
    visualization_msgs::msg::Marker sample_marker;
    sample_marker.header.frame_id = global_frame_;
    sample_marker.header.stamp = node_->now();
    sample_marker.ns = "rrt_samples";
    sample_marker.id = 0;
    sample_marker.type = visualization_msgs::msg::Marker::POINTS;
    sample_marker.action = visualization_msgs::msg::Marker::ADD;
    sample_marker.scale.x = 0.05;
    sample_marker.scale.y = 0.05;
    sample_marker.color.r = 0.0;
    sample_marker.color.g = 0.0;
    sample_marker.color.b = 1.0;
    sample_marker.color.a = 0.7;

    geometry_msgs::msg::Point pt;

    // 随机采样直到获得无碰撞点
    while (true)
    {
      double x = min_x + (max_x - min_x) * real_dist_(gen_);
      double y = min_y + (max_y - min_y) * real_dist_(gen_);
      pt.x = x;
      pt.y = y;
      pt.z = 0.0;
      sample_marker.points.push_back(pt);
      if (!isCollision(x, y))
      {
        // 只发布最后一个采样点，避免marker过大
        if (marker_pub_) {
          visualization_msgs::msg::Marker last_marker = sample_marker;
          last_marker.points.clear();
          last_marker.points.push_back(pt);
          marker_pub_->publish(last_marker);
        }
        return {x, y};
      }
    }
  }

  // 从两棵树生成路径
  void RRTStarConnectPlanner::getPathFromTree1ConnectTree2(
      std::vector<RRTStarConnectNode>& tree1,
      std::vector<RRTStarConnectNode>& tree2,
      RRTStarConnectNode& connect_node,
      std::vector<geometry_msgs::msg::PoseStamped>& plan)
  {
    plan.clear();
    std::vector<std::pair<double, double>> path_points;

    // 从第一棵树获取从起点到连接点的路径
    // 首先找到起始节点
    RRTStarConnectNode start_node = tree1[0];

    // 构建从起点到最后一个节点的路径
    std::vector<std::pair<double, double>> first_tree_path;
    first_tree_path.emplace_back(start_node.x, start_node.y);
    
    std::unordered_map<int, int> node_map; // node_id -> 数组索引的映射
    for (size_t i = 0; i < tree1.size(); ++i) {
      node_map[tree1[i].node_id] = i;
    }
    
    // 从最后一个节点回溯到起点
    RRTStarConnectNode current = tree1.back();
    while (current.parent_id != -1) {
      first_tree_path.emplace_back(current.x, current.y);
      auto it = node_map.find(current.parent_id);
      if (it != node_map.end()) {
        current = tree1[it->second];
      } else {
        break;
      }
    }
    
    // 反转路径，使其从起点指向连接点
    std::reverse(first_tree_path.begin(), first_tree_path.end());
    path_points = first_tree_path;

    // 从第二棵树获取从连接点到终点的路径
    std::vector<std::pair<double, double>> second_tree_path;
    
    // 重置节点映射
    node_map.clear();
    for (size_t i = 0; i < tree2.size(); ++i) {
      node_map[tree2[i].node_id] = i;
    }
    
    // 从连接节点回溯到终点
    current = connect_node;
    second_tree_path.emplace_back(current.x, current.y);
    
    while (current.parent_id != -1) {
      auto it = node_map.find(current.parent_id);
      if (it != node_map.end()) {
        current = tree2[it->second];
        second_tree_path.emplace_back(current.x, current.y);
      } else {
        break;
      }
    }
    
    // 确保包含终点
    if (second_tree_path.empty() || 
        (second_tree_path.back().first != tree2[0].x || 
         second_tree_path.back().second != tree2[0].y)) {
      second_tree_path.emplace_back(tree2[0].x, tree2[0].y);
    }

    // 合并路径，避免连接点重复
    if (!path_points.empty() && !second_tree_path.empty() &&
        fabs(path_points.back().first - second_tree_path.front().first) < 1e-6 &&
        fabs(path_points.back().second - second_tree_path.front().second) < 1e-6) {
      // 跳过第二条路径的第一个点（即连接点）
      path_points.insert(path_points.end(), second_tree_path.begin() + 1, second_tree_path.end());
    } else {
      path_points.insert(path_points.end(), second_tree_path.begin(), second_tree_path.end());
    }

    // 路径优化
    insertPointForPath(path_points, path_point_spacing_);
    optimizationPath(path_points, angle_difference_);
    cutPathPoint(path_points);

    // 转换为ROS路径消息
    for (const auto& pt : path_points)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = node_->now();
      pose.pose.position.x = pt.first;
      pose.pose.position.y = pt.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;  // 初始化为单位四元数
      plan.push_back(pose);
    }

    // 优化路径朝向
    optimizationOrientation(plan);
  }

  // 从一棵树生成路径
  void RRTStarConnectPlanner::getPathFromTree(
      std::vector<RRTStarConnectNode>& tree1,
      std::vector<RRTStarConnectNode>& tree2,
      RRTStarConnectNode& connect_node,
      std::vector<geometry_msgs::msg::PoseStamped>& plan,
      Mode mode)
  {
    plan.clear();
    std::vector<std::pair<double, double>> path_points;
    RRTStarConnectNode current = connect_node;

    // 根据模式选择路径回溯的树
    auto& target_tree = (mode == Mode::TREE1 || mode == Mode::CONNECT2TO1) ? tree1 : tree2;

    // 回溯路径
    while (current.parent_id != -1)
    {
      path_points.emplace_back(current.x, current.y);
      // 查找父节点
      for (const auto& node : target_tree)
      {
        if (node.node_id == current.parent_id)
        {
          current = node;
          break;
        }
      }
    }
    path_points.emplace_back(current.x, current.y);  // 添加起点/终点
    std::reverse(path_points.begin(), path_points.end());

    // 处理反向树的路径（如果需要）
    if (mode == Mode::CONNECT2TO1)
    {
      current = connect_node;
      std::vector<std::pair<double, double>> reverse_points;
      while (current.parent_id != -1)
      {
        reverse_points.emplace_back(current.x, current.y);
        for (const auto& node : tree2)
        {
          if (node.node_id == current.parent_id)
          {
            current = node;
            break;
          }
        }
      }
      reverse_points.emplace_back(current.x, current.y);
      path_points.insert(path_points.end(), reverse_points.begin(), reverse_points.end());
    }

    // 路径优化
    insertPointForPath(path_points, path_point_spacing_);
    optimizationPath(path_points, angle_difference_);
    cutPathPoint(path_points);

    // 转换为ROS路径消息
    for (const auto& pt : path_points)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = node_->now();
      pose.pose.position.x = pt.first;
      pose.pose.position.y = pt.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    // 优化路径朝向
    optimizationOrientation(plan);
  }
// 检查点是否碰撞
bool RRTStarConnectPlanner::isCollision(double x, double y)
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my))
    return true;
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY())
    return true;
  if (costmap_->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return true;
  return false;
}

// 检查点周围是否无障碍物
bool RRTStarConnectPlanner::isAroundFree(double wx, double wy)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my))
      return false;
    if (mx <= 1 || my <= 1 || mx >= costmap_->getSizeInCellsX() - 1 || my >= costmap_->getSizeInCellsY() - 1)
      return false;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        int x = static_cast<int>(mx) + i;
        int y = static_cast<int>(my) + j;
        if (costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          return false;
      }
    }
    return true;
  }

// 判断新节点是否与另一棵树连接
bool RRTStarConnectPlanner::isConnect(RRTStarConnectNode new_node, std::vector<RRTStarConnectNode>& another_tree, std::vector<RRTStarConnectNode>& current_tree, RRTStarConnectNode& connect_node)
  {
    // 查找距离最近的节点
    double min_distance = 1e8;
    size_t min_index = 0;
    for (size_t i = 0; i < another_tree.size(); i++) {
      double dist = calculateHeuristic(new_node.x, new_node.y, another_tree[i].x, another_tree[i].y);
      if (dist < min_distance) {
        min_distance = dist;
        min_index = i;
      }
    }
    
    // 检查是否在连接半径内
    if (min_distance < goal_radius_) {
      // 检查连接路径是否无障碍
      if (obstacleFree(new_node, another_tree[min_index].x, another_tree[min_index].y)) {
        connect_node = another_tree[min_index];
        
        // 创建连接节点，实现两棵树的连接
        RRTStarConnectNode connect_to_other = connect_node;
        connect_to_other.node_id = current_tree.size();
        connect_to_other.parent_id = new_node.node_id;
        connect_to_other.cost = new_node.cost + calculateHeuristic(new_node.x, new_node.y, connect_node.x, connect_node.y);
        
        // 找到更好的父节点
        connect_to_other = chooseParent(new_node, connect_to_other, current_tree);
        current_tree.push_back(connect_to_other);
        
        // 重连接树
        rewire(current_tree, connect_to_other);
        return true;
      }
    }
    return false;
  }

// 在树中找到离随机点最近的节点
RRTStarConnectNode RRTStarConnectPlanner::getNearest(std::vector<RRTStarConnectNode> nodes, std::pair<double, double> p_rand)
  {
    double min_dist = calculateHeuristic(nodes[0].x, nodes[0].y, p_rand.first, p_rand.second);
    size_t min_index = 0;
    for (size_t i = 1; i < nodes.size(); i++) {
      double dist = calculateHeuristic(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second);
      if (dist < min_dist) {
        min_dist = dist;
        min_index = i;
      }
    }
    return nodes[min_index];
  }

// 为新节点选择最优父节点
RRTStarConnectNode RRTStarConnectPlanner::chooseParent(RRTStarConnectNode nn, RRTStarConnectNode newnode, std::vector<RRTStarConnectNode> nodes)
  {
    double dist_nn = calculateHeuristic(nn.x, nn.y, newnode.x, newnode.y);
    for (size_t i = 0; i < nodes.size(); i++) {
      double dist = calculateHeuristic(nodes[i].x, nodes[i].y, newnode.x, newnode.y);
      if (dist < search_radius_ && nodes[i].cost + dist < nn.cost + dist_nn && obstacleFree(nodes[i], newnode.x, newnode.y)) {
        nn = nodes[i];
      }
    }
    newnode.cost = nn.cost + calculateHeuristic(nn.x, nn.y, newnode.x, newnode.y);
    if (!isAroundFree(newnode.x, newnode.y))
      newnode.cost += 0.3;
    newnode.parent_id = nn.node_id;
    return newnode;
  }

// 重连操作
void RRTStarConnectPlanner::rewire(std::vector<RRTStarConnectNode>& nodes, RRTStarConnectNode newnode)
  {
    for (size_t i = 0; i < nodes.size(); i++) {
      RRTStarConnectNode& node = nodes[i];
      if (node.node_id != newnode.parent_id && calculateHeuristic(node.x, node.y, newnode.x, newnode.y) < search_radius_ &&
          newnode.cost + calculateHeuristic(node.x, node.y, newnode.x, newnode.y) < node.cost && obstacleFree(node, newnode.x, newnode.y)) {
        node.parent_id = newnode.node_id;
        node.cost = newnode.cost + calculateHeuristic(node.x, node.y, newnode.x, newnode.y);
        if (!isAroundFree(node.x, node.y))
          node.cost += 0.3;
      }
    }
  }

// 从起点向目标点移动固定步长生成新点
std::pair<double, double> RRTStarConnectPlanner::steer(double x1, double y1, double x2, double y2)
  {
    std::pair<double, double> p_new;
    double dist = calculateHeuristic(x1, y1, x2, y2);
    
    // 如果距离在合理范围内，直接返回目标点
    if (dist <= epsilon_max_ && dist >= epsilon_min_) {
      p_new.first = x2;
      p_new.second = y2;
    } 
    // 如果距离太小，保持原点不变
    else if (dist < epsilon_min_) {
      p_new.first = x1;
      p_new.second = y1;
    } 
    // 如果距离太大，按最大步长前进
    else {
      double theta = atan2(y2 - y1, x2 - x1);
      p_new.first = x1 + epsilon_max_ * cos(theta);
      p_new.second = y1 + epsilon_max_ * sin(theta);
    }
    
    return p_new;
  }

// 检查从最近节点到新点的线段是否无障碍物
bool RRTStarConnectPlanner::obstacleFree(RRTStarConnectNode node_nearest, double px, double py)
  {
    double dist = calculateHeuristic(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution_) {
      return !isCollision(px, py);
    } else {
      int value = static_cast<int>(floor(dist / resolution_));
      double theta = atan2(py - node_nearest.y, px - node_nearest.x);
      for (int n = 1; n <= value; n++) {
        double x = node_nearest.x + n * resolution_ * cos(theta);
        double y = node_nearest.y + n * resolution_ * sin(theta);
        if (isCollision(x, y))
          return false;
      }
      return true;
    }
  }

// 检查点是否在圆内
bool RRTStarConnectPlanner::pointCircleCollision(double x1, double y1, double x2, double y2, double radius)
  {
    double dist = calculateHeuristic(x1, y1, x2, y2);
    return dist <= radius;
  }

// 优化路径点的朝向
void RRTStarConnectPlanner::optimizationOrientation(std::vector<geometry_msgs::msg::PoseStamped>& plan)
  {
    size_t n = plan.size();
    if (n < 2) return;

    // 首先确保所有路径点都有有效的朝向
    for (size_t i = 0; i < n - 1; ++i) {
      double dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
      double dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
      
      // 只有当两点之间有足够的距离时才计算朝向
      // 防止微小距离引起的大方向变化
      if (std::hypot(dx, dy) > 0.001) {
        double angle = atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        plan[i].pose.orientation.x = q.x();
        plan[i].pose.orientation.y = q.y();
        plan[i].pose.orientation.z = q.z();
        plan[i].pose.orientation.w = q.w();
      } else if (i > 0) {
        // 如果两点太近，保持前一个方向
        plan[i].pose.orientation = plan[i-1].pose.orientation;
      }
    }
    
    // 确保最后一个点有正确的朝向
    if (n > 1) {
      // 最后一个点使用前一个点的朝向
      plan[n - 1].pose.orientation = plan[n - 2].pose.orientation;
    }
  }

// 在路径中插入点
void RRTStarConnectPlanner::insertPointForPath(std::vector<std::pair<double, double> >& pathin, double param)
  {
    std::vector<std::pair<double, double> > pathout;
    size_t num = pathin.size();
    if (num < 2) return;
    for (size_t i = 0; i < num - 1; i++) {
      double dx = pathin[i + 1].first - pathin[i].first;
      double dy = pathin[i + 1].second - pathin[i].second;
      double dist = calculateHeuristic(pathin[i + 1].first, pathin[i + 1].second, pathin[i].first, pathin[i].second);
      size_t insert_size = static_cast<size_t>(dist / param);
      for (size_t j = 0; j < insert_size; j++) {
        double ratio = static_cast<double>(j) / static_cast<double>(insert_size);
        double x = pathin[i].first + ratio * dx;
        double y = pathin[i].second + ratio * dy;
        // 插值点障碍物检测，跳过穿越障碍物的点
        if (!isCollision(x, y)) {
          pathout.emplace_back(x, y);
        }
      }
    }
    // 终点一定加入
    if (!isCollision(pathin.back().first, pathin.back().second)) {
      pathout.push_back(pathin.back());
    }
    pathin = pathout;
  }

// 路径优化
int RRTStarConnectPlanner::optimizationPath(std::vector<std::pair<double, double> >& plan, double movement_angle_range)
  {
    if (plan.empty()) return 0;
    size_t num = plan.size();
    bool is_run = false;
    int ci = 0;
    for (ci = 0; ci < 1000; ci++) {
      is_run = false;
      for (size_t i = 1; i < num - 1; i++) {
        double px = plan[i - 1].first;
        double py = plan[i - 1].second;
        double cx = plan[i].first;
        double cy = plan[i].second;
        double nx = plan[i + 1].first;
        double ny = plan[i + 1].second;
        double a_p = normalizeAngle(atan2(cy - py, cx - px), 0, 2 * M_PI);
        double a_n = normalizeAngle(atan2(ny - cy, nx - cx), 0, 2 * M_PI);
        if (std::max(a_p, a_n) - std::min(a_p, a_n) > movement_angle_range) {
          plan[i].first = (px + nx) / 2;
          plan[i].second = (py + ny) / 2;
          is_run = true;
        }
      }
      if (!is_run) return ci;
    }
    return ci;
  }

// 检查两点间的线段是否无障碍物
bool RRTStarConnectPlanner::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
  {
    double dist = calculateHeuristic(p1.first, p1.second, p2.first, p2.second);
    if (dist < resolution_) return true;
    int value = static_cast<int>(floor(dist / resolution_));
    double theta = atan2(p2.second - p1.second, p2.first - p1.first);
    for (int n = 1; n <= value; n++) {
      double x = p1.first + n * resolution_ * cos(theta);
      double y = p1.second + n * resolution_ * sin(theta);
      if (isCollision(x, y)) return false;
    }
    return true;
  }

// 裁剪路径中冗余的点
void RRTStarConnectPlanner::cutPathPoint(std::vector<std::pair<double, double> >& plan)
  {
    size_t current_index = 0;
    size_t check_index = current_index + 2;
    while (current_index < plan.size() - 2) {
      if (isLineFree(plan[current_index], plan[check_index])) {
        if (check_index - current_index - 1 == 1) {
          plan.erase(plan.begin() + static_cast<int>(current_index + 1));
        } else {
          plan.erase(plan.begin() + static_cast<int>(current_index + 1), plan.begin() + static_cast<int>(check_index));
          check_index = current_index + 2;
        }
      } else {
        if (check_index < plan.size() - 1)
          check_index++;
        else {
          current_index++;
          check_index = current_index + 2;
        }
      }
    }
  }

// 发布树的标记
void RRTStarConnectPlanner::pubTreeMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, const visualization_msgs::msg::Marker& marker, int id)
  {
    auto marker_copy = marker;
    marker_copy.id = id;
    marker_pub->publish(marker_copy);
  }
// ...existing code...
  void RRTStarConnectPlanner::configure(
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
    node_, planner_name_ + ".max_nodes_num", rclcpp::ParameterValue(10000));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".plan_time_out", rclcpp::ParameterValue(5.0));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".search_radius", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".goal_radius", rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".epsilon_min", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".epsilon_max", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".path_point_spacing", rclcpp::ParameterValue(0.005));
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".angle_difference", rclcpp::ParameterValue(0.523));  // 约30度(弧度)
    nav2_util::declare_parameter_if_not_declared(
        node_, planner_name_ + ".allow_unknown", rclcpp::ParameterValue(false));

    // 获取参数
    {
      int max_nodes_num_tmp = 10000;
      node_->get_parameter(planner_name_ + ".max_nodes_num", max_nodes_num_tmp);
      max_nodes_num_ = static_cast<size_t>(max_nodes_num_tmp);
    }
    node_->get_parameter(planner_name_ + ".plan_time_out", plan_time_out_);
    node_->get_parameter(planner_name_ + ".search_radius", search_radius_);
    node_->get_parameter(planner_name_ + ".goal_radius", goal_radius_);
    node_->get_parameter(planner_name_ + ".epsilon_min", epsilon_min_);
    node_->get_parameter(planner_name_ + ".epsilon_max", epsilon_max_);
    node_->get_parameter(planner_name_ + ".path_point_spacing", path_point_spacing_);
    node_->get_parameter(planner_name_ + ".angle_difference", angle_difference_);
    node_->get_parameter(planner_name_ + ".allow_unknown", allow_unknown_);

    resolution_ = costmap_->getResolution();

    // 初始化随机数生成器
    gen_ = std::mt19937(rd_());
    real_dist_ = std::uniform_real_distribution<>(0.0, 1.0);

    // 创建RViz可视化发布器（生命周期节点使用create_publisher）
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        planner_name_ + "/tree_markers", rclcpp::QoS(10).transient_local());

    // 初始化可视化标记
    marker_tree_.header.frame_id = global_frame_;
    marker_tree_.ns = "tree_1";
    marker_tree_.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_tree_.action = visualization_msgs::msg::Marker::ADD;
    marker_tree_.scale.x = 0.05;
    marker_tree_.color.r = 1.0;
    marker_tree_.color.g = 0.0;
    marker_tree_.color.b = 0.0;
    marker_tree_.color.a = 1.0;

    marker_tree_2_.header.frame_id = global_frame_;
    marker_tree_2_.ns = "tree_2";
    marker_tree_2_.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_tree_2_.action = visualization_msgs::msg::Marker::ADD;
    marker_tree_2_.scale.x = 0.05;
    marker_tree_2_.color.r = 0.0;
    marker_tree_2_.color.g = 1.0;
    marker_tree_2_.color.b = 0.0;
    marker_tree_2_.color.a = 1.0;

    initialized_ = true;
    RCLCPP_INFO(logger_, "RRT* Connect planner configured successfully");
  }

  void RRTStarConnectPlanner::activate()
  {
    RCLCPP_INFO(logger_, "RRT* Connect planner activated");
  }

  void RRTStarConnectPlanner::deactivate()
  {
    RCLCPP_INFO(logger_, "RRT* Connect planner deactivated");
  }

  void RRTStarConnectPlanner::cleanup()
  {
    marker_pub_.reset();
    initialized_ = false;
    RCLCPP_INFO(logger_, "RRT* Connect planner cleaned up");
  }

  nav_msgs::msg::Path RRTStarConnectPlanner::createPlan(
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

    // 检查坐标系是否一致
    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          logger_,
          "Start and goal must be in global frame (%s), but got start=%s, goal=%s",
          global_frame_.c_str(), start.header.frame_id.c_str(), goal.header.frame_id.c_str());
      return path;
    }

    // 检查起点是否在障碍物内
    if (isCollision(start.pose.position.x, start.pose.position.y))
    {
      RCLCPP_ERROR(logger_, "起点在障碍物内，无法规划路径！");
      return path;
    }

    // 检查终点是否在障碍物内
    if (isCollision(goal.pose.position.x, goal.pose.position.y))
    {
      RCLCPP_ERROR(logger_, "终点在障碍物内，无法规划路径！");
      return path;
    }

    // 初始化树结构
    std::vector<RRTStarConnectNode> tree1, tree2;
  RRTStarConnectNode start_node(start.pose.position.x, start.pose.position.y, 0.0, 0, -1);
  RRTStarConnectNode goal_node(goal.pose.position.x, goal.pose.position.y, 0.0, 0, -1);
    tree1.push_back(start_node);
    tree2.push_back(goal_node);

    // 初始化可视化标记
    marker_tree_.points.clear();
    marker_tree_2_.points.clear();

    std::pair<double, double> p_rand;
    std::pair<double, double> p_new, p_new_2;
  RRTStarConnectNode connect_node1(0.0, 0.0, 0.0, 0, -1);
  RRTStarConnectNode connect_node2(0.0, 0.0, 0.0, 0, -1);
    bool is_connect_to_tree1 = false;
    bool is_connect_to_tree2 = false;
  RRTStarConnectNode node_nearest(0.0, 0.0, 0.0, 0, -1);
    double start_time = node_->now().seconds();

    // 随机采样lambda
    auto sampleRandomPoint = [&]() {
      std::uniform_int_distribution<> dis(0, 9);
      unsigned int rand_nu = dis(gen_);
      if (rand_nu > 1) {
        p_rand = sampleFree();
      } else {
        p_rand.first = goal.pose.position.x;
        p_rand.second = goal.pose.position.y;
      }
    };

    // 扩展树lambda
    auto expandTree = [&](std::vector<RRTStarConnectNode>& currentTree, std::vector<RRTStarConnectNode>& anotherTree,
                         std::pair<double, double>& pNew, visualization_msgs::msg::Marker& marker,
                         bool& isConnected, RRTStarConnectNode& connectNode, const geometry_msgs::msg::PoseStamped& target) {
      // 偶尔直接尝试向目标采样，提高规划效率
      std::uniform_int_distribution<> goal_bias_dist(0, 9);
      if (goal_bias_dist(gen_) <= 1) {
        // 10%概率直接朝向目标点采样
        p_rand.first = target.pose.position.x;
        p_rand.second = target.pose.position.y;
      } else {
        // 90%概率随机采样
        p_rand = sampleFree();
      }
      
      // 获取距离随机点最近的树节点
      node_nearest = getNearest(currentTree, p_rand);
      
      // 从最近点向随机点扩展固定距离
      pNew = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);
      
      // 检查新路径是否无碰撞
      if (obstacleFree(node_nearest, pNew.first, pNew.second)) {
        // 创建新节点
        RRTStarConnectNode newnode(pNew.first, pNew.second, 0.0, currentTree.size(), node_nearest.node_id);
        
        // 为新节点选择最优父节点
        newnode = chooseParent(node_nearest, newnode, currentTree);
        
        // 将新节点添加到树中
        currentTree.push_back(newnode);
        
        // 优化树结构
        rewire(currentTree, newnode);

        // 可视化
        geometry_msgs::msg::Point pt1, pt2;
        if (newnode.parent_id >= 0 && static_cast<size_t>(newnode.parent_id) < currentTree.size()) {
          pt1.x = currentTree[newnode.parent_id].x;
          pt1.y = currentTree[newnode.parent_id].y;
          pt1.z = 0.0;
        } else {
          pt1.x = newnode.x;
          pt1.y = newnode.y;
          pt1.z = 0.0;
        }
        pt2.x = newnode.x;
        pt2.y = newnode.y;
        pt2.z = 0.0;
        marker.points.push_back(pt1);
        marker.points.push_back(pt2);

        if (currentTree.size() % 10 == 0) {
          pubTreeMarker(marker_pub_, marker, &currentTree == &tree1 ? 1 : 2);
        }

        if (isConnect(newnode, anotherTree, currentTree, connectNode)) {
          isConnected = true;
        }
        return true;
      }
      return false;
    };

    // 检查是否到达目标lambda
    auto checkGoalReached = [&](const std::pair<double, double>& p, const geometry_msgs::msg::PoseStamped& target,
                                const std::vector<RRTStarConnectNode>& tree, Mode mode) {
      if (pointCircleCollision(p.first, p.second, target.pose.position.x, target.pose.position.y, goal_radius_)) {
        if (mode == Mode::TREE1) {
          getPathFromTree(const_cast<std::vector<RRTStarConnectNode>&>(tree1), const_cast<std::vector<RRTStarConnectNode>&>(tree2), const_cast<RRTStarConnectNode&>(tree.back()), path.poses, mode);
        } else {
          getPathFromTree(const_cast<std::vector<RRTStarConnectNode>&>(tree2), const_cast<std::vector<RRTStarConnectNode>&>(tree1), const_cast<RRTStarConnectNode&>(tree.back()), path.poses, mode);
        }
        if (!path.poses.empty()) {
          path.poses.front().pose.orientation = start.pose.orientation;
          path.poses.back().pose.orientation = goal.pose.orientation;
        }
        return true;
      }
      return false;
    };

    // 检查树是否连接lambda
    auto checkTreesConnected = [&](bool isConnected, Mode mode) {
      if (isConnected) {
        RRTStarConnectNode& connectNode = mode == Mode::CONNECT1TO2 ? connect_node2 : connect_node1;
        getPathFromTree1ConnectTree2(tree1, tree2, connectNode, path.poses);
        if (!path.poses.empty()) {
          path.poses.front().pose.orientation = start.pose.orientation;
          path.poses.back().pose.orientation = goal.pose.orientation;
        }
        return true;
      }
      return false;
    };

    // 主循环
    while (rclcpp::ok() && tree1.size() + tree2.size() < max_nodes_num_) {
      if ((node_->now().seconds() - start_time) > plan_time_out_) {
        RCLCPP_WARN(logger_, "路径规划超时，未找到可行路径。");
        return path;
      }

      if (expandTree(tree1, tree2, p_new, marker_tree_, is_connect_to_tree2, connect_node2, goal)) {
        if (checkTreesConnected(is_connect_to_tree2, Mode::CONNECT1TO2)) return path;
        if (checkGoalReached(p_new, goal, tree1, Mode::TREE1)) return path;
      }

      p_rand = p_new;
      if (expandTree(tree2, tree1, p_new_2, marker_tree_2_, is_connect_to_tree1, connect_node1, start)) {
        if (checkTreesConnected(is_connect_to_tree1, Mode::CONNECT2TO1)) return path;
        if (checkGoalReached(p_new_2, start, tree2, Mode::TREE2)) return path;
      }
    }
    RCLCPP_WARN(logger_, "路径规划失败，未找到可行路径。");
    return path;
  }
}  // namespace nav2_rrt_star_connect_planner

// 导出插件，确保Nav2能通过pluginlib加载
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_star_connect_planner::RRTStarConnectPlanner, nav2_core::GlobalPlanner)
