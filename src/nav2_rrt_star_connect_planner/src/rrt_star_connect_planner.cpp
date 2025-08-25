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
    
    // 进行路径处理和优化
    // 先进行严格的路径验证和修复
    // 验证路径中的每个点都不在障碍物中
    for (size_t i = 0; i < path_points.size(); /* no increment here */) {
      if (isCollision(path_points[i].first, path_points[i].second)) {
        // 如果点在障碍物中，尝试修复
        bool fixed = false;
        const int search_radius = 5; // 增加搜索半径
        
        // 尝试在周围找到一个安全点，使用更密集的搜索
        for (int r = 1; r <= search_radius && !fixed; r++) {
          // 增加角度搜索精度
          const int angle_steps = 16; 
          for (int a = 0; a < angle_steps && !fixed; a++) {
            double angle = a * 2.0 * M_PI / angle_steps;
            double try_x = path_points[i].first + r * resolution_ * cos(angle);
            double try_y = path_points[i].second + r * resolution_ * sin(angle);
            
            if (!isCollision(try_x, try_y)) {
              // 确保前后连接也是安全的
              bool safe_connection = true;
              if (i > 0) {
                safe_connection = safe_connection && isLineFree(path_points[i-1], std::make_pair(try_x, try_y));
              }
              if (i < path_points.size() - 1) {
                safe_connection = safe_connection && isLineFree(std::make_pair(try_x, try_y), path_points[i+1]);
              }
              
              if (safe_connection) {
                path_points[i].first = try_x;
                path_points[i].second = try_y;
                fixed = true;
              }
            }
          }
        }
        
        // 如果无法修复，移除该点
        if (!fixed) {
          path_points.erase(path_points.begin() + i);
        } else {
          i++;
        }
      } else {
        // 只有在当前点通过验证时才增加索引
        i++;
      }
    }
    
    // 确保路径不为空
    if (path_points.empty()) {
      RCLCPP_ERROR(logger_, "路径验证后为空，无法生成有效路径！");
      return;
    }
    
    // 先插入点以确保路径点间距合理
    insertPointForPath(path_points, path_point_spacing_);
    
    // 验证并修复路径中相邻点之间的连线
    // 多次迭代，确保彻底检查
    for (int iteration = 0; iteration < 3; iteration++) {
      bool path_modified = false;
      
      for (size_t i = 0; i < path_points.size() - 1; i++) {
        if (!isLineFree(path_points[i], path_points[i+1])) {
          path_modified = true;
          // 尝试使用递归二分法找到安全路径
          std::vector<std::pair<double, double>> intermediate_points;
          bool found_safe_path = findSafePath(path_points[i], path_points[i+1], intermediate_points, 0);
          
          // 如果找到安全路径，插入中间点
          if (found_safe_path && !intermediate_points.empty()) {
            path_points.insert(path_points.begin() + i + 1, 
                              intermediate_points.begin(), 
                              intermediate_points.end());
            i += intermediate_points.size(); // 跳过新插入的点
          } else {
            // 如果找不到安全路径，尝试不同策略
            
            // 1. 首先尝试垂直偏移搜索
            double dx = path_points[i+1].first - path_points[i].first;
            double dy = path_points[i+1].second - path_points[i].second;
            double dist = std::hypot(dx, dy);
            
            if (dist > resolution_) {
              // 计算垂直方向
              double nx = -dy / dist;
              double ny = dx / dist;
              
              // 在垂直方向上搜索
              bool found = false;
              const double max_offset = 10.0 * resolution_; // 最大偏移距离
              
              for (double offset = resolution_; offset <= max_offset && !found; offset += resolution_) {
                // 尝试两个方向
                for (double sign : {1.0, -1.0}) {
                  // 中点
                  double mid_x = (path_points[i].first + path_points[i+1].first) / 2.0 + sign * offset * nx;
                  double mid_y = (path_points[i].second + path_points[i+1].second) / 2.0 + sign * offset * ny;
                  
                  if (!isCollision(mid_x, mid_y) && 
                      isLineFree(path_points[i], std::make_pair(mid_x, mid_y)) && 
                      isLineFree(std::make_pair(mid_x, mid_y), path_points[i+1])) {
                    path_points.insert(path_points.begin() + i + 1, std::make_pair(mid_x, mid_y));
                    found = true;
                    break;
                  }
                }
              }
              
              // 2. 如果垂直搜索失败，尝试在连线周围随机采样
              if (!found) {
                const int max_tries = 20; // 最大尝试次数
                std::uniform_real_distribution<> pos_dist(0.3, 0.7); // 位置分布
                std::uniform_real_distribution<> radius_dist(1.0, 8.0); // 半径分布
                std::uniform_real_distribution<> angle_dist(0.0, 2.0 * M_PI); // 角度分布
                
                for (int try_count = 0; try_count < max_tries && !found; try_count++) {
                  // 在连线上随机选择一个点
                  double t = pos_dist(gen_);
                  double base_x = path_points[i].first + t * dx;
                  double base_y = path_points[i].second + t * dy;
                  
                  // 在该点周围随机采样
                  double rand_radius = radius_dist(gen_) * resolution_;
                  double rand_angle = angle_dist(gen_);
                  
                  double try_x = base_x + rand_radius * cos(rand_angle);
                  double try_y = base_y + rand_radius * sin(rand_angle);
                  
                  if (!isCollision(try_x, try_y) && 
                      isLineFree(path_points[i], std::make_pair(try_x, try_y)) && 
                      isLineFree(std::make_pair(try_x, try_y), path_points[i+1])) {
                    path_points.insert(path_points.begin() + i + 1, std::make_pair(try_x, try_y));
                    found = true;
                  }
                }
              }
            }
          }
        }
      }
      
      // 如果这一轮没有修改路径，说明路径已经安全，可以退出循环
      if (!path_modified) {
        break;
      }
    }
    
    // 优化路径，使其更平滑
    optimizationPath(path_points, angle_difference_);
    
    // 再次验证并平滑路径
    if (path_points.size() > 4) {
      smoothPath(path_points);
    }

    // 最后优化和整理路径
    insertPointForPath(path_points, path_point_spacing_);
    optimizationPath(path_points, angle_difference_);
    cutPathPoint(path_points);
    
    // 最终安全检查
    for (size_t i = 0; i < path_points.size() - 1; i++) {
      if (!isLineFree(path_points[i], path_points[i+1])) {
        // 如果还有不安全段，增加中间点
        std::pair<double, double> mid;
        mid.first = (path_points[i].first + path_points[i+1].first) / 2.0;
        mid.second = (path_points[i].second + path_points[i+1].second) / 2.0;
        
        // 如果中点不安全，尝试调整
        if (isCollision(mid.first, mid.second)) {
          const double safety_offset = 2.0 * resolution_;
          // 尝试8个方向
          bool found_safe = false;
          for (int dir = 0; dir < 8 && !found_safe; dir++) {
            double angle = dir * M_PI / 4.0;
            double try_x = mid.first + safety_offset * cos(angle);
            double try_y = mid.second + safety_offset * sin(angle);
            
            if (!isCollision(try_x, try_y)) {
              mid.first = try_x;
              mid.second = try_y;
              found_safe = true;
            }
          }
        }
        
        if (!isCollision(mid.first, mid.second)) {
          path_points.insert(path_points.begin() + i + 1, mid);
          i++; // 跳过插入的点
        }
      }
    }

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
    
    // 验证路径中的每个点都不在障碍物中
    for (size_t i = 0; i < path_points.size(); /* no increment here */) {
      if (isCollision(path_points[i].first, path_points[i].second)) {
        // 如果点在障碍物中，移除该点
        path_points.erase(path_points.begin() + i);
      } else {
        // 只有在当前点通过验证时才增加索引
        i++;
      }
    }
    
    // 验证路径中相邻点之间的连线不穿过障碍物
    for (size_t i = 0; i < path_points.size() - 1; i++) {
      if (!isLineFree(path_points[i], path_points[i+1])) {
        // 如果存在穿过障碍物的线段，在中间插入一个点
        double mid_x = (path_points[i].first + path_points[i+1].first) / 2.0;
        double mid_y = (path_points[i].second + path_points[i+1].second) / 2.0;
        
        // 尝试在周围找一个无碰撞的点
        bool found = false;
        for (int r = 1; r <= 3 && !found; r++) {
          for (int dx = -r; dx <= r && !found; dx++) {
            for (int dy = -r; dy <= r && !found; dy++) {
              if (dx*dx + dy*dy > r*r) continue; // 只在圆形范围内查找
              
              double try_x = mid_x + dx * resolution_;
              double try_y = mid_y + dy * resolution_;
              if (!isCollision(try_x, try_y) && 
                  isLineFree(path_points[i], std::make_pair(try_x, try_y)) && 
                  isLineFree(std::make_pair(try_x, try_y), path_points[i+1])) {
                // 找到了一个可用的点
                path_points.insert(path_points.begin() + i + 1, std::make_pair(try_x, try_y));
                found = true;
              }
            }
          }
        }
        
        // 如果找不到无碰撞点，可能需要重新规划或跳过这个连接
        if (!found) {
          RCLCPP_WARN(logger_, "无法找到连接点，路径可能不安全");
        }
      }
    }

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
    return true;  // 超出地图边界被视为碰撞
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY())
    return true;  // 超出地图边界被视为碰撞
  
  // 直接检查中心点的代价值
  unsigned char center_cost = costmap_->getCost(mx, my);
  if (center_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return true;  // 中心点碰撞，直接返回
  }
  
  // 使用扩展的碰撞检查，检查周围的一个区域
  const int safety_radius = 3;  // 增加安全半径，单位为单元格
  for (int dx = -safety_radius; dx <= safety_radius; dx++) {
    for (int dy = -safety_radius; dy <= safety_radius; dy++) {
      // 跳过中心点，因为已经检查过
      if (dx == 0 && dy == 0) continue;
      
      // 计算距离中心的距离平方（避免开方操作）
      int dist_sq = dx * dx + dy * dy;
      if (dist_sq > safety_radius * safety_radius) continue; // 只检查圆形区域
      
      int cx = static_cast<int>(mx) + dx;
      int cy = static_cast<int>(my) + dy;
      
      // 确保检查点在地图范围内
      if (cx >= 0 && cy >= 0 && 
          cx < static_cast<int>(costmap_->getSizeInCellsX()) && 
          cy < static_cast<int>(costmap_->getSizeInCellsY())) {
        
        unsigned char cost = costmap_->getCost(cx, cy);
        // 使用更低的阈值，增强安全性
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          return true;
        }
      }
    }
  }
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
    // 找到k个最近的点，而不仅仅是一个
    const size_t k_nearest = 5; // 考虑最近的5个点
    std::vector<std::pair<size_t, double>> nearest_indices;
    
    // 查找距离最近的节点
    for (size_t i = 0; i < another_tree.size(); i++) {
      double dist = calculateHeuristic(new_node.x, new_node.y, another_tree[i].x, another_tree[i].y);
      if (dist < goal_radius_ * 1.5) { // 扩大搜索半径，找到更多候选点
        nearest_indices.push_back({i, dist});
      }
    }
    
    // 如果没有找到足够近的点，直接返回失败
    if (nearest_indices.empty()) {
      return false;
    }
    
    // 按距离排序
    std::sort(nearest_indices.begin(), nearest_indices.end(), 
              [](const auto& a, const auto& b) { return a.second < b.second; });
    
    // 只保留最近的k个点或全部（如果少于k个）
    if (nearest_indices.size() > k_nearest) {
      nearest_indices.resize(k_nearest);
    }
    
    // 检查所有近点中是否有可行连接
    std::vector<std::pair<size_t, double>> valid_connections;
    
    for (const auto& [idx, dist] : nearest_indices) {
      // 只考虑在目标半径内的点
      if (dist < goal_radius_) {
        // 检查连接路径是否无障碍
        if (obstacleFree(new_node, another_tree[idx].x, another_tree[idx].y)) {
          // 计算通过该点的总路径代价
          double connection_cost = new_node.cost + dist;
          valid_connections.push_back({idx, connection_cost});
        }
      }
    }
    
    // 如果没有有效连接，返回失败
    if (valid_connections.empty()) {
      return false;
    }
    
    // 选择代价最低的连接点
    std::sort(valid_connections.begin(), valid_connections.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });
    
    size_t best_idx = valid_connections[0].first;
    connect_node = another_tree[best_idx];
    
    // 创建连接节点，实现两棵树的连接
    RRTStarConnectNode connect_to_other = connect_node;
    connect_to_other.node_id = current_tree.size();
    connect_to_other.parent_id = new_node.node_id;
    connect_to_other.cost = new_node.cost + calculateHeuristic(new_node.x, new_node.y, connect_node.x, connect_node.y);
    
    // 再次确认连接路径无碰撞
    std::pair<double, double> p1 = {new_node.x, new_node.y};
    std::pair<double, double> p2 = {connect_node.x, connect_node.y};
    if (!isLineFree(p1, p2)) {
      return false;
    }
    
    // 将连接点添加到当前树
    current_tree.push_back(connect_to_other);
    
    // 通过连接点rewire当前树
    rewire(current_tree, connect_to_other);
    return true;
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
    double best_cost = nn.cost + dist_nn;
    RRTStarConnectNode best_parent = nn;
    
    // 搜索半径内的所有潜在父节点
    std::vector<std::pair<RRTStarConnectNode, double>> candidates;
    candidates.push_back({nn, best_cost});
    
    for (size_t i = 0; i < nodes.size(); i++) {
      double dist = calculateHeuristic(nodes[i].x, nodes[i].y, newnode.x, newnode.y);
      if (dist < search_radius_ && obstacleFree(nodes[i], newnode.x, newnode.y)) {
        double new_cost = nodes[i].cost + dist;
        
        // 如果新代价明显更好，则添加到候选列表
        if (new_cost < best_cost * 1.1) { // 允许10%的代价差异
          candidates.push_back({nodes[i], new_cost});
          
          // 更新最佳候选
          if (new_cost < best_cost) {
            best_cost = new_cost;
            best_parent = nodes[i];
          }
        }
      }
    }
    
    // 如果有多个代价相近的候选者，我们需要额外的评判标准
    if (candidates.size() > 1) {
      // 按代价排序
      std::sort(candidates.begin(), candidates.end(), 
        [](const auto& a, const auto& b) { return a.second < b.second; });
      
      // 只考虑代价接近的候选（例如，在最佳代价的5%范围内）
      std::vector<RRTStarConnectNode> close_candidates;
      for (const auto& c : candidates) {
        if (c.second <= best_cost * 1.05) {
          close_candidates.push_back(c.first);
        }
      }
      
      // 如果有多个接近的候选，使用额外标准
      if (close_candidates.size() > 1) {
        // 选择节点ID较小的父节点，倾向于树的早期节点，产生更稳定的路径
        std::sort(close_candidates.begin(), close_candidates.end(), 
          [](const RRTStarConnectNode& a, const RRTStarConnectNode& b) {
            return a.node_id < b.node_id;
          });
        best_parent = close_candidates[0];
      }
    }
    
    // 设置父节点和代价
    newnode.parent_id = best_parent.node_id;
    newnode.cost = best_parent.cost + calculateHeuristic(best_parent.x, best_parent.y, newnode.x, newnode.y);
    
    // 考虑地形代价
    if (!isAroundFree(newnode.x, newnode.y)) {
      newnode.cost += 0.5; // 增加障碍物附近的代价惩罚
    }
    
    return newnode;
  }

// 重连操作
void RRTStarConnectPlanner::rewire(std::vector<RRTStarConnectNode>& nodes, RRTStarConnectNode newnode)
  {
    // 对于路径稳定性，只对距离新节点较近的节点进行rewire
    double rewire_radius = search_radius_ * 0.8; // 使用更小的rewire半径提高稳定性
    
    for (size_t i = 0; i < nodes.size(); i++) {
      RRTStarConnectNode& node = nodes[i];
      
      // 跳过父节点
      if (node.node_id == newnode.parent_id) {
        continue;
      }
      
      double dist = calculateHeuristic(node.x, node.y, newnode.x, newnode.y);
      
      // 只rewire在rewire_radius范围内的节点
      if (dist < rewire_radius) {
        // 计算通过新节点的路径代价
        double new_cost = newnode.cost + dist;
        
        // 只有当新路径明显优于现有路径时才进行rewire（至少5%的改进）
        if (new_cost < node.cost * 0.95 && obstacleFree(node, newnode.x, newnode.y)) {
          // 防止形成环路
          bool creates_loop = false;
          int parent_id = newnode.parent_id;
          while (parent_id != -1) {
            if (parent_id == node.node_id) {
              creates_loop = true;
              break;
            }
            
            // 查找父节点的父节点
            for (const auto& p_node : nodes) {
              if (p_node.node_id == parent_id) {
                parent_id = p_node.parent_id;
                break;
              }
            }
          }
          
          // 只有不形成环路时才rewire
          if (!creates_loop) {
            node.parent_id = newnode.node_id;
            node.cost = new_cost;
            
            // 考虑地形代价
            if (!isAroundFree(node.x, node.y)) {
              node.cost += 0.5;
            }
          }
        }
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
      
      // 使用自适应步长，避免步长过大导致穿过障碍物
      // 在开阔区域使用较大步长，在障碍物附近使用较小步长
      double adaptive_epsilon = epsilon_max_;
      
      // 检查原点周围环境
      if (!isAroundFree(x1, y1)) {
        // 在障碍物附近，使用较小的步长
        adaptive_epsilon = std::max(epsilon_min_, epsilon_max_ / 2.0);
      }
      
      // 尝试不同步长的点，避免穿过障碍物
      const int num_tries = 3;
      bool found = false;
      
      for (int i = 0; i < num_tries && !found; i++) {
        // 第一次尝试使用自适应步长，之后逐步减小
        double current_epsilon = adaptive_epsilon * (1.0 - 0.2 * i);
        double test_x = x1 + current_epsilon * cos(theta);
        double test_y = y1 + current_epsilon * sin(theta);
        
        // 确保新点不在障碍物中
        if (!isCollision(test_x, test_y)) {
          p_new.first = test_x;
          p_new.second = test_y;
          found = true;
        }
      }
      
      // 如果所有尝试都失败，回退到起点
      if (!found) {
        p_new.first = x1;
        p_new.second = y1;
      }
    }
    
    return p_new;
  }

// 检查从最近节点到新点的线段是否无障碍物
bool RRTStarConnectPlanner::obstacleFree(RRTStarConnectNode node_nearest, double px, double py)
  {
    // 首先检查端点是否无碰撞
    if (isCollision(px, py)) {
      return false;
    }
    
    double dist = calculateHeuristic(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution_) {
      // 对于短距离，检查中点
      double mid_x = (node_nearest.x + px) / 2.0;
      double mid_y = (node_nearest.y + py) / 2.0;
      return !isCollision(mid_x, mid_y);
    } else {
      // 使用更小的分辨率进行更密集的检查
      double check_resolution = resolution_ / 2.0;
      int value = static_cast<int>(ceil(dist / check_resolution));
      
      for (int n = 0; n <= value; n++) {
        double ratio = static_cast<double>(n) / static_cast<double>(value);
        double x = node_nearest.x + ratio * (px - node_nearest.x);
        double y = node_nearest.y + ratio * (py - node_nearest.y);
        
        if (isCollision(x, y)) {
          return false;
        }
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
    
    // 始终保留原始路径的起点
    pathout.push_back(pathin[0]);
    
    for (size_t i = 0; i < num - 1; i++) {
      double dx = pathin[i + 1].first - pathin[i].first;
      double dy = pathin[i + 1].second - pathin[i].second;
      double dist = calculateHeuristic(pathin[i + 1].first, pathin[i + 1].second, pathin[i].first, pathin[i].second);
      
      // 计算需要插入的点数量
      size_t insert_size = static_cast<size_t>(dist / param);
      
      // 如果两点之间距离太远，插入中间点
      if (insert_size > 0) {
        // 检查原始两点间的线段是否穿过障碍物
        bool line_has_collision = !isLineFree(pathin[i], pathin[i+1]);
        
        for (size_t j = 1; j < insert_size; j++) {
          double ratio = static_cast<double>(j) / static_cast<double>(insert_size);
          double x = pathin[i].first + ratio * dx;
          double y = pathin[i].second + ratio * dy;
          
          // 检查插入点是否无碰撞
          if (!isCollision(x, y)) {
            // 如果线段有碰撞，需要验证每个插入点与前一点的连线是否穿过障碍物
            if (!line_has_collision || 
                (pathout.size() > 0 && isLineFree(pathout.back(), std::make_pair(x, y)))) {
              pathout.emplace_back(x, y);
            } else {
              // 如果连线穿过障碍物，尝试找一个无碰撞的替代点
              for (int offset_x = -1; offset_x <= 1; offset_x++) {
                for (int offset_y = -1; offset_y <= 1; offset_y++) {
                  if (offset_x == 0 && offset_y == 0) continue;
                  
                  double try_x = x + offset_x * resolution_;
                  double try_y = y + offset_y * resolution_;
                  
                  if (!isCollision(try_x, try_y) && 
                      pathout.size() > 0 && 
                      isLineFree(pathout.back(), std::make_pair(try_x, try_y))) {
                    pathout.emplace_back(try_x, try_y);
                    break;
                  }
                }
                if (pathout.size() > 0 && 
                    pathout.back().first != x && 
                    pathout.back().second != y) break;
              }
            }
          }
        }
      }
      
      // 确保添加下一个原始路径点（如果它不在障碍物中）
      if (!isCollision(pathin[i + 1].first, pathin[i + 1].second)) {
        // 检查与前一点的连线是否无碰撞
        if (pathout.empty() || isLineFree(pathout.back(), pathin[i + 1])) {
          pathout.push_back(pathin[i + 1]);
        } else {
          // 如果连线有碰撞，寻找替代点
          double best_x = pathin[i + 1].first;
          double best_y = pathin[i + 1].second;
          bool found = false;
          
          for (int r = 1; r <= 3; r++) {
            for (int dx = -r; dx <= r; dx++) {
              for (int dy = -r; dy <= r; dy++) {
                if (dx*dx + dy*dy > r*r) continue; // 只在圆形范围内查找
                
                double try_x = pathin[i + 1].first + dx * resolution_;
                double try_y = pathin[i + 1].second + dy * resolution_;
                
                if (!isCollision(try_x, try_y) && isLineFree(pathout.back(), std::make_pair(try_x, try_y))) {
                  best_x = try_x;
                  best_y = try_y;
                  found = true;
                  break;
                }
              }
              if (found) break;
            }
            if (found) break;
          }
          pathout.emplace_back(best_x, best_y);
        }
      }
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
          // 计算平滑点
          double new_x = (px + nx) / 2;
          double new_y = (py + ny) / 2;
          
          // 检查该点是否无碰撞
          if (!isCollision(new_x, new_y)) {
            // 检查到前后点的连线是否无碰撞
            std::pair<double, double> new_p = std::make_pair(new_x, new_y);
            std::pair<double, double> prev_p = std::make_pair(px, py);
            std::pair<double, double> next_p = std::make_pair(nx, ny);
            
            if (isLineFree(prev_p, new_p) && isLineFree(new_p, next_p)) {
              plan[i].first = new_x;
              plan[i].second = new_y;
              is_run = true;
            }
          }
        }
      }
      if (!is_run) return ci;
    }
    return ci;
  }

// 检查两点间的线段是否无障碍物
bool RRTStarConnectPlanner::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
  {
    // 首先检查两个端点
    if (isCollision(p1.first, p1.second) || isCollision(p2.first, p2.second)) {
      return false;
    }
    
    double dist = calculateHeuristic(p1.first, p1.second, p2.first, p2.second);
    if (dist < resolution_ / 8.0) {
      // 距离非常小，但仍需进行中点检查以确保安全
      double mid_x = (p1.first + p2.first) / 2.0;
      double mid_y = (p1.second + p2.second) / 2.0;
      return !isCollision(mid_x, mid_y);
    }
    
    // 使用更密集的检查点，取resolution_的1/6，确保不会漏检
    double check_resolution = resolution_ / 6.0;
    int num_checks = std::max(20, static_cast<int>(ceil(dist / check_resolution)));
    
    // 首先检查均匀分布的点
    for (int n = 1; n < num_checks; n++) {
      double ratio = static_cast<double>(n) / static_cast<double>(num_checks);
      double x = p1.first + ratio * (p2.first - p1.first);
      double y = p1.second + ratio * (p2.second - p1.second);
      
      if (isCollision(x, y)) {
        return false;
      }
    }
    
    // 检查关键区域 - 特别是在线段的中间部分进行更密集的检查
    const int detailed_checks = 10;
    std::uniform_real_distribution<> mid_section_dist(0.3, 0.7); // 中间区域
    for (int i = 0; i < detailed_checks; i++) {
      double ratio = mid_section_dist(gen_);
      double x = p1.first + ratio * (p2.first - p1.first);
      double y = p1.second + ratio * (p2.second - p1.second);
      
      if (isCollision(x, y)) {
        return false;
      }
    }
    
    // 额外检查一些随机位置点，覆盖整个线段
    std::uniform_real_distribution<> extra_check_dist(0.05, 0.95);
    for (int i = 0; i < 8; i++) {
      double ratio = extra_check_dist(gen_);
      double x = p1.first + ratio * (p2.first - p1.first);
      double y = p1.second + ratio * (p2.second - p1.second);
      
      if (isCollision(x, y)) {
        return false;
      }
    }
    
    return true;
  }

// 裁剪路径中冗余的点
void RRTStarConnectPlanner::cutPathPoint(std::vector<std::pair<double, double> >& plan)
  {
    // 如果路径太短，不需要裁剪
    if (plan.size() < 3) {
      return;
    }
    
    size_t current_index = 0;
    size_t check_index = current_index + 2;
    
    while (current_index < plan.size() - 2 && check_index < plan.size()) {
      // 检查当前点和检查点之间的线段是否无碰撞
      if (isLineFree(plan[current_index], plan[check_index])) {
        // 还需要检查沿途的每个中间点到线段的距离，确保不会切割太多
        bool can_cut = true;
        
        // 计算current_index到check_index之间的直线参数
        double dx = plan[check_index].first - plan[current_index].first;
        double dy = plan[check_index].second - plan[current_index].second;
        double line_length = std::hypot(dx, dy);
        
        // 如果线段很短，直接进行裁剪
        if (line_length < resolution_ * 3.0) {
          // 裁剪单个点
          if (check_index - current_index - 1 == 1) {
            plan.erase(plan.begin() + static_cast<int>(current_index + 1));
          } else {
            // 裁剪多个点
            plan.erase(plan.begin() + static_cast<int>(current_index + 1), plan.begin() + static_cast<int>(check_index));
            check_index = current_index + 2;
            if (check_index >= plan.size()) {
              break;
            }
          }
        } else {
          // 增加检查点
          if (check_index < plan.size() - 1) {
            check_index++;
          } else {
            current_index++;
            check_index = current_index + 2;
            if (check_index >= plan.size()) {
              break;
            }
          }
        }
      } else {
        // 有碰撞，不能裁剪
        if (check_index < plan.size() - 1) {
          check_index++;
        } else {
          current_index++;
          check_index = current_index + 2;
          if (check_index >= plan.size()) {
            break;
          }
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

// 递归寻找避障安全路径
bool RRTStarConnectPlanner::findSafePath(
  const std::pair<double, double>& start,
  const std::pair<double, double>& end,
  std::vector<std::pair<double, double>>& path,
  int depth)
{
  // 防止无限递归
  const int max_depth = 10; // 增加最大递归深度
  if (depth > max_depth) {
    return false;
  }
  
  // 计算起点和终点之间的距离
  double dist = calculateHeuristic(start.first, start.second, end.first, end.second);
  
  // 如果距离很小且路径安全，直接返回
  if (dist < resolution_ && isLineFree(start, end)) {
    path.push_back(end);
    return true;
  }
  
  // 检查直接路径是否安全
  if (isLineFree(start, end)) {
    // 对于较长的路径，即使直线安全也在中间插入一个点，以确保路径质量
    if (dist > resolution_ * 5) {
      std::pair<double, double> mid;
      mid.first = (start.first + end.first) / 2.0;
      mid.second = (start.second + end.second) / 2.0;
      
      // 确保中点安全
      if (!isCollision(mid.first, mid.second)) {
        path.push_back(mid);
        path.push_back(end);
        return true;
      }
    }
    
    path.push_back(end);
    return true;
  }
  
  // 如果直接路径不安全，计算中点
  std::pair<double, double> mid;
  mid.first = (start.first + end.first) / 2.0;
  mid.second = (start.second + end.second) / 2.0;
  
  // 检查中点是否在障碍物中
  if (isCollision(mid.first, mid.second)) {
    bool found_safe = false;
    double r_start = resolution_ / 2.0; // 更精细的起始半径
    
    // 使用更密集的角度采样，增加搜索点的数量
    const int angle_steps = 16; // 增加角度采样数
    
    // 在中点周围以不断扩大的半径寻找安全点
    for (double radius = r_start; radius <= r_start * 10 && !found_safe; radius += r_start) {
      for (int angle_step = 0; angle_step < angle_steps && !found_safe; angle_step++) {
        double angle = angle_step * 2.0 * M_PI / angle_steps;
        double try_x = mid.first + radius * cos(angle);
        double try_y = mid.second + radius * sin(angle);
        
        if (!isCollision(try_x, try_y) && 
            isLineFree(start, std::make_pair(try_x, try_y)) && 
            isLineFree(std::make_pair(try_x, try_y), end)) {
          // 只有当到起点和终点的路径都安全时，才使用该点
          mid.first = try_x;
          mid.second = try_y;
          found_safe = true;
        }
      }
    }
    
    // 如果找不到安全点，尝试更大范围的搜索
    if (!found_safe) {
      // 尝试在起点和终点之间的垂直方向上寻找安全点
      double dx = end.first - start.first;
      double dy = end.second - start.second;
      
      // 计算垂直方向上的单位向量
      double length = std::hypot(dx, dy);
      if (length > 0) {
        double perp_x = -dy / length;
        double perp_y = dx / length;
        
        // 在垂直方向上搜索
        for (double offset = resolution_; offset <= resolution_ * 15 && !found_safe; offset += resolution_) {
          // 尝试两个方向
          for (int sign : {1, -1}) {
            double try_x = mid.first + sign * offset * perp_x;
            double try_y = mid.second + sign * offset * perp_y;
            
            if (!isCollision(try_x, try_y) && 
                isLineFree(start, std::make_pair(try_x, try_y)) && 
                isLineFree(std::make_pair(try_x, try_y), end)) {
              mid.first = try_x;
              mid.second = try_y;
              found_safe = true;
              break;
            }
          }
        }
      }
      
      // 如果仍然找不到安全点，放弃该路径
      if (!found_safe) {
        return false;
      }
    }
  }
  
  // 递归处理起点到中点，和中点到终点的路径
  std::vector<std::pair<double, double>> left_path, right_path;
  bool left_ok = findSafePath(start, mid, left_path, depth + 1);
  bool right_ok = findSafePath(mid, end, right_path, depth + 1);
  
  // 合并路径
  if (left_ok && right_ok) {
    path.insert(path.end(), left_path.begin(), left_path.end());
    path.insert(path.end(), right_path.begin(), right_path.end());
    return true;
  } else if (left_ok) {
    path.insert(path.end(), left_path.begin(), left_path.end());
    path.push_back(mid);
    return true;
  } else if (right_ok) {
    path.push_back(mid);
    path.insert(path.end(), right_path.begin(), right_path.end());
    return true;
  }
  
  // 如果两侧都失败，则整体失败
  return false;
}

// 平滑路径
void RRTStarConnectPlanner::smoothPath(std::vector<std::pair<double, double>>& path)
{
  if (path.size() <= 3) {
    return; // 点太少无需平滑
  }
  
  std::vector<std::pair<double, double>> smoothed_path = path;
  const double initial_weight = 0.4;   // 初始平滑权重，较小以确保安全
  const double weight_decay = 0.8;     // 每次迭代后权重衰减
  const int iterations = 3;            // 减少迭代次数，避免过度平滑
  double current_weight = initial_weight;
  
  // 多次迭代以获得更平滑的路径
  for (int iter = 0; iter < iterations; iter++) {
    bool any_change = false; // 跟踪本次迭代是否有任何改变
    
    // 保持第一个和最后一个点不变
    for (size_t i = 1; i < path.size() - 1; i++) {
      // 计算当前点、前一个点和后一个点的加权平均
      double new_x = path[i].first * (1 - current_weight) + 
                    (path[i-1].first + path[i+1].first) / 2.0 * current_weight;
      double new_y = path[i].second * (1 - current_weight) + 
                    (path[i-1].second + path[i+1].second) / 2.0 * current_weight;
      
      // 先检查新点是否在障碍物内
      if (isCollision(new_x, new_y)) {
        continue; // 新点在障碍物中，跳过
      }
      
      // 计算新点与原点的距离
      double move_dist = std::hypot(new_x - path[i].first, new_y - path[i].second);
      
      // 如果移动距离太大，可能导致路径不稳定，则限制移动距离
      const double max_move_dist = resolution_ * 1.5;
      if (move_dist > max_move_dist) {
        // 按比例缩小移动距离
        double scale = max_move_dist / move_dist;
        new_x = path[i].first + (new_x - path[i].first) * scale;
        new_y = path[i].second + (new_y - path[i].second) * scale;
      }
      
      // 严格检查新点与前后点的连线是否安全
      if (!isCollision(new_x, new_y) && 
          isLineFree({path[i-1].first, path[i-1].second}, {new_x, new_y}) &&
          isLineFree({new_x, new_y}, {path[i+1].first, path[i+1].second})) {
        
        // 额外的安全检查 - 验证与当前路径的偏离不会太大
        smoothed_path[i].first = new_x;
        smoothed_path[i].second = new_y;
        any_change = true;
      }
    }
    
    // 更新原始路径以便下一次迭代
    path = smoothed_path;
    
    // 如果没有任何改变，提前结束迭代
    if (!any_change) {
      break;
    }
    
    // 减小权重，使后续迭代的平滑效果更轻微
    current_weight *= weight_decay;
  }
  
  // 最后检查一遍整个路径，确保没有穿过障碍物的线段
  for (size_t i = 0; i < path.size() - 1; i++) {
    if (!isLineFree({path[i].first, path[i].second}, {path[i+1].first, path[i+1].second})) {
      // 如果发现不安全的线段，尝试修复
      std::vector<std::pair<double, double>> safe_segment;
      if (findSafePath({path[i].first, path[i].second}, {path[i+1].first, path[i+1].second}, safe_segment, 0) && 
          !safe_segment.empty()) {
        // 插入安全点
        path.insert(path.begin() + i + 1, safe_segment.begin(), safe_segment.end());
        i += safe_segment.size(); // 跳过新插入的点
      }
    }
  }
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
      // 根据当前策略选择采样点
      std::uniform_int_distribution<> goal_bias_dist(0, 9);
      std::uniform_int_distribution<> strategy_dist(0, 4);
      int strategy = strategy_dist(gen_);
      
      if (goal_bias_dist(gen_) <= 2) {
        // 增加到20%的概率直接朝向目标点采样，提高规划效率
        p_rand.first = target.pose.position.x;
        p_rand.second = target.pose.position.y;
      } else if (strategy == 0 && !currentTree.empty() && currentTree[0].node_id == 0) {
        // 偶尔尝试朝向起点方向采样，确保路径的连通性
        p_rand.first = currentTree[0].x;
        p_rand.second = currentTree[0].y;
      } else if (strategy == 1 && currentTree.size() > 10) {
        // 偶尔尝试在已有路径的附近采样，增加路径稠密度
        size_t random_idx = std::uniform_int_distribution<size_t>(0, currentTree.size()-1)(gen_);
        double angle = std::uniform_real_distribution<double>(0, 2*M_PI)(gen_);
        double radius = std::uniform_real_distribution<double>(resolution_, 3*resolution_)(gen_);
        p_rand.first = currentTree[random_idx].x + radius * cos(angle);
        p_rand.second = currentTree[random_idx].y + radius * sin(angle);
      } else {
        // 随机采样
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
        
        // 为新节点选择最优父节点，加入额外代价因素避免摇摆
        newnode = chooseParent(node_nearest, newnode, currentTree);
        
        // 只有当新节点带来显著改善时才添加它
        if (newnode.cost < 1e8) { // 确保不是无穷大代价
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
