#ifndef NAV2_RRT_STAR_CONNECT_PLANNER__RRT_STAR_CONNECT_PLANNER_HPP_
#define NAV2_RRT_STAR_CONNECT_PLANNER__RRT_STAR_CONNECT_PLANNER_HPP_


#include <memory>                // 智能指针
#include <cmath>                 // 数学函数
#include <string>                // 字符串
#include <vector>                // 向量容器
#include <utility>               // std::pair
#include <random>                // 随机数
#include <unordered_map>         // 哈希表
#include <unordered_set>         // 哈希集合

#include "rclcpp_lifecycle/lifecycle_node.hpp"  // ROS2生命周期节点
#include "nav2_core/global_planner.hpp"         // nav2全局规划器接口
#include "nav2_costmap_2d/costmap_2d_ros.hpp"   // 代价地图
#include "tf2_ros/buffer.h"                     // TF变换
#include "geometry_msgs/msg/pose_stamped.hpp"   // 位姿消息
#include "nav_msgs/msg/path.hpp"                // 路径消息
#include "visualization_msgs/msg/marker.hpp"    // RViz可视化标记
#include "rclcpp/publisher.hpp"                 // ROS2发布器


namespace nav2_rrt_star_connect_planner
{

  /**
   * @brief 路径生成模式枚举
   */
  enum class Mode
  {
    TREE1 = 1,        ///< 使用第一棵树生成路径
    TREE2 = 2,        ///< 使用第二棵树生成路径
    CONNECT1TO2 = 3,  ///< 连接第一棵树到第二棵树生成路径
    CONNECT2TO1 = 4,  ///< 连接第二棵树到第一棵树生成路径
  };


  /**
   * @brief RRT*节点结构体
   */
  struct RRTStarConnectNode
  {
    double x;   ///< 节点x坐标
    double y;   ///< 节点y坐标
    double cost; ///< 从起点到当前节点的总代价
    int node_id; ///< 节点唯一编号
    int parent_id; ///< 父节点编号
  /**
   * @brief 构造函数
   * @param x_ x坐标
   * @param y_ y坐标
   * @param c_ 代价
   * @param node_id_ 节点编号
   * @param parent_id_ 父节点编号
   */
  RRTStarConnectNode(double x_, double y_, double c_, int node_id_, int parent_id_)
    : x(x_), y(y_), cost(c_), node_id(node_id_), parent_id(parent_id_) {}

    /**
     * @brief 判断节点是否相等
     */
    bool operator==(const RRTStarConnectNode& node) const
    {
      return (fabs(x - node.x) < 0.0001) && (fabs(y - node.y) < 0.0001) && (node_id == node.node_id) && (parent_id == node.parent_id) && (fabs(cost - node.cost) < 0.0001);
    }

    /**
     * @brief 判断节点是否不等
     */
    bool operator!=(const RRTStarConnectNode& node) const
    {
      return !(*this == node);
    }
  };

  /**
   * @brief 归一化角度到指定区间[min, max)
   *
   * 此函数将输入角度归一化，使其结果落在[min, max)区间内。
   * 支持正负角度，自动进行取模处理，确保角度在指定范围内循环。
   *
   * @param val 待归一化的角度值
   * @param min 归一化区间下界（包含）
   * @param max 归一化区间上界（不包含）
   * @return 归一化后的角度，范围在[min, max)
   */
  double normalizeAngle(double val, double min, double max)
  {
    double normalization = 0.0;

    if (val >= min)
      normalization = min + fmod((val - min), (max - min));
    else
      normalization = max - fmod((min - val), (max - min));

    return normalization;
  }

  /**
   * @brief RRT* Connect全局路径规划器
   *
   * 该类实现了基于RRT* Connect算法的全局路径规划器，继承自nav2_core::GlobalPlanner接口。
   * 支持在ROS2导航框架中作为插件加载，能够在已知地图环境下为移动机器人生成无碰撞的最优路径。
   * 主要功能包括：路径生成、路径优化、碰撞检测、可视化等。
   */
  class RRTStarConnectPlanner : public nav2_core::GlobalPlanner
  {
  public:
    /**
     * @brief 构造函数
     */
    RRTStarConnectPlanner() = default;

    /**
     * @brief 析构函数
     */
    ~RRTStarConnectPlanner() = default;

    // 禁用拷贝构造和赋值，防止不必要的复制
    RRTStarConnectPlanner(const RRTStarConnectPlanner&) = delete;
    RRTStarConnectPlanner& operator=(const RRTStarConnectPlanner&) = delete;

    friend class RRTStarConnectPlannerTest;

    /**
     * @brief 配置规划器
     * @param parent 父节点
     * @param name 规划器名称
     * @param tf TF变换缓冲区
     * @param costmap_ros 代价地图ROS包装器
     */
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                  std::string name,
                  std::shared_ptr<tf2_ros::Buffer> tf,
                  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief 清理资源
     */
    void cleanup() override;

    /**
     * @brief 激活规划器
     */
    void activate() override;

    /**
     * @brief 失活规划器
     */
    void deactivate() override;

    /**
     * @brief 生成路径
     * @param start 起点
     * @param goal 终点
     * @return 路径
     */
    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal) override;

    /**
     * @brief 从两棵树生成路径
     * @param tree1 第一棵树
     * @param tree2 第二棵树
     * @param connect_node 连接节点
     * @param plan 输出路径
     */
    void getPathFromTree1ConnectTree2(std::vector<RRTStarConnectNode>& tree1,
                                      std::vector<RRTStarConnectNode>& tree2,
                                      RRTStarConnectNode& connect_node,
                                      std::vector<geometry_msgs::msg::PoseStamped>& plan);

    /**
     * @brief 从一棵树生成路径
     * @param tree1 当前树
     * @param tree2 另一棵树
     * @param connect_node 连接节点
     * @param plan 输出路径
     * @param mode 路径生成模式
     */
    void getPathFromTree(std::vector<RRTStarConnectNode>& tree1,
                         std::vector<RRTStarConnectNode>& tree2,
                         RRTStarConnectNode& connect_node,
                         std::vector<geometry_msgs::msg::PoseStamped>& plan,
                         Mode mode);

    /**
     * @brief 计算启发式代价
     * @param x1 起点x
     * @param y1 起点y
     * @param x2 终点x
     * @param y2 终点y
     * @return 欧氏距离
     */
    double calculateHeuristic(double x1, double y1, double x2, double y2) const;

    /**
     * @brief 在自由空间采样点
     * @return 采样点坐标
     */
    std::pair<double, double> sampleFree();

    /**
     * @brief 检查点是否碰撞
     * @param x x坐标
     * @param y y坐标
     * @return 是否碰撞
     */
    bool isCollision(double x, double y);

    /**
     * @brief 检查点周围是否无障碍物
     * @param wx 世界坐标x
     * @param wy 世界坐标y
     * @return 是否无障碍物
     */
    bool isAroundFree(double wx, double wy);

    /**
     * @brief 判断新节点是否与另一棵树连接
     * @param new_node 新节点
     * @param another_tree 另一棵树
     * @param current_tree 当前树
     * @param connect_node 输出连接节点
     * @return 是否连接
     */
    bool isConnect(RRTStarConnectNode new_node, std::vector<RRTStarConnectNode>& another_tree, std::vector<RRTStarConnectNode>& current_tree, RRTStarConnectNode& connect_node);

    /**
     * @brief 在树中找到离随机点最近的节点
     * @param nodes 节点集合
     * @param p_rand 随机点
     * @return 最近节点
     */
    RRTStarConnectNode getNearest(std::vector<RRTStarConnectNode> nodes, std::pair<double, double> p_rand);

    /**
     * @brief 为新节点选择最优父节点（最小化代价）
     * @param nn 最近节点
     * @param newnode 新节点
     * @param nodes 节点集合
     * @return 最优父节点
     */
    RRTStarConnectNode chooseParent(RRTStarConnectNode nn, RRTStarConnectNode newnode, std::vector<RRTStarConnectNode> nodes);

    /**
     * @brief 重连操作（RRT*核心优化）：更新周围节点的父节点以减小代价
     * @param nodes 节点集合
     * @param newnode 新节点
     */
    void rewire(std::vector<RRTStarConnectNode>& nodes, RRTStarConnectNode newnode);

    /**
     * @brief 从起点向目标点移动固定步长生成新点
     * @param x1 起点x
     * @param y1 起点y
     * @param x2 目标x
     * @param y2 目标y
     * @return 新点坐标
     */
    std::pair<double, double> steer(double x1, double y1, double x2, double y2);

    /**
     * @brief 检查从最近节点到新点的线段是否无障碍物
     * @param node_nearest 最近节点
     * @param px 新点x
     * @param py 新点y
     * @return 是否无障碍物
     */
    bool obstacleFree(RRTStarConnectNode node_nearest, double px, double py);

    /**
     * @brief 检查点(x1,y1)是否在以(x2,y2)为中心、radius为半径的圆内
     * @param x1 点1x
     * @param y1 点1y
     * @param x2 圆心x
     * @param y2 圆心y
     * @param radius 半径
     * @return 是否在圆内
     */
    bool pointCircleCollision(double x1, double y1, double x2, double y2, double radius);

    /**
     * @brief 优化路径点的朝向
     * @param plan 路径点集合
     */
    void optimizationOrientation(std::vector<geometry_msgs::msg::PoseStamped>& plan);

    /**
     * @brief 在路径中插入点（控制路径点密度）
     * @param pathin 输入路径
     * @param param 插值参数
     */
    void insertPointForPath(std::vector<std::pair<double, double> >& pathin, double param);

    /**
     * @brief 路径优化（如去除冗余点，使路径更平滑）
     * @param plan 路径点集合
     * @param movement_angle_range 允许的运动角度范围
     * @return 优化后路径点数
     */
    int optimizationPath(std::vector<std::pair<double, double> >& plan, double movement_angle_range = M_PI / 4);

    /**
     * @brief 检查两点间的线段是否无障碍物
     * @param p1 点1
     * @param p2 点2
     * @return 是否无障碍物
     */
    bool isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2);

    /**
     * @brief 裁剪路径中冗余的点（简化路径）
     * @param plan 路径点集合
     */
    void cutPathPoint(std::vector<std::pair<double, double> >& plan);

    /**
     * @brief 发布树的标记（用于RViz可视化）
     * @param marker_pub 标记发布器
     * @param marker 标记消息
     * @param id 标记ID
     */
    void pubTreeMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, const visualization_msgs::msg::Marker& marker, int id);

  private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;             ///< 节点指针
    std::shared_ptr<tf2_ros::Buffer> tf_;                         ///< TF缓冲区
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  ///< 代价地图ROS包装器
    nav2_costmap_2d::Costmap2D* costmap_;                         ///< 代价地图指针
    rclcpp::Logger logger_{rclcpp::get_logger("RRTStarConnectPlanner")};  ///< 日志
    rclcpp::Clock::SharedPtr clock_;                              ///< 时钟
    std::string planner_name_;                                    ///< 规划器名称
    std::string global_frame_;                                    ///< 全局坐标系名称

  visualization_msgs::msg::Marker marker_tree_;                 ///< 第一棵树可视化标记
  visualization_msgs::msg::Marker marker_tree_2_;               ///< 第二棵树可视化标记
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; ///< RViz标记发布器

    size_t max_nodes_num_;    ///< 最大节点数
    double plan_time_out_;    ///< 规划超时时间
    double search_radius_;    ///< 搜索半径
    double goal_radius_;      ///< 目标区域半径
    double epsilon_min_;      ///< 最小步长
    double epsilon_max_;      ///< 最大步长

    double path_point_spacing_; ///< 路径点间距
    double angle_difference_;   ///< 允许的角度差

    double resolution_;         ///< 地图分辨率
    bool initialized_;          ///< 是否已初始化
    bool allow_unknown_;

  std::random_device rd_;     ///< 随机数设备
  std::mt19937 gen_;          ///< 随机数生成器
  std::uniform_int_distribution<> int_dist_;   ///< 整数分布
  std::uniform_real_distribution<> real_dist_; ///< 浮点数分布
  };
}  // namespace nav2_rrt_star_connect_planner

#endif  // NAV2_RRT_STAR_CONNECT_PLANNER_HPP_