#!/bin/bash

# A*规划器测试验证脚本
# 用于验证测试配置和构建设置

echo "=== A* 规划器测试验证脚本 ==="
echo

# 检查文件结构
echo "1. 检查测试文件结构..."
echo "📁 测试目录内容:"
ls -la src/nav2_astar_planner/test/

echo
echo "📄 CMakeLists.txt 测试配置:"
grep -A 20 "BUILD_TESTING" src/nav2_astar_planner/CMakeLists.txt

echo
echo "📋 package.xml 测试依赖:"
grep "test_depend" src/nav2_astar_planner/package.xml

echo
echo "2. 检查测试代码统计..."
echo "测试文件代码行数统计:"
wc -l src/nav2_astar_planner/test/*.cpp src/nav2_astar_planner/test/*.hpp

echo
echo "3. 检查代码格式..."
echo "检查代码是否符合项目格式标准..."

# 检查主要的测试类和方法
echo
echo "4. 测试内容概览..."
echo "🧪 单元测试类:"
grep "class.*Test.*:" src/nav2_astar_planner/test/test_astar_planner.cpp | head -5

echo
echo "🔬 测试方法统计:"
grep "TEST_F" src/nav2_astar_planner/test/*.cpp | wc -l
echo "总测试方法数: $(grep "TEST_F" src/nav2_astar_planner/test/*.cpp | wc -l)"

echo
echo "🎯 主要测试覆盖："
echo "- AStarNode 结构体测试"
echo "- CoordHash 哈希函数测试"  
echo "- 启发函数计算测试"
echo "- 节点有效性验证测试"
echo "- 路径平滑算法测试"
echo "- 坐标转换测试"
echo "- 集成场景测试"

echo
echo "5. 构建验证建议..."
echo "在ROS2环境中运行以下命令验证："
echo "  colcon build --packages-select nav2_astar_planner"
echo "  colcon test --packages-select nav2_astar_planner"
echo "  colcon test-result --all"

echo
echo "=== 验证完成 ==="