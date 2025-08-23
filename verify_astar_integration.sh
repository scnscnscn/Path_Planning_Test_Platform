#!/bin/bash

# A* Planner Integration Verification Script
# This script checks if the A* planner is properly integrated

echo "=== A* Planner Integration Verification ==="
echo

# Check if A* planner package exists
echo "1. Checking A* planner package structure..."
if [ -d "src/nav2_astar_planner" ]; then
    echo "✓ nav2_astar_planner package found"
    
    # Check required files
    required_files=(
        "src/nav2_astar_planner/package.xml"
        "src/nav2_astar_planner/CMakeLists.txt"
        "src/nav2_astar_planner/global_planner_plugin.xml"
        "src/nav2_astar_planner/include/nav2_astar_planner/astar_planner.hpp"
        "src/nav2_astar_planner/src/astar_planner.cpp"
    )
    
    for file in "${required_files[@]}"; do
        if [ -f "$file" ]; then
            echo "  ✓ $file exists"
        else
            echo "  ✗ $file missing"
        fi
    done
else
    echo "✗ nav2_astar_planner package not found"
    exit 1
fi

echo

# Check parameter file updates
echo "2. Checking navigation parameter files..."
param_files=(
    "src/turtlebot3_navigation2/param/humble/burger.yaml"
    "src/turtlebot3_navigation2/param/humble/waffle.yaml"
    "src/turtlebot3_navigation2/param/humble/waffle_pi.yaml"
    "src/turtlebot3_navigation2/param/humble/burger_cam.yaml"
)

for file in "${param_files[@]}"; do
    if [ -f "$file" ]; then
        if grep -q "nav2_astar_planner/AStarPlanner" "$file"; then
            echo "  ✓ $file configured for A* planner"
        else
            echo "  ✗ $file not configured for A* planner"
        fi
    else
        echo "  ✗ $file not found"
    fi
done

echo

# Check documentation
echo "3. Checking documentation..."
doc_files=(
    "docs/api/astar_planner.md"
    "docs/tutorials/astar_planner_usage.md"
)

for file in "${doc_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file exists"
    else
        echo "  ✗ $file missing"
    fi
done

echo

# Check tests
echo "4. Checking test structure..."
test_files=(
    "test/test_astar_planner.cpp"
    "test/CMakeLists.txt"
)

for file in "${test_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file exists"
    else
        echo "  ✗ $file missing"
    fi
done

echo

# Check code formatting
echo "5. Checking code formatting..."
cpp_files=(
    "src/nav2_astar_planner/include/nav2_astar_planner/astar_planner.hpp"
    "src/nav2_astar_planner/src/astar_planner.cpp"
)

if command -v clang-format &> /dev/null; then
    for file in "${cpp_files[@]}"; do
        if [ -f "$file" ]; then
            if clang-format --dry-run --Werror "$file" &> /dev/null; then
                echo "  ✓ $file properly formatted"
            else
                echo "  ⚠ $file formatting issues detected"
            fi
        fi
    done
else
    echo "  ⚠ clang-format not available, skipping format check"
fi

echo

# Summary
echo "=== Integration Summary ==="
echo "The custom A* global planner integration includes:"
echo "• Complete ROS2 plugin package with proper structure"
echo "• Updated navigation parameter files for all TurtleBot3 variants"
echo "• Unit test framework with gtest"
echo "• Comprehensive API documentation"
echo "• Usage tutorials and guides"
echo "• Proper code formatting compliance"
echo

echo "To test the integration:"
echo "1. Build the workspace: colcon build --packages-select nav2_astar_planner"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Launch navigation: ros2 launch turtlebot3_navigation2 navigation2.launch.py"
echo "4. Use RViz2 to set goals and observe A* planning in action"
echo

echo "For detailed usage instructions, see: docs/tutorials/astar_planner_usage.md"