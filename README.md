# Path Planning Test Platform

## 项目概述 (Project Overview)

本项目基于 TurtleBot3--ROS2 Humble 改进而成，是一个专门用于测试和验证路径规划算法的综合性平台。该平台集成了 TurtleBot3 官方功能包（如 turtlebot3_description、turtlebot3_navigation2）与用户自定义规划器，并计划扩展"不同算法耗时测试框架"功能，用于验证路径规划算法的功能正确性与性能表现。

This project is based on TurtleBot3--ROS2 Humble and serves as a comprehensive platform for testing and validating path planning algorithms. The platform integrates official TurtleBot3 packages (such as turtlebot3_description, turtlebot3_navigation2) with custom user-defined planners, and plans to expand the "algorithm performance testing framework" functionality to validate both functional correctness and performance of path planning algorithms.

## 主要特性 (Key Features)

- 🚀 **多算法支持** - 支持多种路径规划算法的集成与测试
- 📊 **性能基准测试** - 提供算法执行时间和路径质量的量化分析
- 🔧 **易于扩展** - 采用模块化设计，方便添加新的规划算法
- 📚 **完整文档** - 提供详细的API文档和开发指南
- ✅ **全面测试** - 包含单元测试、集成测试和性能测试框架

## 快速开始 (Quick Start)

### 环境要求 (Prerequisites)

- ROS2 Humble
- Ubuntu 22.04 LTS
- C++17 编译器
- Python 3.8+

### 安装步骤 (Installation)

```bash
# 1. 克隆仓库 (Clone repository)
git clone https://github.com/scnscnscn/Path_Planning_Test_Platform.git
cd Path_Planning_Test_Platform

# 2. 安装依赖 (Install dependencies)
rosdep install --from-paths . --ignore-src -r -y

# 3. 构建项目 (Build project)
colcon build

# 4. 设置环境 (Source workspace)
source install/setup.bash

# 5. 运行测试 (Run tests)
colcon test
```

### 基本使用 (Basic Usage)

```bash
# 启动仿真环境 (Launch simulation)
ros2 launch path_planning_platform simulation.launch.py

# 运行路径规划测试 (Run path planning tests)
ros2 run path_planning_platform test_planners

# 查看性能报告 (View performance report)
ros2 run path_planning_platform generate_report
```

## 项目结构 (Project Structure)

```
Path_Planning_Test_Platform/
├── .github/                    # GitHub 配置文件
│   └── copilot-instructions.md # Copilot 指令文档
├── custom_planners/            # 自定义路径规划器
├── docs/                       # 项目文档
│   ├── api/                   # API 文档
│   ├── development/           # 开发指南
│   ├── testing/              # 测试文档
│   └── tutorials/            # 教程文档
├── test/                      # 测试文件
├── .clang-format             # C++ 代码格式配置
├── .gitignore               # Git 忽略文件
├── CONTRIBUTING.md          # 贡献指南
├── LICENSE                  # 开源许可证
└── README.md               # 项目说明
```

## 开发指南 (Development Guide)

### 代码规范 (Code Standards)

- **C++ 代码:** 使用 clang-format 和 Google 风格（自定义配置）
- **Python 代码:** 遵循 PEP 8 规范，使用 black 和 flake8
- **提交信息:** 使用规范化的提交前缀 `[FEATURE]`, `[FIX]`, `[DOC]`, `[TEST]`

### 贡献流程 (Contribution Workflow)

1. Fork 项目仓库
2. 创建功能分支：`git checkout -b feature/your-feature`
3. 提交更改：`git commit -m "[FEATURE] 描述你的更改"`
4. 推送分支：`git push origin feature/your-feature`
5. 创建 Pull Request

详细指南请参阅 [CONTRIBUTING.md](CONTRIBUTING.md)

## 文档 (Documentation)

- 📖 **[完整文档](docs/README.md)** - 项目完整文档索引
- 🔧 **[开发指南](docs/development/README.md)** - 开发者贡献指南
- 📚 **[API 文档](docs/api/README.md)** - 详细的 API 参考
- 🧪 **[测试指南](docs/testing/README.md)** - 测试框架和最佳实践

## 维护者 (Maintainer)

- **姓名 (Name):** scnscnscn
- **邮箱 (Email):** WLQVincent@gmail.com
- **GitHub:** [@scnscnscn](https://github.com/scnscnscn)

## 许可证 (License)

本项目采用 Apache License 2.0 许可证。详见 [LICENSE](LICENSE) 文件。

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

## 致谢 (Acknowledgments)

- TurtleBot3 团队提供的优秀机器人平台
- ROS2 Navigation2 团队的导航框架
- 所有为该项目做出贡献的开发者

## 支持 (Support)

如果您遇到问题或有建议，请：

1. 查看 [文档](docs/README.md) 寻找解决方案
2. 在 [Issues](https://github.com/scnscnscn/Path_Planning_Test_Platform/issues) 中搜索相关问题
3. 创建新的 Issue 描述您的问题
4. 联系维护者：WLQVincent@gmail.com

---

**欢迎贡献代码和提出建议！(Contributions and suggestions are welcome!)**
