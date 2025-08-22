# Contributing to Path_Planning_Test_Platform

Thank you for your interest in contributing to the Path Planning Test Platform! This guide will help you understand our development process and requirements.

## 项目概述 (Project Overview)

本仓库是基于 TurtleBot3 的路径规划器测试平台，包含 TurtleBot3 官方功能包（如turtlebot3_description、turtlebot3_navigation2）与用户自定义规划器，未来计划扩展 "不同算法耗时测试框架" 功能，用于验证路径规划算法的功能正确性与性能表现。

This repository is a TurtleBot3-based path planning test platform that includes official TurtleBot3 packages (such as turtlebot3_description, turtlebot3_navigation2) and custom planners. Future plans include expanding the "algorithm performance testing framework" functionality to validate both functional correctness and performance of path planning algorithms.

## 维护者信息 (Maintainer Information)

- **维护者 (Maintainer):** scnscnscn
- **邮箱 (Email):** WLQVincent@gmail.com
- **GitHub:** [scnscnscn](https://github.com/scnscnscn)

## 代码提交前的必要操作 (Required Operations Before Code Submission)

### C++ 代码格式规范 (C++ Code Formatting Standards)

C++ 代码需严格符合以下 clang-format 配置规则：

```yaml
BasedOnStyle: Google
IndentWidth: 2          # 缩进宽度为2个空格
ColumnLimit: 0          # 不限制行宽
AccessModifierOffset: -2 # 访问修饰符缩进偏移为-2
PointerAlignment: Left  # 指针符号（*）靠左对齐
AlignTrailingComments: true # 尾随注释对齐
AllowShortIfStatementsOnASingleLine: false # 禁止短if语句写在单行
AllowShortLoopsOnASingleLine: false # 禁止短循环写在单行
BinPackArguments: true  # 函数参数打包排列
BinPackParameters: true # 函数形参打包排列
BreakBeforeBraces: Allman # 大括号单独占一行（Allman风格）
DerivePointerAlignment: false # 不自动推导指针对齐方式
NamespaceIndentation: All # 命名空间内所有内容均缩进
ReflowComments: true    # 自动重排注释格式
SortIncludes: false     # 不自动排序头文件包含
TabWidth: 2             # Tab键宽度等效2个空格
UseTab: Never           # 禁止使用Tab键，仅用空格缩进
```

**格式化命令 (Formatting Commands):**
```bash
# 格式化单个文件
clang-format -i src/my_file.cpp

# 检查格式（不修改文件）
clang-format --dry-run --Werror src/my_file.cpp

# 格式化所有C++文件
find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

### Python 代码规范 (Python Code Standards)

Python 代码需遵循 PEP 8 规范：

```bash
# 使用 black 自动格式化
black my_script.py

# 使用 flake8 检查代码风格
flake8 my_script.py

# 修复常见问题
autopep8 --in-place --aggressive my_script.py
```

## 分支命名规则 (Branch Naming Convention)

按功能类型添加前缀，格式为 `type/feature-name`：

- **新增功能 (New Features):** `feature/add-rrt-planner`
- **修复bug (Bug Fixes):** `fix/astar-obstacle-crash`
- **新增测试 (New Tests):** `test/add-teb-time-test`
- **文档更新 (Documentation):** `docs/update-api-reference`

## 提交信息格式 (Commit Message Format)

使用前缀说明操作类型：

- `[FEATURE]` - 新增功能
- `[FIX]` - 修复问题
- `[DOC]` - 文档更新
- `[TEST]` - 测试相关
- `[REFACTOR]` - 代码重构
- `[CONFIG]` - 配置变更

**示例 (Examples):**
- `[FEATURE] 新增RRT规划器（路径：custom_planners/rrt/），配套gtest单元测试`
- `[DOC] 更新docs/planners_api.md，补充A*规划器公共API参数说明`
- `[TEST] 修复test_astar_planner.cpp中路径长度计算错误的测试用例`

## PR 提交要求 (Pull Request Requirements)

### 审核人指定 (Reviewer Assignment)
必须将 `scnscnscn` 设为 PR 审核人，并通过邮件同步告知 PR 信息。

### PR 描述内容 (PR Description Content)

PR 描述必须包含以下 3 部分核心信息：

1. **功能/修改说明 (Functionality/Change Description)**
   - 清晰说明本次 PR 实现的功能或修复的问题
   - 示例："新增 Dijkstra 规划器" 或 "解决 TEB 规划器在窄通道场景下路径震荡问题"

2. **本地验证结果 (Local Verification Results)**
   - 明确说明本地编译（`colcon build`）、测试（`colcon test`）是否通过
   - 报告是否存在未解决的警告/错误

3. **文档更新情况 (Documentation Updates)**
   - 说明是否修改 `docs/` 目录下的文件
   - 示例："已更新 docs/test_framework.md，补充耗时测试模块使用说明"
   - 未修改需标注："无文档更新"

## 开发规范 (Development Standards)

### 依赖注入模式 (Dependency Injection Pattern)
在合适场景中使用依赖注入模式，如：
- 规划器实例传递
- 测试框架与算法解耦
- 提升代码可扩展性与可测试性

### 单元测试编写 (Unit Testing)
- 新增功能必须配套编写单元测试
- 优先采用 `gtest` 框架（C++）
- 测试用例需覆盖核心逻辑与异常场景

### 文档与注释完善 (Documentation and Comments)
- 对公共 API（如规划器的 `compute_path` 函数）编写详细说明
- 复杂业务逻辑（如路径平滑算法）需详细注释
- 更新相关文档（如新增规划器的 API 说明）

## 开发流程 (Development Workflow)

### 1. 环境设置 (Environment Setup)
```bash
# 克隆仓库
git clone https://github.com/scnscnscn/Path_Planning_Test_Platform.git
cd Path_Planning_Test_Platform

# 创建功能分支
git checkout -b feature/my-new-feature

# 构建项目
colcon build

# 运行测试
colcon test
```

### 2. 代码开发 (Code Development)
- 遵循代码格式规范
- 实现功能并编写对应测试
- 更新相关文档

### 3. 提交前检查 (Pre-commit Checklist)
- [ ] 代码格式化完成（clang-format/black）
- [ ] 本地构建成功（`colcon build`）
- [ ] 所有测试通过（`colcon test`）
- [ ] 文档已更新
- [ ] 提交信息格式正确
- [ ] 新功能包含单元测试

### 4. 提交 PR (Submit PR)
- 推送分支到 GitHub
- 创建 Pull Request
- 设置 `scnscnscn` 为审核人
- 填写完整的 PR 描述

## 依赖管理规范 (Dependency Management)

### C++ 依赖管理
新增依赖时，需在对应功能包的 `package.xml` 中添加：

```xml
<package format="3">
  <name>my_planner</name>
  <version>1.0.0</version>
  <description>Custom path planner</description>
  <maintainer email="WLQVincent@gmail.com">scnscnscn</maintainer>
  <license>Apache-2.0</license>
  
  <!-- 添加新依赖 -->
  <depend>nav2_core</depend>
  <depend>geometry_msgs</depend>
  
  <test_depend>gtest</test_depend>
</package>
```

### 构建配置 (Build Configuration)
更新 `CMakeLists.txt`：

```cmake
find_package(nav2_core REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(my_planner src/my_planner.cpp)
ament_target_dependencies(my_planner nav2_core geometry_msgs)
```

## 项目结构维护 (Project Structure Maintenance)

### 禁止操作 (Prohibited Actions)
- 随意新增/删除核心目录
- 修改目录用途
- 更改官方包源码
- 调整已有功能逻辑（无明确指令时）

### 推荐结构 (Recommended Structure)
```
Path_Planning_Test_Platform/
├── custom_planners/         # 自定义规划器
├── docs/                    # 项目文档
├── test/                    # 测试文件
├── .clang-format           # 格式化配置
├── .gitignore              # Git忽略规则
└── README.md               # 项目说明
```

## 质量保证 (Quality Assurance)

### 代码审查标准 (Code Review Standards)
- 代码符合格式规范
- 功能正确实现
- 测试覆盖充分
- 文档完整准确
- 性能表现合理

### 持续集成 (Continuous Integration)
所有 PR 将自动执行：
- 代码格式检查
- 编译构建验证
- 单元测试运行
- 文档构建检查

## 常见问题 (FAQ)

### Q: 如何设置开发环境？
A: 参考 [开发指南](docs/development/README.md) 中的详细步骤。

### Q: 代码格式化失败怎么办？
A: 确保安装了 clang-format，并使用项目根目录的 `.clang-format` 配置文件。

### Q: 测试失败如何调试？
A: 使用 `colcon test --event-handlers console_direct+` 查看详细输出。

### Q: 如何添加新的路径规划算法？
A: 参考 [API文档](docs/api/README.md) 中的接口说明和示例。

## 联系方式 (Contact)

如有问题或建议，请联系：
- **邮箱 (Email):** WLQVincent@gmail.com
- **GitHub Issues:** 在仓库中创建 issue
- **讨论 (Discussions):** 使用 GitHub Discussions 功能

感谢您的贡献！(Thank you for your contribution!)