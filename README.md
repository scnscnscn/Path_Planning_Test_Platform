# Path Planning Test Platform

本项目基于TurtleBot3--humble改进而成，用于测试Navigation2相关插件。

## 项目简介

Path Planning Test Platform 是一个基于 ROS2 Humble 和 TurtleBot3 的路径规划测试平台，专门用于测试和验证 Navigation2 相关插件的性能和功能。本项目遵循 Apache License 2.0开源协议。

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Python 3.10+
- 至少 4GB RAM
- 20GB 可用磁盘空间

## 安装指南

### 1. 安装 ROS2 Humble

#### 1.1 设置本地化环境
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### 1.2 设置软件源
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 1.3 安装 ROS2 Humble
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

#### 1.4 设置环境变量
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 1.5 安装开发工具
```bash
sudo apt install python3-pip python3-pytest-cov ros-dev-tools
sudo apt install python3-flake8-docstrings python3-flake8-comprehensions
sudo apt install python3-flake8-builtins python3-flake8-quotes
```

### 2. 安装 TurtleBot3 相关包

#### 2.1 安装 TurtleBot3 包
```bash
sudo apt install ros-humble-turtlebot3*
```

#### 2.2 安装 Navigation2
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

#### 2.3 安装 Gazebo 仿真环境
```bash
sudo apt install ros-humble-gazebo-*
```

### 3. 下载和配置项目

#### 3.1 克隆项目
```bash
cd ~/
git clone https://github.com/scnscnscn/Path_Planning_Test_Platform.git
cd Path_Planning_Test_Platform
```

#### 3.2 设置 TurtleBot3 模型
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc
```

#### 3.3 安装依赖包
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

## 使用指南

### 1. 启动仿真环境

#### 1.1 启动 Gazebo 仿真
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=True

#### 1.2 启动 RViz2 可视化 (新终端)
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### 2. 建图 (SLAM)

#### 2.1 启动建图功能
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

#### 2.2 遥控机器人建图 (新终端)
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

#### 2.3 保存地图
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 3. 路径规划和导航

#### 3.1 准备地图文件
确保地图文件位于正确位置，例如：
```bash
# 将地图文件复制到指定位置
cp ~/map.yaml /home/yourname/map.yaml
cp ~/map.pgm /home/yourname/map.pgm
```

#### 3.2 启动导航系统
**重要：启动路径规划时必须明确指定地图配置文件位置**

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/yourname/map.yaml
```

> **注意**：命令中的 `map:=/home/yourname/map.yaml` 参数必须显式声明地图配置文件的完整路径。请根据实际的地图文件位置调整路径。

#### 3.3 设置导航目标
在 RViz2 中：
1. 点击 "2D Pose Estimate" 设置机器人初始位置
2. 点击 "2D Goal Pose" 设置目标位置
3. 观察机器人自主导航到目标位置

### 4. 测试 Navigation2 插件

项目支持测试各种 Navigation2 插件，包括：
- 全局路径规划器插件
- 局部路径规划器插件
- 行为树插件
- 恢复行为插件

## 常见问题

### Q1: ROS2 环境变量未设置
**A1**: 确保在每个新终端中都执行了 `source /opt/ros/humble/setup.bash`，或将其添加到 `~/.bashrc` 中。

### Q2: TurtleBot3 模型未找到
**A2**: 检查 `TURTLEBOT3_MODEL` 环境变量是否正确设置为 `waffle`、`burger` 或 `waffle_pi`。

### Q3: 地图文件加载失败
**A3**: 确认地图文件路径正确，且 yaml 文件中的图像路径指向正确的 pgm 文件。

### Q4: 导航启动失败
**A4**: 检查是否正确设置了 `use_sim_time:=True` 参数，且地图文件路径正确。

## 贡献指南

本项目欢迎社区贡献，请遵循以下步骤：

1. Fork 本仓库
2. 创建特性分支
3. 提交更改
4. 推送到分支
5. 创建 Pull Request

## 许可证

本项目基于 Apache License 2.0 开源协议，与 TurtleBot3 项目保持一致。详见 [LICENSE](LICENSE) 文件。

## 致谢

本项目基于以下开源项目：
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [Navigation2](https://github.com/ros-planning/navigation2)
- [ROS2](https://github.com/ros2)

感谢 ROBOTIS 公司和 ROS 社区为机器人开源生态做出的杰出贡献。

## 联系方式

如有问题或建议，请通过 GitHub Issues 联系我。