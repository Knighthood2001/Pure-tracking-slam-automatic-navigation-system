# su7ultra_simulation

基于 **ROS 2 + Gazebo Classic** 的 **小米 SU7 Ultra** 阿克曼转向车辆自主导航系统。

系统集成两套导航方案：
1. **自研纯追踪 + A\* 路径规划**（`nav_slam`）—— 轻量级，适合学习与定制
2. **Nav2 全栈导航**（`su7ultra_navigation2`）—— 工业级，支持动态避障与行为恢复

同时提供自定义 Gazebo 车辆控制插件（`four_wheeled_vehicle`），实现阿克曼运动学的 PID 闭环控制。

---

## 目录

- [环境要求](#环境要求)
- [导航方案对比](#导航方案对比)
- [快速开始](#快速开始)
- [功能包总览](#功能包总览)
- [项目架构](#项目架构)
  - [1. su7ultra_description](#1-su7ultra_description--su7-ultra-车辆模型描述)
  - [2. su7ultra_navigation2](#2-su7ultra_navigation2--nav2-导航配置)
  - [3. nav_slam](#3-nav_slam--自研-slam--路径规划--纯追踪导航)
  - [4. ackermann_steering_angle](#4-ackermann_steering_angle--转向角监测)
  - [5. four_wheeled_vehicle](#5-four_wheeled_vehicle--自定义阿克曼车辆控制插件)
  - [6. gazebo_modele](#6-gazebo_modele--简易-gazebo-模型启动)
- [TF 坐标系树](#tf-坐标系树)
- [ROS 话题一览](#ros-话题一览)
- [常见问题排查](#常见问题排查)
- [完整导航工作流程](#完整导航工作流程)
- [地图文件说明](#地图文件说明)
- [常用调试命令速查](#常用调试命令速查)

---

## 环境要求

| 项目 | 版本 / 说明 |
|------|-------------|
| 操作系统 | Ubuntu 22.04 LTS（推荐） |
| ROS 2 | Humble / Iron / Jazzy |
| Gazebo | Classic 11.x（非 Ignition/Garden） |
| Python | 3.10+（需 `numpy`, `scipy`, `tf-transformations`） |

**系统依赖安装：**

```bash
# ROS 2 基础依赖
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs \
                 ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher

# Nav2 导航栈（su7ultra_navigation2 需要）
sudo apt install ros-$ROS_DISTRO-navigation2 \
                 ros-$ROS_DISTRO-nav2-bringup
```

---

## 导航方案对比

本系统提供两套互补的导航方案，可根据场景需求选择：

| 对比维度 | 方案一：Nav2 全栈导航 | 方案二：自研 SLAM + 纯追踪 |
|----------|----------------------|--------------------------|
| **功能包** | `su7ultra_navigation2` | `nav_slam` |
| **定位** | AMCL 粒子滤波 | 里程计 + odom→map TF |
| **路径规划** | SmacPlanner Hybrid-A*（Reeds-Shepp） | A* + B 样条平滑 |
| **路径跟踪** | MPPI 控制器（Ackermann 约束） | Pure Pursuit 纯追踪 |
| **避障** | 全局 + 局部代价地图，动态避障 | 栅格地图膨胀，静态避障 |
| **行为恢复** | 清除地图 / 前进 / 等待 / 后退 | 无（到达阈值 0.2m 停止） |
| **适用场景** | 复杂环境，动态障碍物，长距离 | 已知环境，短距离，教学演示 |
| **依赖** | Nav2 全家桶 | numpy + scipy |

---

## 快速开始

### 1. 编译

```bash
cd Pure-tracking-slam-automatic-navigation-system
colcon build
source install/setup.bash
```

> 如需仅编译特定包：`colcon build --packages-select su7ultra_description su7ultra_navigation2`
>
> 编译 `four_wheeled_vehicle` 需要 Gazebo 开发库：`sudo apt install libgazebo-dev`

### 2. 安装 Gazebo 模型

```bash
cp -r src/su7ultra_description/models/* ~/.gazebo/models
```

> **注意：** 复制后需检查以下文件中 mesh 的 `filename` 路径是否指向 `~/.gazebo/models/su7ultra/meshes/`：
> - `src/su7ultra_description/urdf/vehicle/base.urdf.xacro` → `car.dae`
> - `src/su7ultra_description/urdf/vehicle/actuator/wheel.urdf.xacro` → `wheel_l.dae` / `wheel_r.dae`

### 3. 启动 Gazebo 仿真

```bash
ros2 launch su7ultra_description gazebo_sim.launch.py
```

该命令执行流程：
```
xacro 解析 vehicle.urdf.xacro
  → robot_state_publisher 发布 /robot_description
    → Gazebo 启动 + 加载 ackermann_test.world
      → spawn_entity.py 在 (0, 0, 0.325) 生成 SU7 Ultra 模型
        → gazebo_ackermann_drive 插件激活（订阅 /cmd_vel，发布 /odom）
        → joint_state_publisher 插件激活（发布 /joint_states）
```

### 4a. 启动 Nav2 导航（方案一）

```bash
ros2 launch su7ultra_navigation2 navigation2.launch.py
```

该命令执行流程：
```
nav2_bringup/bringup_launch.py
  → map_server 加载 test.yaml 地图
    → AMCL 定位节点启动（等待 2D Pose Estimate 初始定位）
      → controller_server（MPPI）+ planner_server（SmacPlanner）
        → behavior_server + bt_navigator + velocity_smoother
          → RViz2 打开（nav2_default_view.rviz）
```

在 RViz2 中操作：
1. 点击 **`2D Pose Estimate`** → 在地图上点击设定初始位姿（AMCL 定位）
2. 点击 **`Nav2 Goal`** → 在地图上点击设定目标点开始导航

### 4b. 启动自研 SLAM + 纯追踪导航（方案二）

```bash
ros2 launch nav_slam 2dpoints.launch.py
```

该命令同时启动 5 个节点 + RViz：
```
points_pub_map  → 订阅 /points_raw + /odom → 发布变换后点云 /mapokk
map_pub         → 订阅 /mapokk + /odom → 构建 2D 栅格地图 /combined_grid
astar           → 订阅 /combined_grid + /goal_pose → 规划路径 /path + /path2
start_nav       → 订阅 /path + /odom → 纯追踪发布 /cmd_vel
odom_map_tf     → 订阅 /odom → 发布 TF map→odom
```

在 RViz2 中使用 **`2D Goal Pose`** 发送目标点即可自动导航。

### 5. 其他启动命令

| 命令 | 说明 |
|------|------|
| `ros2 launch su7ultra_description display_robot.launch.py` | RViz 中展示机器人模型（不启动 Gazebo） |
| `ros2 launch su7ultra_description rviz.launch.py` | 单独启动 RViz（配合 Gazebo 使用） |
| `ros2 launch ackermann_steering_angle steering_angle.launch.py` | 启动转向角监测节点 |
| `ros2 launch four_wheeled_vehicle vehicle_gazebo_ok.launch.py` | 使用自研 C++ 插件启动 Gazebo 仿真 |
| `ros2 launch gazebo_modele gazebo.launch.py` | 简易模型启动（调试用） |

---

## 功能包总览

| 功能包 | 构建类型 | 说明 |
|--------|----------|------|
| `su7ultra_description` | ament_cmake | SU7 Ultra URDF 模型、传感器、世界文件 |
| `su7ultra_navigation2` | ament_cmake | Nav2 导航栈完整配置（MPPI + SmacPlanner） |
| `nav_slam` | ament_python | 自研 SLAM 建图 + A* 路径规划 + 纯追踪控制器 |
| `ackermann_steering_angle` | ament_python | 通过 TF / JointState 实时监测前轮转向角 |
| `four_wheeled_vehicle` | ament_cmake | 自定义 C++ Gazebo 阿克曼控制插件（PID 闭环） |
| `gazebo_modele` | ament_python | 简易 Gazebo 模型启动器（调试用） |

---

## 项目架构

### 1. su7ultra_description — SU7 Ultra 车辆模型描述

SU7 Ultra 阿克曼转向车辆的完整 URDF 模型定义，包含底盘、执行器、传感器及 Gazebo 仿真插件。

```
su7ultra_description/
├── urdf/vehicle/
│   ├── base.urdf.xacro              # 底盘（质量 2000kg，包围盒 5.07×1.97×0.47m）
│   ├── vehicle.urdf.xacro           # 主入口，组装所有组件
│   ├── vehicle_simple.urdf.xacro    # 简化版（无详细 mesh）
│   ├── actuator/
│   │   ├── steering.urdf.xacro      # 前轮转向关节（revolute，±0.7rad，PID 5000/0/800）
│   │   └── wheel.urdf.xacro         # 车轮（左/右独立宏，摩擦系数 μ1=2.0, μ2=1.5）
│   ├── sensor/
│   │   ├── lidar3d.urdf.xacro       # 3D 激光雷达（32 线，270° FOV，100m）
│   │   ├── laser.urdf.xacro         # 2D 激光雷达（360°，30m，供 Nav2 / SLAM 使用）
│   │   ├── camera.urdf.xacro        # 前置摄像头（1280×720@30fps，80° FOV）
│   │   ├── imu.urdf.xacro           # IMU（200Hz，高斯噪声）
│   │   └── gps.urdf.xacro           # GPS（10Hz）
│   └── plugin/
│       └── gazebo_control.xacro     # 阿克曼驱动 + 关节状态发布插件
├── models/su7ultra/meshes/          # 3D 网格（car.dae, wheel_l/r.dae）+ 纹理贴图
├── worlds/
│   ├── ackermann_test.world         # 阿克曼测试场地（默认加载）
│   ├── 2d.world                     # 2D 平面世界
│   └── 3d.world                     # 3D 立体世界
├── rviz/
│   ├── display_model.rviz           # 模型展示配置
│   └── gazebo_sim.rviz              # 仿真配置
├── launch/
│   ├── gazebo_sim.launch.py         # 启动 Gazebo + 生成车辆
│   ├── display_robot.launch.py      # RViz 模型展示（无 Gazebo）
│   └── rviz.launch.py               # 单独启动 RViz
├── CMakeLists.txt
└── package.xml
```

**车辆关键参数：**

| 参数 | 值 |
|------|-----|
| 整车质量 | 2000 kg |
| 车身尺寸 (长×宽×高) | 5.07 × 1.97 × 0.47 m |
| 车轮半径 | 0.30 m |
| 车轮宽度 | 0.35 m |
| 前轮轮距 | 1.666 m |
| 前后轴距 | 2.990 m |
| 最大转向角 | ±0.7 rad（≈40°） |
| 最大速度 | 5.0 m/s（插件设定） |
| 最小转弯半径 | 4.39 m |
| 车轮摩擦系数 | μ1 = 2.0, μ2 = 1.5 |

**传感器配置：**

| 传感器 | 安装位置 (x, y, z) m | 话题 | 更新率 | 参数 |
|--------|----------------------|------|--------|------|
| 3D 激光雷达 | (1.30, 0, 0.95) | `/points_raw` (PointCloud2) | 10Hz | 32 线，270° FOV，100m |
| 2D 激光雷达 | (1.30, 0, 0.30) | `/scan` (LaserScan) | 5Hz | 360°，30m，Nav2/SLAM 用 |
| 前置摄像头 | (2.20, 0, 0.60) | `/image_raw`, `/camera_info` | 30Hz | 1280×720，R8G8B8 |
| IMU | (0, 0, 0.30) | `/imu_raw` (Imu) | 200Hz | accel σ=0.05, gyro σ=0.001 |
| GPS | (-0.50, 0, 0.95) | `/gps/fix`, `/gps/vel` | 10Hz | — |

**Gazebo 插件：**
- `gazebo_ackermann_drive` — 阿克曼驱动，订阅 `/cmd_vel`，发布 `/odom` 和 `odom→base_footprint` TF
- `gazebo_ros_joint_state_publisher` — 发布 6 个关节状态到 `/joint_states`

**阿克曼驱动插件参数 (`gazebo_control.xacro`)：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `update_rate` | 50.0 Hz | 控制更新频率 |
| `max_steer` | 0.699 rad | 最大转向角 |
| `max_speed` | 5.0 m/s | 最大速度 |
| 转向 PID (左/右) | P=5000, I=0, D=800 | 前轮转向控制 |
| 速度 PID | P=2000, I=0, D=2 | 后轮驱动控制 |
| `publish_odom` | true | 发布里程计 |
| `publish_odom_tf` | true | 发布 odom→base_footprint TF |
| `odometry_frame` | odom | 里程计坐标系 |
| `robot_base_frame` | base_footprint | 机器人基准坐标系 |

**Xacro 宏设计模式：**

整个 URDF 采用模块化 xacro 宏设计，每个组件为独立宏定义，通过参数化实现复用：

```xml
<!-- 宏定义示例：steering.urdf.xacro -->
<xacro:macro name="steering_xacro" params="steering_name parent_link xyz">
    <!-- 关节 + link 定义 -->
</xacro:macro>

<!-- 主入口调用：vehicle.urdf.xacro -->
<xacro:steering_xacro steering_name="front_left" parent_link="base_link" xyz="1.445 0.833 -0.024" />
<xacro:steering_xacro steering_name="front_right" parent_link="base_link" xyz="1.445 -0.833 -0.024" />
```

这种设计的优势：代码复用、模块化分离、修改单个组件不影响整体、新增传感器只需添加新 xacro 文件并在主入口引用。

**车辆模型变体：**

| 文件 | 包含组件 | 适用场景 |
|------|----------|----------|
| `vehicle.urdf.xacro` | 底盘 + 转向 + 车轮 + **全部传感器** + 控制插件 | 完整仿真（推荐） |
| `vehicle_simple.urdf.xacro` | 底盘 + 转向 + 车轮 + 控制插件（**无传感器**） | 轻量化调试 |

**Launch 文件说明：**

| 文件 | 启动内容 |
|------|----------|
| `gazebo_sim.launch.py` | xacro 解析 → Gazebo + ackermann_test.world → spawn 车辆 → robot_state_publisher |
| `display_robot.launch.py` | xacro 解析 → robot_state_publisher + joint_state_publisher + RViz |
| `rviz.launch.py` | 仅启动 RViz2（配合 Gazebo 使用） |

---

### 2. su7ultra_navigation2 — Nav2 导航配置

基于 Nav2 导航栈的完整配置，针对阿克曼车辆特性进行了深度适配。

```
su7ultra_navigation2/
├── config/
│   ├── nav2_params.yaml             # Nav2 全栈参数（AMCL、MPPI、SmacPlanner 等）
│   └── test_nav2_params.yaml        # 测试用参数
├── maps/
│   ├── test.yaml / test.pgm         # 测试地图（60×40m，分辨率 0.05m/px）
│   ├── room.yaml / room.pgm         # 房间地图（36×34m）
│   └── world.yaml / world.pgm       # 世界地图（71×22m）
├── behavior_trees/
│   ├── navigate_to_pose_w_replanning_and_recovery.xml       # 单目标导航 BT
│   └── navigate_through_poses_w_replanning_and_recovery.xml # 多路点导航 BT
├── launch/
│   └── navigation2.launch.py        # Nav2 启动入口
├── CMakeLists.txt
└── package.xml
```

**Nav2 核心模块配置：**

| 模块 | 插件 | 关键参数 |
|------|------|----------|
| **定位 (AMCL)** | `nav2_amcl::DifferentialMotionModel` | 粒子 500~5000，激光模型 likelihood_field |
| **路径规划** | `nav2_smac_planner/SmacPlannerHybrid` | Reeds-Shepp 运动模型，最小转弯半径 4.39m |
| **路径跟踪** | `nav2_mppi_controller::MPPIController` | Ackermann 运动学，batch_size=2000，vx_max=2.0 |
| **代价地图（局部）** | VoxelLayer + InflationLayer | 15×15m 滑动窗口，分辨率 0.05m |
| **代价地图（全局）** | StaticLayer + ObstacleLayer + InflationLayer | 静态地图 + 障碍膨胀 |
| **路径平滑** | `nav2_smoother::SimpleSmoother` | 1000 次迭代，w_smooth=0.5 |
| **速度平滑** | velocity_smoother | 最大加速度 1.5 m/s²，最大减速度 2.0 m/s² |
| **行为恢复** | BackUp / DriveOnHeading / Wait | 无 Spin（阿克曼车辆无法原地旋转） |

**Nav2 行为树 (Behavior Tree) 流程：**

系统提供两棵自定义行为树，均移除了 `Spin`（阿克曼车辆无法原地旋转），替换为 `DriveOnHeading`：

**单目标导航 BT** (`navigate_to_pose_w_replanning_and_recovery.xml`)：
```
NavigateRecovery (最多重试 6 次)
├── NavigateWithReplanning (PipelineSequence)
│   ├── ComputePathToPose (1Hz 重规划，失败→清除全局代价地图)
│   └── FollowPath (失败→清除局部代价地图)
└── RecoveryFallback (RoundRobin 轮询恢复)
    ├── ClearingActions (清除全局+局部代价地图)
    ├── DriveOnHeading (前进 0.30m, 速度 0.05m/s)
    ├── Wait (等待 5 秒)
    └── BackUp (后退 0.30m, 速度 0.05m/s)
```

**多路点导航 BT** (`navigate_through_poses_w_replanning_and_recovery.xml`)：
- 额外包含 `RemovePassedGoals`（移除已通过的路点，半径 0.7m）
- 重规划频率降低至 0.333Hz（每 3 秒一次）
- 其余恢复策略与单目标 BT 相同

**Nav2 参数文件：**

| 文件 | 用途 | 关键差异 |
|------|------|----------|
| `nav2_params.yaml` | 正式 SU7 Ultra 模型 | `base_footprint` 基准帧，footprint 5.07×1.97m，min_turning_r=4.39m |
| `test_nav2_params.yaml` | 简化测试模型 | `base_link` 基准帧，footprint 0.84×0.70m，min_turning_r=1.48m，含 spin 行为 |

**阿克曼车辆适配要点：**

- **MPPI 控制器** — `motion_model: "Ackermann"`，`min_turning_r: 4.39m`，使用 8 个评价函数：
  - `ConstraintCritic` (w=4.0) — 运动学约束
  - `ObstaclesCritic` (critical_w=30.0) — 障碍物排斥
  - `GoalCritic` (w=15.0) — 目标导向
  - `GoalAngleCritic` (w=3.0) — 目标角度
  - `PathAlignCritic` (w=25.0) — 路径对齐
  - `PathFollowCritic` (w=25.0) — 路径跟踪
  - `PathAngleCritic` (w=4.0) — 路径角度
  - `PreferForwardCritic` (w=1.5) — 前进偏好
- **SmacPlanner** — `motion_model_for_search: "REEDS_SHEPP"`，支持前进/倒车路径，`reverse_penalty: 10.0`
- **行为树** — 移除 `Spin`，替换为 `DriveOnHeading`；恢复策略：清除代价地图 → 前进 → 等待 → 后退
- **车辆 footprint** — `[[2.535, 0.985], [2.535, -0.985], [-2.535, -0.985], [-2.535, 0.985]]`
- **速度限制** — 前进 2.0 m/s，后退 -1.0 m/s，角速度 1.2 rad/s

**Launch 参数：**

```bash
# 默认启动（使用 test.yaml 地图）
ros2 launch su7ultra_navigation2 navigation2.launch.py

# 指定地图
ros2 launch su7ultra_navigation2 navigation2.launch.py map:=/path/to/room.yaml

# 指定自定义参数文件
ros2 launch su7ultra_navigation2 navigation2.launch.py params_file:=/path/to/custom_params.yaml
```

---

### 3. nav_slam — 自研 SLAM + 路径规划 + 纯追踪导航

轻量级自研导航方案，不依赖 Nav2，适合学习阿克曼车辆的路径规划与轨迹跟踪原理。

```
nav_slam/
├── nav_slam/
│   ├── astar.py              # A* 路径规划节点
│   ├── start_nav.py          # 纯追踪 (Pure Pursuit) 控制器节点
│   ├── map_pub.py            # 点云 → 2D OccupancyGrid 建图节点
│   ├── points_pub_map.py     # 点云坐标变换（传感器→地图坐标系）节点
│   └── odom_map_tf.py        # odom→map TF 变换发布节点
├── config/
│   └── rviz.rviz             # RViz 可视化配置
├── launch/
│   └── 2dpoints.launch.py    # 一键启动全部 5 个节点 + RViz
├── map/                      # 预建地图文件
├── package.xml
└── setup.py
```

**节点说明：**

| 节点 | 订阅 | 发布 | 功能 |
|------|------|------|------|
| `points_pub_map` | `/points_raw` (3D 点云), `/odom` | `/mapokk` (变换后点云) | 将 3D 激光雷达点云从传感器坐标系变换到地图坐标系 |
| `map_pub` | `/mapokk`, `/odom` | `/combined_grid` (OccupancyGrid) | 点云投影为 2D 栅格地图，含 3 层障碍物膨胀 |
| `astar` | `/combined_grid`, `/odom`, `/goal_pose` | `/path`, `/path2` | A* 搜索 + B 样条曲线平滑路径规划 |
| `start_nav` | `/odom`, `/path` | `/cmd_vel` | 纯追踪控制器，自适应速度 |
| `odom_map_tf` | `/odom` | TF: `map→odom` | 发布 odom 到 map 的坐标变换 |

**纯追踪控制器参数：**

| 参数 | 值 | 说明 |
|------|-----|------|
| lookahead_distance | 0.5 m | 前瞻距离 |
| 速度范围 | 0.6 ~ 1.5 m/s | 根据转向角自适应调速 |
| 到达阈值 | 0.2 m | 终点判定距离 |
| 路径插值间隔 | 0.1 m | 路径点线性插值 |

**纯追踪速度自适应公式：**

控制器根据当前转向角 \(\alpha\) 动态调节前进速度：

\[v = \max(0.6,\; 1.5 - 1.5 \cdot \sin(0.6 \cdot |\alpha|))\]

- 转向角为 0°（直行）→ 速度 1.5 m/s（最大）
- 转向角增大 → 速度平滑下降
- 最小速度保底 0.6 m/s，确保车辆不会停滞

**`map_pub` 建图节点参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grid_width` | 60.0 m | 栅格地图宽度 |
| `grid_height` | 60.0 m | 栅格地图高度 |
| `resolution` | 0.1 m/px | 栅格地图分辨率 |
| `min_height` | 0.1 m | 点云有效最小高度（过滤地面） |
| `max_height` | 1.0 m | 点云有效最大高度（过滤高处） |
| `obstacle_radius` | 0.2 m | 障碍物基础半径 |

> 膨胀采用 3 层同心圆策略：第 1 层（代价=5）、第 2 层（代价=-8）、第 3 层（代价=-120），半径分别为 1×、2×、3× `obstacle_radius`。

**A* 规划器特性：**
- 8 方向搜索（含对角线），欧几里得启发式
- 障碍物膨胀 5 个栅格（`expansion_size = 5`）
- B 样条曲线平滑路径，消除折线效应

**自研导航数据流：**

```
3D 激光雷达 (/points_raw)
        │
        ▼
┌─────────────────┐   /odom    ┌───────────────────┐
│  points_pub_map  │◄──────────│   Gazebo 里程计     │
│  坐标变换到地图系  │           └───────────────────┘
└────────┬────────┘                    │
         │ /mapokk (变换后点云)          │ /odom
         ▼                             │
┌─────────────────┐                    │
│    map_pub       │                    │
│  点云→2D栅格地图  │                    │
│  3层膨胀(5/10/15格)│                   │
└────────┬────────┘                    │
         │ /combined_grid              │
         ▼                             │
┌─────────────────┐   /goal_pose       │
│     astar        │◄─────────────────┘
│  A*搜索+B样条平滑 │   (RViz 点击目标点)
└────────┬────────┘
         │ /path (原始) + /path2 (平滑后)
         ▼
┌─────────────────┐
│   start_nav      │──→ /cmd_vel ──→ Gazebo 阿克曼插件
│  Pure Pursuit     │
│  自适应速度控制    │
└─────────────────┘
```

**启动方式：**

```bash
# 一键启动（需先启动 Gazebo 仿真）
ros2 launch nav_slam 2dpoints.launch.py
```

---

### 4. ackermann_steering_angle — 转向角监测

实时监测阿克曼车辆前轮实际转向角度的 ROS 2 节点，同时提供 TF 方式和 JointState 方式两种测量手段。

```
ackermann_steering_angle/
├── ackermann_steering_angle/
│   └── ackermann_steering_angle.py   # 转向角监测节点
├── launch/
│   └── steering_angle.launch.py      # 启动文件
├── package.xml
└── setup.py
```

**节点功能：**
- **TF 方式**：查询 `base_link → front_left/right_steering_link` 的变换，提取偏航角
- **JointState 方式**：订阅 `/joint_states`，读取转向关节位置
- 发布左/右/平均转向角到以下话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vehicle/front_left_steering_angle` | Float32 | 左前轮转向角 (°) |
| `/vehicle/front_right_steering_angle` | Float32 | 右前轮转向角 (°) |
| `/vehicle/steering_angle` | Float32 | 左右平均转向角 (°) |

**Launch 参数（可在 launch 文件中修改）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `reference_frame` | `base_link` | 参考坐标系 |
| `publish_rate` | 2.0 Hz | 发布频率 |

```bash
ros2 launch ackermann_steering_angle steering_angle.launch.py
```

---

### 5. four_wheeled_vehicle — 自定义阿克曼车辆控制插件

自研 C++ Gazebo 插件，实现标准 `cmd_vel` → 阿克曼运动学的闭环控制，包含 PID 转向与速度控制、里程计发布。

```
four_wheeled_vehicle/
├── include/
│   └── std_msg_vehicle_plugin.h      # 插件头文件
├── src/
│   ├── std_msg_vehicle_plugin.cpp     # 核心控制插件实现
│   ├── odom_mapTF.cpp                 # odom→map TF 发布节点
│   └── odom_baselinkTF.cpp            # odom→base_link TF 发布节点
├── urdf/vehicle/                      # URDF 模型（与 su7ultra 结构相同）
├── models/                            # 3D 模型文件
├── launch/                            # 启动文件
├── worlds/                            # Gazebo 世界文件
├── rviz/                              # RViz 配置
├── map/                               # 预建地图
├── CMakeLists.txt
└── package.xml
```

**插件核心逻辑 (`std_msg_vehicle_plugin.cpp`)：**

1. 订阅 `/cmd_vel`（Twist），将 `linear.x` 和 `angular.z` 转换为目标转向角：
   - `steer = atan(angular_z * wheelbase / linear_x)`
2. **阿克曼转向几何**：左右前轮独立计算转向角
   - `fl = atan(L * tan(δ) / (L - 0.5 * W * tan(δ)))`
   - `fr = atan(L * tan(δ) / (L + 0.5 * W * tan(δ)))`
3. **PID 控制器**：
   - 转向 PID：P=2000, I=0, D=300（输出限幅 ±5000）
   - 速度 PID：P=1000, I=0, D=1（输出限幅 ±5000）
4. 发布 `odom`（frame: `odom` → `base_footprint`），频率 10Hz

**与 su7ultra_description 的区别：**

| 对比项 | four_wheeled_vehicle | su7ultra_description |
|--------|---------------------|---------------------|
| 驱动插件 | 自研 C++ 插件 (`libstd_msg_vehicle_plugin.so`) | Gazebo 官方 `libgazebo_ros_ackermann_drive.so` |
| 转向控制 | 阿克曼几何公式 + PID 闭环 | 内置阿克曼 + PID |
| 模型格式 | SDF（`model_sensor.sdf`） | URDF/xacro |
| 里程计发布 | 自研 `PublishOdom()` 方法（10Hz） | 插件内置 `publish_odom: true` |
| TF 发布 | 额外提供 `odom_mapTF`、`odom_baselinkTF` 可执行文件 | 插件内置 `publish_odom_tf: true` |
| 传感器 TF | 静态 TF 手动发布（lidar3d、gps、imu、camera） | URDF xacro 自动定义 |
| 适用场景 | 自定义控制逻辑、深度调试、学习阿克曼原理 | 快速集成、标准方案、推荐生产使用 |

**`four_wheeled_vehicle` 的 Launch 文件：**

| 文件 | 说明 |
|------|------|
| `vehicle_gazebo_ok.launch.py` | 完整启动：Gazebo + SDF 模型 + 静态 TF + odom_mapTF 节点 |
| `gazebo_sim.launch.py` | 仅启动 Gazebo 仿真 |
| `display_robot.launch.py` | RViz 模型展示 |
| `rviz.launch.py` | 单独 RViz |

> `vehicle_gazebo_ok.launch.py` 使用 SDF 格式加载模型，并通过 `static_transform_publisher` 手动发布传感器 TF：
> - `base_link → lidar3d_link` (z=2.25m)
> - `base_link → gps_link` (z=2.25m)
> - `base_link → imu_link` (z=2.25m)
> - `base_link → camera_link` (z=2.35m)
> - `odom → base_link` (z=2.0m)

**C++ 可执行节点：**

| 节点 | 源码 | 功能 |
|------|------|------|
| `odom_mapTF` | `odom_mapTF.cpp` | 订阅 `/odom`，发布动态 TF `map→odom` |
| `odom_baselinkTF` | `odom_baselinkTF.cpp` | 订阅 `/odom`，发布动态 TF `odom→base_footprint` |

---

### 6. gazebo_modele — 简易 Gazebo 模型启动

轻量级启动包，用于快速在 Gazebo 中加载简单 URDF 模型进行调试，包含静态 TF 发布。

```
gazebo_modele/
├── urdf/model.urdf           # 简单 URDF 模型
├── world/
│   ├── 2d.world
│   └── 3d.world
├── launch/
│   └── gazebo.launch.py      # 启动 Gazebo + 模型 + 静态 TF
├── package.xml
└── setup.py
```

> 此包主要用于开发调试阶段，正式发布推荐使用 `su7ultra_description`。

---

## TF 坐标系树

### Gazebo 仿真模式（su7ultra_description + gazebo_ackermann_drive）

```
map
 └── odom                          (Gazebo 阿克曼插件发布)
      └── base_footprint           (阿克曼插件发布 odom→base_footprint TF)
           └── base_link           (robot_state_publisher 发布)
                ├── front_left_steering_link
                │    └── front_left_wheel_link
                ├── front_right_steering_link
                │    └── front_right_wheel_link
                ├── rear_left_wheel_link
                ├── rear_right_wheel_link
                ├── lidar3d_link
                ├── laser_link
                ├── camera_link
                ├── imu_link
                └── gps_link
```

### 自研 SLAM 导航模式（nav_slam）

```
map                                  (odom_map_tf 节点发布 map→odom)
 └── odom
      └── base_footprint / base_link (Gazebo 插件发布)
           └── (其余传感器 link 同上)
```

---

## ROS 话题一览

### 传感器话题

| 话题 | 消息类型 | 来源 | 说明 |
|------|----------|------|------|
| `/scan` | LaserScan | 2D 激光雷达 | 360° 扫描，Nav2/SLAM 使用 |
| `/points_raw` | PointCloud2 | 3D 激光雷达 | 32 线点云 |
| `/image_raw` | Image | 前置摄像头 | RGB 图像 |
| `/camera_info` | CameraInfo | 前置摄像头 | 相机参数 |
| `/imu_raw` | Imu | IMU | 加速度计 + 陀螺仪 |
| `/gps/fix` | NavSatFix | GPS | 定位数据 |
| `/gps/vel` | TwistStamped | GPS | 速度数据 |

### 控制与导航话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/cmd_vel` | Twist | 速度指令（linear.x 前进，angular.z 转向） |
| `/odom` | Odometry | 里程计（frame: odom → base_footprint） |
| `/joint_states` | JointState | 6 个关节状态（2 转向 + 4 车轮） |
| `/path` | Path | A* 规划原始路径 |
| `/path2` | Path | B 样条平滑后路径 |
| `/combined_grid` | OccupancyGrid | 自研 2D 栅格地图 |
| `/goal_pose` | PoseStamped | 导航目标点 |

---

## 常见问题排查

### 1. SLAM Toolbox 丢弃消息

**现象：**
```
Message Filter dropping message: frame 'laser_link' ... queue is full
```
**解决：** 降低 2D 激光雷达更新率，在 `laser.urdf.xacro` 中设置 `<update_rate>5</update_rate>`。

### 2. TF 时间戳不匹配

**现象：**
```
the timestamp on the message is earlier than all the data in the transform cache
```
**解决：** 确保 Gazebo 插件使用仿真时间，在 `std_msg_vehicle_plugin.cpp` 中：
```cpp
ros_node_ = rclcpp::Node::make_shared("std_msg_vehicle_controller",
    rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
```

### 3. 转向关节 TF 缺失

**现象：**
```
No transform from [front_left_steering_link] to [odom]
```
**解决：** 确保 Gazebo 关节状态发布插件已加载，`/joint_states` 话题正常发布。在 `gazebo_control.xacro` 中添加：
```xml
<gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
        <ros>
            <namespace>/</namespace>
            <remapping>~/out:=/joint_states</remapping>
        </ros>
        <update_rate>30</update_rate>
        <joint_name>front_left_steering_joint</joint_name>
        <joint_name>front_right_steering_joint</joint_name>
        <!-- ... 其余关节 ... -->
    </plugin>
</gazebo>
```

### 4. Nav2 启动后地图为空

**现象：** 运行 `navigation2.launch.py` 后 RViz 中无地图显示。
**解决：** 需先使用 AMCL 进行初始定位。在 RViz 中使用 `2D Pose Estimate` 工具在地图上点击设定初始位姿。

### 5. 模型 mesh 文件加载失败

**现象：** Gazebo 中车辆显示为白色或透明。
**解决：** 确认已将模型文件复制到 `~/.gazebo/models/` 并检查 URDF 中的 `filename` 路径：
```bash
cp -r src/su7ultra_description/models/* ~/.gazebo/models
# 检查路径：file:///home/<YOUR_USER>/.gazebo/models/su7ultra/meshes/car.dae
```

> **Gazebo Classic (11) + ROS 2 专用写法**，如使用 Gazebo Sim (Ignition) 则需通过 `gz_ros_bridge` 做桥接，写法不同。

### 6. 阿克曼车辆无法原地旋转

**现象：** 导航时行为恢复中 `Spin` 动作无响应。
**解决：** 这是正确的行为。阿克曼转向车辆物理上无法原地旋转（不同于差速驱动机器人）。系统已移除此行为，使用 `DriveOnHeading`（前进）替代。请确认：
- `nav2_params.yaml` 中 `behavior_plugins` 不含 `spin`
- 行为树 XML 中无 `<Spin>` 节点

---

## 完整导航工作流程

### Nav2 导航完整流程（推荐）

```
终端 1: 启动 Gazebo 仿真
$ ros2 launch su7ultra_description gazebo_sim.launch.py
  ┌─ Gazebo 窗口出现
  ├─ 车辆模型加载到 ackermann_test.world
  ├─ /cmd_vel, /odom, /scan, /joint_states 话题就绪
  └─ TF 树: odom → base_footprint → base_link → sensors

终端 2: 启动 Nav2
$ ros2 launch su7ultra_navigation2 navigation2.launch.py
  ┌─ 地图加载（test.yaml）
  ├─ AMCL / MPPI / SmacPlanner / BT Navigator 全部启动
  └─ RViz2 打开

RViz2 中操作:
  1. [2D Pose Estimate] → 点击地图上车辆大致位置（AMCL 初始定位）
  2. [Nav2 Goal] → 点击目标位置
     ├─ SmacPlanner 使用 Reeds-Shepp 规划路径（支持倒车）
     ├─ MPPI 控制器跟踪路径（Ackermann 运动学约束）
     ├─ 局部代价地图实时避障
     └─ 如遇困：BT 自动执行恢复策略（清除地图→前进→等待→后退）
```

### 自研纯追踪导航完整流程

```
终端 1: 启动 Gazebo 仿真
$ ros2 launch su7ultra_description gazebo_sim.launch.py

终端 2: 启动自研导航
$ ros2 launch nav_slam 2dpoints.launch.py
  ┌─ 5 个节点同时启动
  ├─ 3D 点云 → 2D 栅格地图实时构建
  └─ RViz2 打开（自定义 rviz.rviz 配置）

RViz2 中操作:
  1. [2D Goal Pose] → 点击目标位置
     ├─ astar 节点在 /combined_grid 上 A* 搜索
     ├─ B 样条曲线平滑路径 → 发布 /path 和 /path2
     └─ start_nav 纯追踪控制器跟踪路径 → 发布 /cmd_vel
```

---

## 地图文件说明

`su7ultra_navigation2/maps/` 包含三张预建地图：

| 地图文件 | 尺寸 (m) | 原点 | 用途 |
|----------|----------|------|------|
| `test.yaml/.pgm` | ~60×40 | [-30, -20] | 默认测试地图 |
| `room.yaml/.pgm` | ~36×34 | [-18, -17.1] | 室内房间地图 |
| `world.yaml/.pgm` | ~71×22 | [-35.5, -10.9] | 大范围世界地图 |

所有地图统一参数：分辨率 0.05 m/px，三值模式（trinary），占用阈值 0.65，空闲阈值 0.25。

如需自定义地图，可使用 SLAM Toolbox 或 Cartographer 建图后保存：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -p save_map_timeout:=10.0
```

---

## 常用调试命令速查

### 话题检查

```bash
# 查看当前所有活跃话题
ros2 topic list

# 检查速度指令是否到达
ros2 topic echo /cmd_vel

# 检查里程计输出
ros2 topic echo /odom

# 检查关节状态（确认 6 个关节都在发布）
ros2 topic echo /joint_states

# 检查 2D 激光雷达扫描
ros2 topic echo /scan

# 检查 3D 点云
ros2 topic echo /points_raw --field header

# 检查自研栅格地图信息
ros2 topic echo /combined_grid --field info

# 检查转向角监测
ros2 topic echo /vehicle/steering_angle
```

### TF 检查

```bash
# 查看完整 TF 树
ros2 run tf2_tools view_frames

# 检查两个坐标系间变换
ros2 run tf2_ros tf2_echo odom base_footprint

# 实时查看 TF（RViz 中可视化）
ros2 run rviz2 rviz2

# 检查特定坐标系的 TF 是否存在
ros2 run tf2_ros tf2_echo map odom
```

### 节点管理

```bash
# 查看当前运行节点
ros2 node list

# 查看节点的话题订阅/发布
ros2 node info /ackermann_steering_angle

# 手动发送速度指令测试（谨慎使用）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}"

# 手动发送目标点（自研导航方案）
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}"
```

### Gazebo 调试

```bash
# 检查 Gazebo 模型列表
gz model -l

# 检查 Gazebo 话题
gz topic -l

# 重置 Gazebo 世界
gz world -r

# 查看 Gazebo 日志级别
gz msg -e
```

### Nav2 调试

```bash
# 检查 Nav2 生命周期节点状态
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server

# 手动清除代价地图
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap

# 查看当前 Nav2 行为树状态
ros2 topic echo /behavior_tree_log

# 发送导航目标（命令行方式）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0}}}}"
```

### 建图与地图保存

```bash
# 使用 SLAM Toolbox 在线建图
ros2 launch slam_toolbox online_async_launch.py

# 保存当前地图
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# 加载自定义地图启动 Nav2
ros2 launch su7ultra_navigation2 navigation2.launch.py map:=~/my_map.yaml
```

---

## 许可证

本项目基于 [Apache License 2.0](LICENSE) 开源。

**贡献者：**
- [Ming2zun](https://github.com/Ming2zun) — 核心开发与导航算法
- [喵了个水蓝蓝](https://www.bilibili.com/video/BV1kzEwzuEFw) — 教程与技术支持
