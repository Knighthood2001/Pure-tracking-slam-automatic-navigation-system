# Pure-tracking-slam-automatic-navigation-system
simulate，ros2，gazebo，navigation，slam

<!--
 * @作者: boxing
 * @b号: 喵了个水蓝蓝
 * @描述: README
-->
## 注意当前分支代码为humble版本

# 基于ROS2实现的差速机器人，slam（建图定位），路径规划（A*），导航控制（纯追踪）

![image](https://github.com/user-attachments/assets/baac6889-d251-4891-8d21-c47fa4b45a33)


# 还包含su7模型的阿克曼底盘
<img width="2344" height="1400" alt="image" src="https://github.com/user-attachments/assets/9a933d84-e864-4fbf-98d5-312a79838130" />

项目中`four_wheeled_vehicle`功能包就是包含su7模型的阿克曼底盘，你clone下来后，需要把`src/four_wheeled_vehicle/models/four_wheeled_vehicle/meshes/car.zip`解压缩一下（`car.dae`原始模型太大了）。这就省的你去网盘里面下载了。


当然，原作者把模型放在网盘中：
链接: https://pan.baidu.com/s/1geyZbNclzaeNOMe9bcLUvg?pwd=rwqb 提取码: rwqb


## 安装运行

```
git clone https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system.git
cd Pure-tracking-slam-automatic-navigation-system
colcon build --symlink-install
source install/setup.bash
```

## 项目架构

### 阿克曼车辆模型架构 (`src/four_wheeled_vehicle`)

```
four_wheeled_vehicle/
├── urdf/
│   └── vehicle/
│       ├── actuator/           # 执行器组件
│       │   ├── steering.urdf.xacro    # 转向关节宏定义
│       │   └── wheel.urdf.xacro       # 轮子宏定义
│       ├── sensor/             # 传感器组件
│       │   ├── lidar3d.urdf.xacro     # 3D激光雷达
│       │   ├── gps.urdf.xacro         # GPS传感器
│       │   ├── imu.urdf.xacro         # IMU传感器
│       │   └── camera.urdf.xacro      # 摄像头
│       ├── plugin/             # 控制插件
│       │   └── gazebo_control.xacro   # Gazebo控制插件
│       ├── base.urdf.xacro     # 基础底盘定义
│       └── vehicle.urdf.xacro  # 主入口文件
├── models/                     # 3D模型文件
├── launch/                     # 启动文件
├── src/                        # 源代码
└── worlds/                     # Gazebo世界文件
```

## 运行测试

### 差速仿真
1. 启动仿真
```bash
ros2 launch gazebo_modele gazebo.launch.py
```

2. 启动导航
```bash
ros2 launch nav_slam 2dpoints.launch.py
```
### su7仿真（阿克曼转向模型）
1. 启动仿真
```bash
# 方式一：使用gazebo_sim.launch.py（推荐）
ros2 launch four_wheeled_vehicle gazebo_sim.launch.py

# 方式二：使用vehicle_gazebo_ok.launch.py（原方式）
ros2 launch four_wheeled_vehicle vehicle_gazebo_ok.launch.py
```

2. 启动导航
```bash
ros2 launch nav_slam 2dpoints.launch.py
```

## 架构优化说明

### XACRO宏定义优势

1. **代码复用**：通过宏参数化实现轮子、转向等组件的复用
2. **模块化设计**：传感器、执行器、插件分离为独立文件
3. **可读性强**：主入口文件清晰展示组件组装关系
4. **易于维护**：修改单个组件不影响其他部分


## 演示视频
https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f

联系：clibang2022@163.com
