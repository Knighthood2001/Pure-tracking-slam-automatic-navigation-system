## su7ultra模型放入gazebo默认模型目录
```
cd Pure-tracking-slam-automatic-navigation-system
cp -r src/su7ultra_description/models/* ~/.gazebo/models
```
修改一下`src/su7ultra_description/urdf/vehicle/base.urdf.xacro`和`src/su7ultra_description/urdf/vehicle/actuator/wheel.urdf.xacro`中的模型filename路径。




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