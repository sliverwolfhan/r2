# 武林-R2

**全国大学生机器人大赛（RoboCon 2026 武林探秘）河北科技大学 A&T 战队 R2 上位机代码**

全向舵轮底盘 + 机械臂 协同控制 ROS2 工作空间。

> 制作人：旭光
>
> [English](README.en.md) | 中文

---

## 一、克隆与编译

### 1. 克隆仓库

```bash
git clone <你的仓库地址> AT_RC
cd AT_RC
```

### 2. 安装依赖

确保已安装 ROS2 Humble，然后在工作空间根目录执行：

```bash
# 系统依赖
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# rosdep 第一次使用需要初始化（已初始化过可跳过）
sudo rosdep init || true
rosdep update

# 自动安装 src/ 下所有包声明的依赖
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

如果要用到的第三方依赖（如 `livox_ros_driver2`、`mujoco`、`yaml-cpp` 等）有特殊安装步骤，请参考各子包内的 README。

### 3. 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 第一次编译可能耗时较长。后续修改单个包可用 `colcon build --packages-select <pkg_name>` 加速。

---

## 二、启动脚本

工作空间根目录提供了 4 个启动脚本，统一支持 `-b` 编译选项和参数透传。

通用参数：

| 参数 | 说明 |
| --- | --- |
| `-b, --build` | 启动前先 `colcon build` 整个工作空间 |
| `-p, --packages "pkg1 pkg2"` | 仅编译指定的包后启动 |
| `-h, --help` | 显示帮助 |
| `-- <args>` | `--` 后的参数原样透传给 `ros2 run` / `ros2 launch` |

### 1. `run_navigation.sh` — 启动导航系统

启动 `at_r2_nav_bringup at_navigation_launch.py`，包含 Nav2、定位、costmap、行为树等导航全栈。

```bash
./run_navigation.sh                    # 直接启动
./run_navigation.sh -b                 # 编译整个工作空间后启动
./run_navigation.sh -p "at_r2_nav_bringup at_r2_bt"  # 仅编译指定包后启动
./run_navigation.sh -- use_sim_time:=true  # 透传 launch 参数
```

### 2. `run_bt_runner.sh` — 启动行为树执行器

启动 `at_r2_bt simple_bt_runner`，运行抓取/搬运任务的行为树（BT）。第一个非选项参数可指定要执行的 BT XML 文件。

```bash
./run_bt_runner.sh                       # 默认行为树
./run_bt_runner.sh grasp_head.xml        # 指定行为树
./run_bt_runner.sh -b grasp_head.xml     # 编译后用指定 BT 启动
./run_bt_runner.sh meilin_mission.xml    # 切到梅林任务
./run_bt_runner.sh -- --ros-args -p tf_namespace:=AT_R2  # 透传 ROS 参数
```

可用的行为树文件位于 `src/action/at_r2_bt/behavior_trees/`。

### 3. `run_plan_publisher.sh` — 启动任务计划发布器

启动 `r2_meilin_planner plan_yaml_publisher_node`，从 yaml 计划文件读取并按 plan 话题发布给行为树消费。

```bash
./run_plan_publisher.sh                  # 直接启动
./run_plan_publisher.sh -b               # 编译后启动
./run_plan_publisher.sh -p "r2_meilin_planner"
./run_plan_publisher.sh -- --ros-args -p plan_file:=/path/to/plan.yaml
```

计划 yaml 位于 `src/action/r2_meilin_planner/config/`。

### 4. `run_virtual_serial.sh` — 启动虚拟串口节点

启动 `virtual_serial_port virtual_serial_port_node`，用于在仿真或调试场景下模拟串口通信。

```bash
./run_virtual_serial.sh                  # 直接启动
./run_virtual_serial.sh -b               # 编译后启动
./run_virtual_serial.sh -- --ros-args -p baudrate:=115200  # 透传参数
```

---

## 三、目录结构与功能包说明

```
AT_RC/
├── src/
│   ├── action/        # 任务执行层：行为树、导航 bringup、任务计划、串口
│   ├── arm/           # 机械臂相关：驱动、解算、视觉、仿真控制
│   ├── location/      # 定位与建图：LIO、点云转换、重定位
│   ├── navigation/    # 导航算法：Nav2 插件、控制器、地形分析
│   └── robot/         # 机器人本体：底盘控制、URDF、SDF 工具
├── run_navigation.sh
├── run_bt_runner.sh
├── run_plan_publisher.sh
├── run_virtual_serial.sh
└── README.md
```

### `src/action/` — 任务执行层

| 包名 | 功能 |
| --- | --- |
| `at_r2_bt` | 基于 BehaviorTree.CPP 的任务行为树框架，包含自定义 BT 节点（PublishGoal、ArmTask、DockToWall、PreSteerAlign 等）和 BT XML、YAML 配置 |
| `at_r2_nav_bringup` | 导航系统总入口，集成 Nav2、定位、行为树等的 launch 与 yaml 配置 |
| `r2_meilin_planner` | 任务级 Planner：把 yaml 描述的整体抓取计划发布为 plan 话题，供 BT 消费 |
| `virtual_serial_port` | 虚拟串口节点，用于调试或仿真场景下替代真实串口通信 |

### `src/arm/` — 机械臂相关

| 包名 | 功能 |
| --- | --- |
| `arm` | 机械臂上层封装 |
| `arm_calc` | 机械臂运动学/逆解计算 |
| `arm_task` | 机械臂任务节点，提供 `robotic_task` action server，接收抓取/移动指令 |
| `dog_controller` | 四足/底盘相关控制（如有） |
| `launch_pack` | 整合用的 launch 集合 |
| `mujoco_ros2_control` | MuJoCo 仿真器与 ros2_control 的桥接 |
| `robot_driver` | 真实机器人硬件驱动 |
| `robot_interfaces` | 自定义消息/服务/action 接口（ArmTask 等） |
| `vision` | 视觉模块，相机检测目标位姿等 |

### `src/location/` — 定位与建图

| 包名 | 功能 |
| --- | --- |
| `livox_ros_driver2` | Livox 激光雷达 ROS2 驱动 |
| `point_lio` | 高精度激光惯性里程计（LIO） |
| `loam_interface` | LOAM 系列 SLAM 接口 |
| `pointcloud_to_laserscan` | 点云转 2D 激光扫描，供 Nav2 使用 |
| `sensor_scan_generation` | 传感器扫描数据生成与处理 |
| `small_gicp_relocalization` | 基于 small_gicp 的全局重定位 |
| `costmap_converter` | costmap → 多边形/线段，供 TEB 等 planner 使用 |

### `src/navigation/` — 导航算法

| 包名 | 功能 |
| --- | --- |
| `at_nav2_plugins` | 项目自定义的 Nav2 插件（goal_checker、controller 等） |
| `pb_omni_pid_pursuit_controller` | 全向底盘 PID 追踪控制器 |
| `teb_local_planner` | TEB 局部规划器 |
| `terrain_analysis` | 地形分析（楼梯、斜坡识别） |
| `terrain_analysis_ext` | 地形分析扩展模块 |

### `src/robot/` — 机器人本体

| 包名 | 功能 |
| --- | --- |
| `at_r2_control` | 底盘 ros2_control 配置，全向舵轮控制 |
| `at_r2_robot_description` | URDF 模型与 robot_state_publisher 配置 |
| `robot_resources` | 模型、贴图等资源文件 |
| `sdformat_tools` | SDF 模型工具 |

---

## 四、典型使用流程

通常需要打开 4 个终端，按顺序启动：

```bash
# 终端 1：导航 + 行为树框架
./run_navigation.sh

# 终端 2：虚拟串口（如果需要）
./run_virtual_serial.sh

# 终端 3：发布任务计划
./run_plan_publisher.sh

# 终端 4：执行行为树
./run_bt_runner.sh grasp_head.xml
```

修改代码后只需在对应启动脚本上加 `-b` 或 `-p "<pkg>"` 即可重新编译再启动。

---

> 制作人：旭光
