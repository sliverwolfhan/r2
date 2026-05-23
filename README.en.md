# WuLin-R2

**National University Robotics Competition (RoboCon 2026 - Wulin Adventure)
Hebei University of Science and Technology A&T Team - R2 High-Level Software**

A ROS2 workspace for coordinated control of an omnidirectional swerve-drive
chassis and a manipulator arm.

> Author: Xuguang
>
> English | [中文](README.md)

---

## 1. Clone and Build

### 1.1 Clone the repository

```bash
git clone <your-repo-url> AT_RC
cd AT_RC
```

### 1.2 Install dependencies

Make sure ROS2 Humble is installed, then run from the workspace root:

```bash
# System packages
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep on first use (skip if already initialized)
sudo rosdep init || true
rosdep update

# Install all package dependencies declared under src/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Some third-party dependencies (e.g. `livox_ros_driver2`, `mujoco`, `yaml-cpp`)
may have extra installation steps. See the README inside each sub-package if so.

### 1.3 Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> The first build may take a while. Use
> `colcon build --packages-select <pkg_name>` to rebuild a single package.

---

## 2. Launch Scripts

Four launch scripts are provided at the workspace root. They share the same
common options for build control and argument forwarding.

Common options:

| Option | Description |
| --- | --- |
| `-b, --build` | Run `colcon build` on the whole workspace before launching |
| `-p, --packages "pkg1 pkg2"` | Build only the listed packages, then launch |
| `-h, --help` | Show help |
| `-- <args>` | Forward everything after `--` to `ros2 run` / `ros2 launch` |

### 2.1 `run_navigation.sh` - Launch the navigation stack

Starts `at_r2_nav_bringup at_navigation_launch.py`, which brings up Nav2,
localization, costmaps and the behavior-tree framework.

```bash
./run_navigation.sh                                  # launch as-is
./run_navigation.sh -b                               # build then launch
./run_navigation.sh -p "at_r2_nav_bringup at_r2_bt"  # build selected packages
./run_navigation.sh -- use_sim_time:=true            # forward launch args
```

### 2.2 `run_bt_runner.sh` - Launch the behavior-tree runner

Starts `at_r2_bt simple_bt_runner` to execute pick-and-place behavior trees.
The first non-option argument selects which BT XML file to run.

```bash
./run_bt_runner.sh                       # default behavior tree
./run_bt_runner.sh grasp_head.xml        # specific BT file
./run_bt_runner.sh -b grasp_head.xml     # build then launch
./run_bt_runner.sh meilin_mission.xml    # switch to Meilin mission
./run_bt_runner.sh -- --ros-args -p tf_namespace:=AT_R2  # forward ROS args
```

Available BT XML files live under `src/action/at_r2_bt/behavior_trees/`.

### 2.3 `run_plan_publisher.sh` - Launch the plan publisher

Starts `r2_meilin_planner plan_yaml_publisher_node`, which reads a yaml plan
file and publishes it on the `plan` topic for the BT to consume.

```bash
./run_plan_publisher.sh                  # launch as-is
./run_plan_publisher.sh -b               # build then launch
./run_plan_publisher.sh -p "r2_meilin_planner"
./run_plan_publisher.sh -- --ros-args -p plan_file:=/path/to/plan.yaml
```

Plan yaml files live under `src/action/r2_meilin_planner/config/`.

### 2.4 `run_virtual_serial.sh` - Launch the virtual serial port

Starts `virtual_serial_port virtual_serial_port_node` to mock serial
communication during simulation or debugging.

```bash
./run_virtual_serial.sh                                       # launch as-is
./run_virtual_serial.sh -b                                    # build then launch
./run_virtual_serial.sh -- --ros-args -p baudrate:=115200     # forward ROS args
```

---

## 3. Directory Layout and Packages

```
AT_RC/
├── src/
│   ├── action/        # Task layer: behavior trees, nav bringup, planner, serial
│   ├── arm/           # Arm: drivers, kinematics, vision, simulation
│   ├── location/      # Localization: LIO, point-cloud tools, relocalization
│   ├── navigation/    # Navigation: Nav2 plugins, controllers, terrain analysis
│   └── robot/         # Robot body: chassis control, URDF, SDF tools
├── run_navigation.sh
├── run_bt_runner.sh
├── run_plan_publisher.sh
├── run_virtual_serial.sh
└── README.md
```

### `src/action/` - Task layer

| Package | Description |
| --- | --- |
| `at_r2_bt` | Behavior-tree framework built on BehaviorTree.CPP. Includes custom BT nodes (PublishGoal, ArmTask, DockToWall, PreSteerAlign, etc.) plus BT XML and YAML configs |
| `at_r2_nav_bringup` | Top-level entry point for navigation. Bundles Nav2, localization, BT runner launch and yaml configs |
| `r2_meilin_planner` | Task-level planner. Publishes the high-level grasp/move plan from yaml onto the `plan` topic for the BT to consume |
| `virtual_serial_port` | Virtual serial port node, used to mock the real serial link during debugging or simulation |

### `src/arm/` - Manipulator arm

| Package | Description |
| --- | --- |
| `arm` | High-level arm wrappers |
| `arm_calc` | Arm kinematics and IK solver |
| `arm_task` | Arm task node, exposes the `robotic_task` action server for grasp/move commands |
| `dog_controller` | Quadruped / chassis support controller (where applicable) |
| `launch_pack` | Aggregated launch files |
| `mujoco_ros2_control` | Bridge between MuJoCo simulator and ros2_control |
| `robot_driver` | Real-hardware driver |
| `robot_interfaces` | Custom messages / services / actions (ArmTask, etc.) |
| `vision` | Vision module: camera-based target pose detection |

### `src/location/` - Localization and mapping

| Package | Description |
| --- | --- |
| `livox_ros_driver2` | Livox LiDAR ROS2 driver |
| `point_lio` | High-accuracy LiDAR-Inertial Odometry |
| `loam_interface` | Adapter for LOAM-family SLAM |
| `pointcloud_to_laserscan` | Converts 3D point clouds to 2D laser scans for Nav2 |
| `sensor_scan_generation` | Sensor scan generation and processing |
| `small_gicp_relocalization` | Global relocalization via small_gicp |
| `costmap_converter` | Converts costmaps to polygons / line segments for planners such as TEB |

### `src/navigation/` - Navigation algorithms

| Package | Description |
| --- | --- |
| `at_nav2_plugins` | Project-specific Nav2 plugins (goal checkers, controllers, etc.) |
| `pb_omni_pid_pursuit_controller` | PID pursuit controller for the omnidirectional chassis |
| `teb_local_planner` | TEB local planner |
| `terrain_analysis` | Terrain analysis (stairs, ramps) |
| `terrain_analysis_ext` | Terrain analysis extensions |

### `src/robot/` - Robot body

| Package | Description |
| --- | --- |
| `at_r2_control` | ros2_control configuration for the omnidirectional swerve chassis |
| `at_r2_robot_description` | URDF model and robot_state_publisher configuration |
| `robot_resources` | Mesh, texture, and other model assets |
| `sdformat_tools` | SDF model utilities |

---

## 4. Typical Workflow

A standard run uses four terminals, started in order:

```bash
# Terminal 1: navigation + BT framework
./run_navigation.sh

# Terminal 2: virtual serial (if needed)
./run_virtual_serial.sh

# Terminal 3: publish the task plan
./run_plan_publisher.sh

# Terminal 4: execute the behavior tree
./run_bt_runner.sh grasp_head.xml
```

After editing code, simply add `-b` or `-p "<pkg>"` to the relevant launch
script to rebuild before launching.

---

> Author: Xuguang
