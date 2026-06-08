# SetMppiParams 行为树节点使用说明

`SetMppiParams` 是 `at_r2_bt` 包里的一个 BehaviorTree.CPP 自定义动作节点，用来在行为树运行过程中动态修改 Nav2 `controller_server` 上 MPPI 控制器插件的参数。

它不是单独启动的 ROS2 node，而是写在 BT XML 里，由 `simple_bt_runner` 或 Nav2 BT Navigator 执行。

源码位置：

- `src/set_mppi_params_action.cpp`
- `include/nav2_bt_publish_goal/set_mppi_params_action.hpp`

## 1. 它做什么

节点执行时会通过 `rclcpp::AsyncParametersClient` 调用目标节点的 `set_parameters` 服务，例如默认目标是：

```text
/AT_R2/controller_server
```

MPPI 参数名会按下面规则拼出来：

```text
<plugin_prefix>.<参数名>
```

默认 `plugin_prefix="FollowPath"`，所以：

```xml
<SetMppiParams node="{node}" vx_std="0.5" goal_angle_weight="6.0"/>
```

实际会设置：

```text
FollowPath.vx_std = 0.5
FollowPath.GoalAngleCritic.cost_weight = 6.0
```

未填写的端口会被忽略；只会设置你在 XML 里写出来的参数。

## 2. 最小使用例子

在行为树 XML 中，把 `SetMppiParams` 放在需要改变 MPPI 行为的位置，例如导航前先把速度采样噪声和目标角度权重调大：

```xml
<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <SetMppiParams node="{node}"
                     controller_node="/AT_R2/controller_server"
                     plugin_prefix="FollowPath"
                     vx_std="0.5"
                     vy_std="0.5"
                     wz_std="0.5"
                     goal_angle_weight="6.0"
                     goal_angle_threshold="2.0"
                     wait_after_set="0.3"/>

      <PublishGoal node="{node}"
                   x="3.0" y="-1.0" yaw="0.0"
                   frame_id="map"
                   goal="{goal}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

说明：

- `node="{node}"`：必填，从黑板拿到当前 ROS2 node。`simple_bt_runner` 已经把它放进黑板。
- `controller_node`：要修改参数的 controller server 名称，默认是 `/AT_R2/controller_server`。
- `plugin_prefix`：MPPI controller 插件实例名，通常是 Nav2 参数文件里的 `FollowPath`。
- `wait_after_set`：参数设置请求完成后再等待多久返回 `SUCCESS`，默认 `0.3` 秒。

## 3. 实战例子：窄路/对准阶段临时调参，再恢复

常见用法是在某段路径前临时调整 MPPI 参数，过完之后再恢复。

```xml
<Sequence name="NarrowPassage">
  <!-- 进入窄路前：降低膨胀半径，并让 MPPI 更重视目标角度 -->
  <SetCostmapInflation node="{node}"
                       local_radius="0.15"
                       global_radius="0.15"
                       local_cost_scaling_factor="5.0"
                       global_cost_scaling_factor="5.0"/>

  <SetMppiParams node="{node}"
                 controller_node="/AT_R2/controller_server"
                 plugin_prefix="FollowPath"
                 goal_angle_weight="6.0"
                 goal_angle_threshold="2.0"
                 vx_std="0.5"
                 vy_std="0.5"
                 wz_std="0.5"
                 wait_after_set="0.3"/>

  <PublishGoal node="{node}"
               x="1.3" y="-5.2" yaw="1.57"
               frame_id="map"
               goal="{goal}"/>

  <!-- 过完窄路后：恢复较普通的参数 -->
  <SetCostmapInflation node="{node}"
                       local_radius="0.3"
                       global_radius="0.3"
                       local_cost_scaling_factor="1.0"
                       global_cost_scaling_factor="4.0"/>

  <SetMppiParams node="{node}"
                 controller_node="/AT_R2/controller_server"
                 plugin_prefix="FollowPath"
                 goal_angle_weight="5.0"
                 goal_angle_threshold="0.3"
                 vx_std="0.45"
                 vy_std="0.45"
                 wz_std="0.45"
                 wait_after_set="0.3"/>
</Sequence>
```

项目里的 `behavior_trees/r2_bt.xml` 已经有类似用法，可以参考其中的 `SetMppiParams` 调用。

## 4. 怎么运行

### 4.1 编译

在工作空间根目录编译 `at_r2_bt`：

```bash
cd /home/zk/wulin_r2
colcon build --packages-select at_r2_bt
source install/setup.bash
```

### 4.2 准备行为树 XML

把上面的示例 XML 放到：

```text
/home/zk/wulin_r2/src/action/at_r2_bt/behavior_trees/example_set_mppi_params.xml
```

然后重新编译或至少重新安装资源文件：

```bash
cd /home/zk/wulin_r2
colcon build --packages-select at_r2_bt
source install/setup.bash
```

### 4.3 启动 Nav2 / controller_server

需要保证目标 controller server 已经启动，并且参数服务存在：

```bash
ros2 node list | grep controller_server
```

默认这个节点会找：

```text
/AT_R2/controller_server
```

如果你的实际节点名不同，就在 XML 里改 `controller_node`。

### 4.4 运行行为树

使用项目里的 `simple_bt_runner`：

```bash
cd /home/zk/wulin_r2
source install/setup.bash
ros2 run at_r2_bt simple_bt_runner example_set_mppi_params.xml
```

如果直接用现有主树：

```bash
ros2 run at_r2_bt simple_bt_runner r2_bt.xml
```

## 5. 如何确认参数确实改了

运行行为树前后可以查看参数：

```bash
ros2 param get /AT_R2/controller_server FollowPath.vx_std
ros2 param get /AT_R2/controller_server FollowPath.vy_std
ros2 param get /AT_R2/controller_server FollowPath.wz_std
ros2 param get /AT_R2/controller_server FollowPath.GoalAngleCritic.cost_weight
ros2 param get /AT_R2/controller_server FollowPath.GoalAngleCritic.threshold_to_consider
```

也可以列出相关参数：

```bash
ros2 param list /AT_R2/controller_server | grep FollowPath
```

如果行为树执行时看到类似日志，说明请求已经发出：

```text
SetMppiParams: requested 5 params on '/AT_R2/controller_server' (prefix 'FollowPath'), waiting 0.30s after set
SetMppiParams: done
```

## 6. 常用端口

### 基本 MPPI 参数

| XML 端口 | 实际参数名 |
|---|---|
| `time_steps` | `FollowPath.time_steps` |
| `model_dt` | `FollowPath.model_dt` |
| `batch_size` | `FollowPath.batch_size` |
| `vx_std` | `FollowPath.vx_std` |
| `vy_std` | `FollowPath.vy_std` |
| `wz_std` | `FollowPath.wz_std` |
| `vx_max` | `FollowPath.vx_max` |
| `vx_min` | `FollowPath.vx_min` |
| `vy_max` | `FollowPath.vy_max` |
| `wz_max` | `FollowPath.wz_max` |
| `ax_max` | `FollowPath.ax_max` |
| `ax_min` | `FollowPath.ax_min` |
| `ay_max` | `FollowPath.ay_max` |
| `ay_min` | `FollowPath.ay_min` |
| `az_max` | `FollowPath.az_max` |
| `iteration_count` | `FollowPath.iteration_count` |
| `temperature` | `FollowPath.temperature` |
| `gamma` | `FollowPath.gamma` |
| `reset_period` | `FollowPath.reset_period` |
| `motion_model` | `FollowPath.motion_model` |
| `regenerate_noises` | `FollowPath.regenerate_noises` |

如果 `plugin_prefix` 不是 `FollowPath`，实际参数名前缀会跟着变，例如 `plugin_prefix="GridBased"` 时会变成 `GridBased.vx_std`。

### 常用 Critic 参数

| XML 端口 | 实际参数名 |
|---|---|
| `goal_weight` | `FollowPath.GoalCritic.cost_weight` |
| `goal_threshold` | `FollowPath.GoalCritic.threshold_to_consider` |
| `goal_angle_weight` | `FollowPath.GoalAngleCritic.cost_weight` |
| `goal_angle_threshold` | `FollowPath.GoalAngleCritic.threshold_to_consider` |
| `costcritic_weight` | `FollowPath.CostCritic.cost_weight` |
| `costcritic_critical_cost` | `FollowPath.CostCritic.critical_cost` |
| `costcritic_consider_footprint` | `FollowPath.CostCritic.consider_footprint` |
| `pathalign_weight` | `FollowPath.PathAlignCritic.cost_weight` |
| `pathalign_threshold` | `FollowPath.PathAlignCritic.threshold_to_consider` |
| `pathfollow_weight` | `FollowPath.PathFollowCritic.cost_weight` |
| `pathfollow_threshold` | `FollowPath.PathFollowCritic.threshold_to_consider` |
| `pathangle_enabled` | `FollowPath.PathAngleCritic.enabled` |
| `pathangle_weight` | `FollowPath.PathAngleCritic.cost_weight` |
| `preferforward_enabled` | `FollowPath.PreferForwardCritic.enabled` |
| `preferforward_weight` | `FollowPath.PreferForwardCritic.cost_weight` |
| `twirling_enabled` | `FollowPath.TwirlingCritic.enabled` |
| `twirling_weight` | `FollowPath.TwirlingCritic.twirling_cost_weight` |

### Goal checker 参数

这几个参数不加 `plugin_prefix`，源码里直接映射到 `general_goal_checker`：

| XML 端口 | 实际参数名 |
|---|---|
| `goal_x_tolerance` | `general_goal_checker.x_goal_tolerance` |
| `goal_y_tolerance` | `general_goal_checker.y_goal_tolerance` |
| `goal_yaw_tolerance` | `general_goal_checker.yaw_goal_tolerance` |
| `goal_stable_duration` | `general_goal_checker.stable_duration` |

示例：

```xml
<SetMppiParams node="{node}"
               controller_node="/AT_R2/controller_server"
               goal_x_tolerance="0.10"
               goal_y_tolerance="0.10"
               goal_yaw_tolerance="0.20"
               goal_stable_duration="0.5"/>
```

## 7. 注意事项

1. `node="{node}"` 必须有，否则节点返回 `FAILURE`。
2. 如果一个可设置参数都没写，节点会打印 warning，然后直接返回 `SUCCESS`。
3. 如果 `/set_parameters` 服务不可用，或者某个参数设置失败，源码当前策略是打印 `WARN`，但行为树仍返回 `SUCCESS`，不会中断后续动作。
4. 参数类型要和源码端口类型一致，例如 `vx_std` 是 double，`time_steps` 是 int，`regenerate_noises` 是 bool。
5. 这个节点只改运行时参数；如果 controller server 重启，参数会恢复到启动 YAML 里的值。
