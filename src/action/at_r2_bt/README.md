# Nav2 BT Publish Goal

## 功能概述

这个包提供了两个自定义的Nav2行为树节点：

1. **PublishGoal**: 用于在行为树执行过程中将目标位置同时发布到ROS话题和黑板
2. **ClimbStair**: 用于控制机器人爬楼梯功能

### 主要特性

#### PublishGoal节点
- ✅ 从XML参数读取目标坐标（x, y, yaw）
- ✅ 发布目标到固定话题 `/goal_pose`
- ✅ 同时将目标写入黑板供Nav2使用
- ✅ 支持自定义坐标系（默认为"map"）
- ✅ 与Nav2无缝集成
- ✅ 支持Groot2可视化

#### ClimbStair节点
- ✅ 通过话题控制爬楼器
- ✅ 实时监听爬楼状态
- ✅ 支持成功/失败状态反馈
- ✅ 参数验证和错误处理
- ✅ 可与导航节点组合使用

## 安装说明

### 依赖项

- ROS2 Humble
- Nav2 Humble
- BehaviorTree.CPP v4

### 编译

```bash
cd ~/pb25_ws
colcon build --packages-select nav2_bt_publish_goal
source install/setup.bash
```

## 快速开始

### 1. 在Nav2配置中添加插件

编辑你的 `nav2_params.yaml` 文件，在 `bt_navigator` 的 `plugin_lib_names` 中添加：

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      # ... 其他Nav2插件 ...
      - nav2_bt_publish_goal  # 添加我们的插件
```

### 2. 在行为树中使用

创建一个行为树XML文件：

```xml
<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <!-- 发布目标 -->
      <PublishGoal x="2.0" y="3.0" yaw="0.0" goal="{goal}"/>
      <!-- 导航到目标 -->
      <NavigateToPose goal="{goal}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 3. 运行测试

```bash
# 启动导航栈
ros2 launch nav2_bringup navigation_launch.py

# 在另一个终端监听目标话题
ros2 topic echo /goal_pose
```

## 参数说明

### PublishGoal节点

| 参数名 | 类型 | 必需 | 默认值 | 描述 |
|--------|------|------|--------|------|
| x | double | 是 | - | X坐标（米） |
| y | double | 是 | - | Y坐标（米） |
| yaw | double | 是 | - | 偏航角（弧度） |
| frame_id | string | 否 | "map" | 参考坐标系 |
| goal | PoseStamped | 输出 | "{goal}" | 输出的目标位置 |

### ClimbStair节点

| 参数名 | 类型 | 必需 | 默认值 | 描述 |
|--------|------|------|--------|------|
| node | Node::SharedPtr | 是 | - | ROS2节点指针（从黑板获取） |
| height | double | 是 | - | 爬升高度（米，必须为正数） |
| climb_height | double | 输出 | - | 实际发布的爬升高度值 |

#### ClimbStair话题接口

**发布话题**:
- `/AT_R2/climb_stair` (std_msgs/Float64): 发布爬升高度命令

**订阅话题**:
- `/AT_R2/climber_status` (std_msgs/Int32): 接收爬楼器状态
  - 1: 正在爬楼 (CLIMBING)
  - 2: 爬楼成功 (SUCCESS)
  - 3: 爬楼失败 (FAILURE)

## 使用示例

### PublishGoal节点示例

#### 单点导航

```xml
<PublishGoal x="2.0" y="3.0" yaw="0.0" goal="{goal}"/>
<NavigateToPose goal="{goal}"/>
```

#### 多点巡航

```xml
<Sequence>
  <PublishGoal x="2.0" y="3.0" yaw="0.0" goal="{goal}"/>
  <NavigateToPose goal="{goal}"/>
  
  <PublishGoal x="4.0" y="5.0" yaw="1.57" goal="{goal}"/>
  <NavigateToPose goal="{goal}"/>
</Sequence>
```

#### 带恢复的导航

```xml
<RecoveryNode number_of_retries="3">
  <Sequence>
    <PublishGoal x="2.0" y="3.0" yaw="0.0" goal="{goal}"/>
    <NavigateToPose goal="{goal}"/>
  </Sequence>
  <Sequence name="Recovery">
    <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    <Spin spin_dist="1.57"/>
  </Sequence>
</RecoveryNode>
```

### ClimbStair节点示例

#### 简单爬楼

```xml
<ClimbStair height="0.3" node="{node}" climb_height="{actual_height}"/>
```

#### 带重试的爬楼

```xml
<Retry num_attempts="3">
  <ClimbStair height="0.3" node="{node}" climb_height="{actual_height}"/>
</Retry>
```

#### 导航+爬楼组合

```xml
<Sequence>
  <!-- 导航到楼梯前 -->
  <PublishGoal x="1.0" y="2.0" yaw="0.0" node="{node}" goal="{nav_goal}"/>
  
  <!-- 爬楼梯 -->
  <ClimbStair height="0.3" node="{node}" climb_height="{actual_height}"/>
  
  <!-- 继续导航 -->
  <PublishGoal x="3.0" y="4.0" yaw="1.57" node="{node}" goal="{nav_goal}"/>
</Sequence>
```

## 测试

### 测试ClimbStair节点

1. 启动测试模拟器（模拟爬楼控制器）:
```bash
cd ~/pb25_ws
source install/setup.bash
python3 test_climb_stair_node.py success  # 或 'failure' 测试失败场景
```

2. 在另一个终端运行行为树:
```bash
cd ~/pb25_ws
source install/setup.bash
ros2 run nav2_bt_publish_goal simple_bt_runner example_climb_stair.xml
```

3. 运行完整测试套件:
```bash
cd ~/pb25_ws
./run_climb_stair_tests.sh
```

## 故障排查

### 问题：插件未加载

**症状**：BT Navigator报错找不到PublishGoal节点

**解决方案**：
1. 确认已在 `nav2_params.yaml` 中添加插件
2. 检查编译是否成功：`ros2 pkg list | grep nav2_bt_publish_goal`
3. 确认已source工作空间：`source ~/pb25_ws/install/setup.bash`

### 问题：目标未发布

**症状**：`ros2 topic echo /goal_pose` 没有输出

**解决方案**：
1. 检查行为树是否正确执行
2. 查看BT Navigator日志是否有错误
3. 确认参数x, y, yaw都已正确设置

### 问题：导航失败

**症状**：目标发布了但机器人不移动

**解决方案**：
1. 确认Nav2导航栈已正确启动
2. 检查目标位置是否在地图范围内
3. 确认坐标系frame_id正确（通常为"map"）
4. 使用RViz查看发布的目标位置是否合理

## 常见问题FAQ

### Q: 可以修改话题名称吗？

A: 当前版本话题名称固定为 `/goal_pose`。如需修改，需要编辑源代码中的话题名称并重新编译。

### Q: 支持3D导航吗？

A: 当前版本只支持2D导航（z坐标固定为0）。如需3D导航，需要扩展节点以支持z坐标和roll/pitch角度。

### Q: 如何在Groot2中查看？

A: 在 `nav2_params.yaml` 中启用Groot2监控：

```yaml
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
```

然后启动Groot2并连接到 `localhost:1667`。

### Q: 可以使用相对坐标吗？

A: 当前版本只支持绝对坐标。相对坐标支持计划在未来版本中添加。

### Q: 性能如何？

A: 节点执行时间通常小于5ms，不会对行为树性能产生明显影响。

## 许可证

Apache License 2.0

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

如有问题，请在GitHub上提交Issue。
