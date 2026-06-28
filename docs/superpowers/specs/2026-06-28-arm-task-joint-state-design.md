# arm_task 获取当前关节角度

## 背景

`arm_task` 的 `Robot` 节点无法获取当前关节角度。`calculate_duration(const std::vector<double>& target_joints)` 使用启发式（基于目标幅度）估算轨迹时间，注释中明确标注"获取当前关节角度"为待填充。

`arm_ctrl` 的 `rviz_joint_pub_` 已在 `/joint_states` 话题发布 `sensor_msgs::msg::JointState`。

## 方案

在 `Robot` 中订阅 `/joint_states`，缓存当前关节角度，用于 `calculate_duration` 的实际关节差计算。

## 变更

### `robot.hpp`

新增：
- `rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_`
- `std::array<double, 6> current_joints_{}`
- `bool has_joint_state_{false}`
- `void on_joint_state(const sensor_msgs::msg::JointState& msg)`

### `robot.cpp`

1. **构造函数创建订阅：**
   ```cpp
   joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
       "/joint_states", 10,
       std::bind(&Robot::on_joint_state, this, std::placeholders::_1));
   ```

2. **回调实现：**
   ```cpp
   void Robot::on_joint_state(const sensor_msgs::msg::JointState& msg) {
       for (size_t i = 0; i < 6 && i < msg.position.size(); ++i) {
           current_joints_[i] = msg.position[i];
       }
       has_joint_state_ = true;
   }
   ```

3. **重写 `calculate_duration(const std::vector<double>& target_joints)`：**
   - 有 joint_state 时：`max(|target[i] - current[i]|) / max_joint_velocity`，clamp 到 `[min_trajectory_duration_, max_trajectory_duration_]`
   - 未收到 joint_state 时：退回原启发式逻辑

## 边界情况

| 情况 | 处理 |
|------|------|
| 未收到 `/joint_states` | 退回原启发式（基于目标幅度） |
| joint 数量不匹配 | 取 `min(target.size(), msg.position.size())` |
| target 为空 | 保持现有逻辑返回默认时间 |

## 验证

执行 move_kfs 任务，日志应输出基于实际关节差的 duration 计算（而非之前的"显著变化关节数"启发式）。
