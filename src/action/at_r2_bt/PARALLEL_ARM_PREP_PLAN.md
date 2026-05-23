# MoveBetweenBlocks 并行机械臂预备 改造方案

## 背景

目前 `meilin_mission.xml` 主循环是：每次 `PopNextStep` 弹出一步，
按 `step_type` 路由到 `MoveBetweenBlocks` / `PickAtBlock` / `PushAtBlock`
子树。每一步只看得到当前步的数据，**看不到下一步是什么**。

诉求：当当前步是 `MOVE`、且**下一步**是 `PICK` 时，机器人爬/下楼梯
的同时把机械臂预先摆到合适的"准备抓取姿态"，以减少到位后的等待时间；
如果下一步不是 PICK 就保持机械臂不动。

进一步，要求"准备姿态"按下一步要抓的**方向**和**台阶高度差**自动选择，
并通过 yaml 命名位姿（`arm_position.yaml`）的方式传名字给机械臂。

## 总体设计

整体上 `MoveBetweenBlocks` 改成下面这种形状：

```
Sequence
  PublishGoal(prep_x, prep_y, prep_yaw)        ← 串行：先开到 prep
  ParallelAll(success_count=2, failure_count=1)
    Switch2 stair_dir:                          ← 楼梯腿（原逻辑）
      UP   → ClimbStair  height={abs_dh}
      DOWN → DescendStair height={abs_dh}
      默认 → AlwaysSuccess
    ForceSuccess                                ← 机械臂腿（兜底，不拖累 MOVE）
      Switch2 next_step_type:
        PICK →
          Sequence
            SelectArmPrepName next_from_id={next_from_id}
                              next_target_id={next_target_id}
                              arm_prep_name="{arm_prep_name}"
            ArmMoveNamed name="{arm_prep_name}"
        默认 → AlwaysSuccess
```

主树在 `PopNextStep` 之后多一句 `PeekNextStep`，把"下一步"的字段写到
黑板（前缀 `next_*`），供 `MoveBetweenBlocks` 子树消费。

## 新增的 BT 节点

### 1. `PeekNextStep`（SyncActionNode）

只读不弹。看共享 `plan` 队首，把字段写黑板。`Pop` 已经弹掉了当前步，
因此 `PeekNextStep` 看到的真就是"下一步"。

- 输入：`plan`（`std::shared_ptr<vector<PlanStep>>`，黑板）
- 输出：
  - `next_step_type`（`MOVE` / `PICK` / `PUSH` / `NONE`）
  - `next_from_id`、`next_target_id`
  - `next_cube_x`、`next_cube_y`、`next_block_height`（备用）
- 队列空 → 全部置默认值，`next_step_type = "NONE"`，返回 SUCCESS
  （不影响 Repeat 计数）。

### 2. `SelectArmPrepName`（SyncActionNode）

读取 `block_blue.yaml`，按 `next_from_id`、`next_target_id` 算出方向 +
高度差，输出对应的命名位姿字符串。

- 输入：
  - `next_from_id`、`next_target_id`
  - `block_yaml`（可选，默认从 `nav2_bt_publish_goal` 包的 share 取
    `yaml/block_blue.yaml`）
- 输出：`arm_prep_name`（字符串）

#### 计算规则

设：
```
from   = blocks[next_from_id]    // {x, y, height}
target = blocks[next_target_id]
dy = target.y - from.y
dh = target.height - from.height       // 单位 m
```

map 系约定：x+ = 前，y+ = 左。

**方向**：
```
direction = (dy < 0) ? "right" : "front"
```
即 target 在 from 的右边（y 更小）→ 右抓；其它（前 / 左）→ 前抓。

**高度桶**：把 `dh` 按 0.2 m 一格四舍五入到整数：
```
bucket = round(dh / 0.2)        // 取值 ∈ {-1, +1, +2}
```

**名字映射表**：

| direction | bucket | name |
|---|---|---|
| front | +2 | `pick_front_up400` |
| front | +1 | `pick_front_up200` |
| front | -1 | `pick_front_down200` |
| right | +1 | `pick_right_up200` |
| right | -1 | `pick_right_down200` |

不在表内的组合（如 `right_up400`、`bucket == 0`、id 不存在）→
`arm_prep_name = ""` 并返回 FAILURE。外层用 `ForceSuccess` 兜住，
机械臂不动。

### 3. `ArmMoveNamed`（StatefulActionNode）

按名字查 `arm_position.yaml`，把 6 个关节角拼成 `ArmTask::Goal{ task_id=1, data=joints }`
通过 action 客户端发出去。骨架完全照搬 `ArmMoveJointAction`，
只是把 `j1..j6` 6 个端口换成 `name` 一个端口。

- 输入：
  - `node`（黑板，ROS node）
  - `name`（字符串）
  - `server_name`（默认 `robotic_task`）
  - `task_id`（默认 1）
  - `arm_yaml`（可选，默认从 `nav2_bt_publish_goal` 包的 share 取
    `yaml/arm_ready_position.yaml`）
- 输出：`err_code`、`reason`

`arm_ready_position.yaml` 由 BT 包自身维护（与 arm_task 解耦），里面只列
本特性需要的 5 个 `pick_*` 命名位姿。

## 文件改动清单

### 新增

- `include/nav2_bt_publish_goal/peek_next_step_action.hpp`
- `src/peek_next_step_action.cpp`
- `include/nav2_bt_publish_goal/select_arm_prep_name_action.hpp`
- `src/select_arm_prep_name_action.cpp`
- `include/nav2_bt_publish_goal/arm_move_named_action.hpp`
- `src/arm_move_named_action.cpp`
- `yaml/block_blue.yaml`（场地方块表，副本，供 SelectArmPrepName 使用）
- `yaml/arm_ready_position.yaml`（5 个 `pick_*` 命名位姿）

### 修改

- `CMakeLists.txt`：
  - library 和 simple_bt_runner 都加 3 份新源文件
  - `find_package(yaml-cpp REQUIRED)`、`target_link_libraries(... yaml-cpp)`
  - `install(DIRECTORY yaml/ DESTINATION share/${PROJECT_NAME}/yaml)`
- `package.xml`：`<depend>yaml-cpp</depend>`（如未声明）
- `simple_bt_runner.cpp`：注册 3 个新节点
- `plugins/plugin_description.xml`：补 3 段描述
- `behavior_trees/meilin_mission.xml`：
  - 主树 `PopNextStep` 后插入 `PeekNextStep`
  - `MoveBetweenBlocks` 改为 `PublishGoal → ParallelAll(楼梯, 机械臂)`

## 边界情况

- **当前步是最后一步**（plan 队空）：`next_step_type = "NONE"`，
  `Switch2` 走默认分支，机械臂不动。
- **下一步是 PICK 但 SelectArmPrepName 算不出名字**：FAILURE 被
  `ForceSuccess` 吃掉，机械臂保持原地。导航/楼梯继续走。
- **arm action server 暂时不可用**：`ArmMoveNamed` FAILURE，
  同样被 ForceSuccess 兜住。
- **楼梯腿和机械臂腿一快一慢**：`ParallelAll(success=2, failure=1)`
  等两条都成功才退出；任一失败立刻 halt 另一条。

## 实施顺序

1. 写本文档（done）
2. 新增 3 对 hpp/cpp 文件
3. 改 CMakeLists.txt / package.xml / plugin_description.xml
4. 改 simple_bt_runner.cpp 注册新节点
5. 改 meilin_mission.xml
6. 在 arm_position.yaml 占位 5 个 `pick_*`
7. `colcon build --packages-select nav2_bt_publish_goal` 验证编译

实测 / 实跑由用户在硬件上验证。
