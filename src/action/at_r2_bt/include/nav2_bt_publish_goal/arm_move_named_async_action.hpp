#ifndef NAV2_BT_PUBLISH_GOAL__ARM_MOVE_NAMED_ASYNC_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__ARM_MOVE_NAMED_ASYNC_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"

#include "nav2_bt_publish_goal/arm_job_runner.hpp"

namespace nav2_bt_publish_goal
{

// 把命名臂动作推进 ArmJobRunner 队列，立即返回 SUCCESS。
//
// 输入端口：
//   - arm_runner_key (string, 默认 "arm_runner")  从黑板取 runner 的 key
//   - pose_name      (string)
//   - task_id        (int32, 默认 1)
//   - duration       (double, 默认 3.0)
//   - arm_yaml       (string, 可选)               覆盖默认 YAML 路径
//
// 返回：入队成功 = SUCCESS；找不到 runner / pose 查不到 = FAILURE。
class ArmMoveNamedAsyncAction : public BT::SyncActionNode
{
public:
  ArmMoveNamedAsyncAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__ARM_MOVE_NAMED_ASYNC_ACTION_HPP_
