#ifndef NAV2_BT_PUBLISH_GOAL__WAIT_ARM_IDLE_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__WAIT_ARM_IDLE_ACTION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"

#include "nav2_bt_publish_goal/arm_job_runner.hpp"

namespace nav2_bt_publish_goal
{

// 阻塞直到 ArmJobRunner 队列空（同步点，用在 PICK 的 ArmTask 之前）。
// 在 BT tick 循环里轮询 runner->busy()，所以底盘那侧的 tick 不受影响。
//
// 输入端口：
//   - arm_runner_key (string, 默认 "arm_runner")
//   - timeout        (double, 默认 0 = 永久等)
//
// 返回：idle = SUCCESS；超时 = FAILURE。
class WaitArmIdleAction : public BT::StatefulActionNode
{
public:
  WaitArmIdleAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<ArmJobRunner> runner_;
  double timeout_seconds_{0.0};
  std::chrono::steady_clock::time_point deadline_;
  bool has_deadline_{false};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__WAIT_ARM_IDLE_ACTION_HPP_
