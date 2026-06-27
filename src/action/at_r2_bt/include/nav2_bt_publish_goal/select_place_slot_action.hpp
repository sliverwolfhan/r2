#ifndef NAV2_BT_PUBLISH_GOAL__SELECT_PLACE_SLOT_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SELECT_PLACE_SLOT_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace nav2_bt_publish_goal
{

/**
 * SelectPlaceSlot: 连续放置多个 KFS 时, 每被 tick 一次就"选定下一个放置 slot",
 * 把它的机器人预备点位姿 (slot_<n>_x / _y / _yaw) 拷到无前缀工作键
 * (slot_x / slot_y / slot_yaw), 供后续 B 段 (导航 + 机械臂关节摆位) 子树读取。
 *
 * 计数器存在黑板键 "place_slot_iter" 里(在所有 SelectPlaceSlot 实例间共享),
 * 第 k 次被 tick 选定的 slot 序号为
 *   index = start + k        (k = 0, 1, 2, ...)
 * 这样即使 XML 里写了多个 <SelectPlaceSlot> (例如 A 段 + C 段循环内), 它们
 * 共用同一个递增序号, 顺序对应 yaml 的 place_slot_priority。
 *
 * 这些 slot_<n>_ 键由 simple_bt_runner.cpp 从 place_kfs_params.yaml 预加载。
 *
 * 输入端口:
 *   - start (int, 默认 1) 起始 slot 序号(对应 place_slot_priority 第几个, 1-based)
 *
 * 返回:
 *   - SUCCESS: 成功拷贝该 slot 的全部参数到工作键
 *   - FAILURE: 黑板里找不到对应 slot_<n>_ 键(序号越界或未加载)
 */
class SelectPlaceSlotAction : public BT::SyncActionNode
{
public:
  SelectPlaceSlotAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool copyDouble(const std::string & prefix, const std::string & suffix);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SELECT_PLACE_SLOT_ACTION_HPP_
