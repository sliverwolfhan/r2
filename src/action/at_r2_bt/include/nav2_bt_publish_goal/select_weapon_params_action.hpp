#ifndef NAV2_BT_PUBLISH_GOAL__SELECT_WEAPON_PARAMS_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SELECT_WEAPON_PARAMS_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace nav2_bt_publish_goal
{

/**
 * SelectWeaponParams: 连续抓取多个武器头时, 每被 tick 一次就"选定下一个武器",
 * 把它的抓取参数从黑板里带前缀的键 (w{n}_grasp_*) 拷到无前缀的工作键
 * (grasp_prep_x / grasp_prep_y / ...), 供后续 GraspOneWeapon 子树读取。
 *
 * 配合 Repeat 装饰器使用:
 *   <Repeat num_cycles="{grasp_count}">
 *     <Sequence>
 *       <SelectWeaponParams start="{grasp_start}"/>
 *       <SubTree ID="GraspOneWeapon" _autoremap="true"/>
 *     </Sequence>
 *   </Repeat>
 *
 * 节点内部维护一个计数器 iter_(从 0 开始), 第 k 次被 tick 选定的武器序号为
 *   index = start + k        (k = 0, 1, 2, ...)
 * 即 start=1 时依次选 w1_/w2_/w3_..., start=4 时依次选 w4_/w5_/w6_...。
 *
 * 这些 w{n}_ 键由 simple_bt_runner.cpp 从 weapon_grasp_params.yaml 预加载。
 *
 * 输入端口:
 *   - start (int, 默认 1) 起始武器序号(对应 weapon_priority 第几个, 1-based)
 *
 * 返回:
 *   - SUCCESS: 成功拷贝该武器的全部参数到工作键
 *   - FAILURE: 黑板里找不到对应 w{n}_ 键(序号越界或未加载)
 */
class SelectWeaponParamsAction : public BT::SyncActionNode
{
public:
  SelectWeaponParamsAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // 把一个带前缀的 double 键拷到无前缀工作键; 找不到源键返回 false。
  bool copyDouble(const std::string & prefix, const std::string & suffix);

  // 已被 tick 的次数(决定本次选第几个武器), 跨 Repeat 周期累加。
  int iter_{0};
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SELECT_WEAPON_PARAMS_ACTION_HPP_
