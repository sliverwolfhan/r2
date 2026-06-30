#ifndef NAV2_BT_PUBLISH_GOAL__SELECT_KFS_PARAMS_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SELECT_KFS_PARAMS_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace nav2_bt_publish_goal
{

/**
 * SelectKfsParams: 连续抓取地上多个 KFS 块时, 每被 tick 一次就"选定下一个 KFS",
 * 把它的取块参数从黑板里带前缀的键 (kfs_<n>_prep_x / _prep_y / _prep_yaw / _place_x /
 * _place_y) 拷到无前缀的工作键 (kfs_prep_x / kfs_prep_y / kfs_prep_yaw / kfs_place_x /
 * kfs_place_y), 供后续 C 段 (导航 + 笛卡尔抓取) 子树读取。
 *
 * 节点内部维护一个计数器 iter_(从 0 开始), 第 k 次被 tick 选定的 KFS 序号为
 *   index = start + k        (k = 0, 1, 2, ...)
 *
 * 计数器存放在黑板键 "kfs_iter" 上, 跨实例共享, 这样 XML 里多处
 * <SelectKfsParams> 标签按 tick 顺序累加, 不会各自从 0 重数。
 *
 * 这些 kfs_<n>_ 键由 simple_bt_runner.cpp 从 place_kfs_params.yaml 预加载。
 *
 * 输入端口:
 *   - start (int, 默认 1) 起始 KFS 序号(对应 kfs_priority 第几个, 1-based)
 *
 * 返回:
 *   - SUCCESS: 成功拷贝该 KFS 的全部参数到工作键
 *   - FAILURE: 黑板里找不到对应 kfs_<n>_ 键(序号越界或未加载)
 */
class SelectKfsParamsAction : public BT::SyncActionNode
{
public:
  SelectKfsParamsAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool copyDouble(const std::string & prefix, const std::string & suffix);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SELECT_KFS_PARAMS_ACTION_HPP_
