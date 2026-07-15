#ifndef NAV2_BT_PUBLISH_GOAL__GET_BLOCK_CENTER_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__GET_BLOCK_CENTER_ACTION_HPP_

#include <map>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace nav2_bt_publish_goal
{

/**
 * GetBlockCenter: 从 block_yaml 里查指定 block_id 的中心坐标 (map frame),
 * 供 BT 在转弯前先发一个"回中心 pivot"的 PublishGoal 使用。
 *
 * Input ports:
 *   - block_id   (int32_t)  节点 id (from_id 或 next_from_id 视调用点而定)
 *   - block_yaml (string, optional) block_*.yaml 绝对路径; 空则回退默认 block_red.yaml
 *
 * Output ports:
 *   - center_x   (double)   block 中心 x (map frame)
 *   - center_y   (double)   block 中心 y (map frame)
 */
class GetBlockCenterAction : public BT::SyncActionNode
{
public:
  GetBlockCenterAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct BlockInfo
  {
    double x{0.0};
    double y{0.0};
  };

  std::map<int32_t, BlockInfo> blocks_;
  std::string loaded_block_yaml_path_;

  bool ensureBlocksLoaded(const std::string & yaml_path);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__GET_BLOCK_CENTER_ACTION_HPP_
