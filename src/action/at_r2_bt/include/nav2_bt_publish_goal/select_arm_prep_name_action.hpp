#ifndef NAV2_BT_PUBLISH_GOAL__SELECT_ARM_PREP_NAME_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SELECT_ARM_PREP_NAME_ACTION_HPP_

#include <map>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace nav2_bt_publish_goal
{

/**
 * SelectArmPrepName: pick which named arm preparation pose to use for the
 * upcoming PICK step.
 *
 * Input ports:
 *   - next_from_id   (int32_t)  the from-node id of the next step
 *   - next_target_id (int32_t)  the target-node id of the next step
 *   - block_yaml     (string, optional) absolute path to block_*.yaml.
 *                    Empty/unset → defaults to share/r2_meilin_planner/config/block_blue.yaml
 *
 * Output port:
 *   - arm_prep_name  (string)   one of:
 *       pick_front_up400 / pick_front_up200 / pick_front_down200
 *       pick_right_up200 / pick_right_down200
 *
 * Returns FAILURE (and writes empty arm_prep_name) when:
 *   - block_yaml load fails;
 *   - either id missing in the yaml;
 *   - the (direction, height_bucket) combination is not in the table
 *     (e.g. dh == 0, or right + up400, etc.).
 *
 * Direction rule (map frame, x+ = front, y+ = left):
 *   dy = target.y - from.y
 *   dy < 0  → "right"     (target on the right of from)
 *   else    → "front"     (target in front or to the left)
 *
 * Height-bucket rule (each step is 0.2 m on this map):
 *   dh = target.height - from.height
 *   bucket = round(dh / 0.2)        // ∈ {-1, +1, +2}
 */
class SelectArmPrepNameAction : public BT::SyncActionNode
{
public:
  SelectArmPrepNameAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  struct BlockInfo
  {
    double x{0.0};
    double y{0.0};
    double height{0.0};
  };

  // Lazy-loaded cache. Re-loaded when `block_yaml` input changes.
  std::map<int32_t, BlockInfo> blocks_;
  std::string loaded_yaml_path_;

  bool ensureLoaded(const std::string & yaml_path);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SELECT_ARM_PREP_NAME_ACTION_HPP_
