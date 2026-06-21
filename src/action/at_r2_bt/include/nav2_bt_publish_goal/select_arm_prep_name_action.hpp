#ifndef NAV2_BT_PUBLISH_GOAL__SELECT_ARM_PREP_NAME_ACTION_HPP_
#define NAV2_BT_PUBLISH_GOAL__SELECT_ARM_PREP_NAME_ACTION_HPP_

#include <map>
#include <set>
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
 *                    Empty/unset → defaults to share/at_r2_bt/yaml/block_blue.yaml
 *   - arm_yaml       (string, optional) absolute path to arm_ready_position.yaml.
 *                    Empty/unset → defaults to share/at_r2_bt/yaml/arm_ready_position.yaml.
 *                    The set of pose names defined here decides whether a "left"
 *                    grab is available for a given height bucket.
 *
 * Output port:
 *   - arm_prep_name  (string)   one of the names defined in arm_yaml,
 *       e.g. pick_left_up200 / pick_left_down200 /
 *            pick_front_up400 / pick_front_up200 / pick_front_down200
 *
 * Selection rule:
 *   The arm can only physically reach to the left of the body, so:
 *     - When the target is on the left in map frame (dy > 0) AND the matching
 *       `pick_left_<suffix>` exists in arm_yaml → use it.
 *     - Otherwise fall back to `pick_front_<suffix>` if it exists.
 *     - If neither exists → FAILURE (with empty arm_prep_name).
 *
 *   dy = target.y - from.y       (map frame, x+ = front, y+ = left)
 *   bucket = round((target.height - from.height) / 0.2)
 *   suffix:  +1 → up200,  +2 → up400,  -1 → down200,  -2 → down400
 *            (bucket == 0 or out of range → FAILURE)
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

  // Lazy-loaded block table cache. Re-loaded when `block_yaml` input changes.
  std::map<int32_t, BlockInfo> blocks_;
  std::string loaded_block_yaml_path_;

  // Lazy-loaded set of arm pose names declared in arm_yaml's `arm_positions:`.
  // Used to decide whether a "left" grab is available for a given height bucket.
  std::set<std::string> arm_pose_names_;
  std::string loaded_arm_yaml_path_;

  bool ensureBlocksLoaded(const std::string & yaml_path);
  bool ensureArmPosesLoaded(const std::string & yaml_path);
};

}  // namespace nav2_bt_publish_goal

#endif  // NAV2_BT_PUBLISH_GOAL__SELECT_ARM_PREP_NAME_ACTION_HPP_
