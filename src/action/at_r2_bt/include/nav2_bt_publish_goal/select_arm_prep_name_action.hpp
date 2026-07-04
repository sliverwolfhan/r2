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
 *   - from_x_override / from_y_override / from_yaw_override (double, optional)
 *                    机器人抓/推这一块时的准备位姿 (prep, map frame)。由 PICK/PUSH 段传入。
 *                    三者齐备 → 把目标块变换到机器人系判定左/前抓(与朝向无关);
 *                    缺任一 → 退回 map 系 dy 判定(仅 yaw≈0 正确)。
 *   - block_yaml     (string, optional) absolute path to block_*.yaml.
 *                    Empty/unset → defaults to share/at_r2_bt/yaml/block_blue.yaml
 *   - arm_yaml       (string, optional) absolute path to arm_ready_position.yaml.
 *                    Empty/unset → defaults to share/at_r2_bt/yaml/arm_ready_position.yaml.
 *                    The set of pose names defined here decides whether a "left"
 *                    grab is available for a given height bucket.
 *
 * Output ports:
 *   - arm_prep_name    (string) one of the names defined in arm_yaml,
 *       e.g. pick_left_up200 / pick_left_down200 /
 *            pick_front_up400 / pick_front_up200 / pick_front_down200
 *   - arm_prep_is_low  (int)    1 if the chosen pose is `down200`/`down400`
 *       (arm hangs low, may scrape ground while chassis moves),
 *       0 otherwise. Always 0 on FAILURE. BT 可用此标志决定臂动作
 *       是否要等导航完成再下发。
 *
 * Selection rule:
 *   The arm can only physically reach to the left of the body, so:
 *     - When the target block's bearing falls in the left sector (45°~135°) AND the
 *       matching `pick_left_<suffix>` exists in arm_yaml → use it.
 *     - Otherwise fall back to `pick_front_<suffix>` if it exists.
 *     - If neither exists → FAILURE (with empty arm_prep_name).
 *
 *   bearing = atan2(ry, rx)      (机器人系: 0°=正前, +90°=正左, -90°=正右)
 *   (rx, ry) = 目标块在机器人坐标系下的前向/横向坐标。用 pick/push 传入的准备位姿
 *              (from_x/y/yaw_override) 把目标块从 map 变换到机器人系算得; 三者未齐则退回
 *              map 系差值 (rx=target.x-from.x, ry=target.y-from.y)(仅 yaw≈0 时正确)。
 *   bucket = round((target.height - from.height) / 0.2)   (height 始终取自块)
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
