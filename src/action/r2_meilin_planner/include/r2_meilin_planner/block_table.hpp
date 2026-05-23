#ifndef R2_MEILIN_PLANNER_BLOCK_TABLE_HPP_
#define R2_MEILIN_PLANNER_BLOCK_TABLE_HPP_

#include <string>
#include <unordered_map>

namespace r2_planner {

struct BlockEntry
{
  double x = 0.0;
  double y = 0.0;
  double height = 0.0;
  double cube_x = 0.0;
  double cube_y = 0.0;
};

/**
 * map 系下的方块/节点表，从 yaml 加载。
 *   - id 1..12: 比赛场上的 12 个方块
 *   - id 0:    入口虚拟节点
 *   - id 13:   出口虚拟节点
 */
class BlockTable
{
public:
  BlockTable() = default;

  /// @brief 从 yaml 加载。文件格式见 r2_meilin_planner/config/blocks.yaml。
  /// @return 成功返回 true；失败时 err 包含原因。
  bool loadFromYaml(const std::string & path, std::string * err = nullptr);

  bool has(int id) const;
  const BlockEntry & at(int id) const;

  size_t size() const { return entries_.size(); }

private:
  std::unordered_map<int, BlockEntry> entries_;
};

}  // namespace r2_planner

#endif  // R2_MEILIN_PLANNER_BLOCK_TABLE_HPP_
