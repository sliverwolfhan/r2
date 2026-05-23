#include "r2_meilin_planner/block_table.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace r2_planner {

bool BlockTable::loadFromYaml(const std::string & path, std::string * err)
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root["blocks"]) {
      if (err) {
        *err = "missing top-level key 'blocks' in " + path;
      }
      return false;
    }
    entries_.clear();
    for (auto it = root["blocks"].begin(); it != root["blocks"].end(); ++it) {
      const int id = it->first.as<int>();
      const YAML::Node & b = it->second;
      BlockEntry e;
      e.x = b["x"].as<double>(0.0);
      e.y = b["y"].as<double>(0.0);
      e.height = b["height"].as<double>(0.0);
      e.cube_x = b["cube_x"].as<double>(e.x);
      e.cube_y = b["cube_y"].as<double>(e.y);
      entries_[id] = e;
    }
    return true;
  } catch (const std::exception & ex) {
    if (err) {
      std::ostringstream oss;
      oss << "failed to parse " << path << ": " << ex.what();
      *err = oss.str();
    }
    return false;
  }
}

bool BlockTable::has(int id) const
{
  return entries_.find(id) != entries_.end();
}

const BlockEntry & BlockTable::at(int id) const
{
  auto it = entries_.find(id);
  if (it == entries_.end()) {
    throw std::out_of_range("BlockTable: id not found: " + std::to_string(id));
  }
  return it->second;
}

}  // namespace r2_planner
