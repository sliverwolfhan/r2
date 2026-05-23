#pragma once

#include "task/base_task.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>


class MoveKFS : public BaseTask {
public:
    MoveKFS(Robot* context, const std::string name);
    ~MoveKFS() override;
    std::string process(const std::string last_task_name) override;

private:
};
