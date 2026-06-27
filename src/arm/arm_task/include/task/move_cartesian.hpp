#pragma once

#include "task/base_task.hpp"


class MoveCartesian : public BaseTask {
public:
    MoveCartesian(Robot* context, const std::string name);
    ~MoveCartesian() override;
    std::string process(const std::string last_task_name) override;

private:
};
