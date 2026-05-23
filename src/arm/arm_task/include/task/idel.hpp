#pragma once

#include "task/base_task.hpp"

class IdelTask :public BaseTask{
public:
    IdelTask(Robot* context,const std::string name);
    ~IdelTask() override;
    std::string process(const std::string last_task_name) override;

private:
    bool is_first_run = true;

};
