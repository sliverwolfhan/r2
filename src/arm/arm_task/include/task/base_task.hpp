#pragma once

#include <string>

class Robot;

class BaseTask{
public:
    BaseTask(Robot* context,const std::string name): robot(context),task_name(name){}
    virtual ~BaseTask() = default;
    virtual std::string process(const std::string last_task_name)=0;
    Robot* robot;
    std::string task_name;
};
