#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace go2w {

struct TaskQueueValidationResult {
    bool valid = true;
    std::vector<std::string> errors;

    std::string summary() const;
};

TaskQueueValidationResult validateTaskQueue(const nlohmann::json& task_queue);

}  // namespace go2w
