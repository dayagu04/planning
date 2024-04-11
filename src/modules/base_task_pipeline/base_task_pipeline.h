#pragma once

#include <string>

#include "ego_planning_config.h"
#include "session.h"

namespace planning {

class BaseTaskPipeline {
 public:
  BaseTaskPipeline(const EgoPlanningConfigBuilder *config_builder,
                   framework::Session *session);

  virtual ~BaseTaskPipeline() = default;

  virtual bool Run() = 0;

 protected:
  void AddErrorInfo(const std::string &task_name);

 protected:
  framework::Session *session_ = nullptr;
};

}  // namespace planning