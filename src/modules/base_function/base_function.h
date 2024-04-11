#pragma once

#include <memory>

#include "base_task_pipeline.h"
#include "session.h"

namespace planning {

class BaseFunction {
 public:
  explicit BaseFunction(framework::Session *session);

  virtual ~BaseFunction() = default;

  virtual bool Reset() = 0;

  virtual bool Plan() = 0;

 protected:
  framework::Session *session_ = nullptr;

  std::unique_ptr<BaseTaskPipeline> task_pipeline_;
};

}  // namespace planning