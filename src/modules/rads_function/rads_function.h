#pragma once

#include "base_function.h"
#include "base_task_pipeline.h"
#include "session.h"

namespace planning {

class RadsFunction : public BaseFunction {
 public:
  RadsFunction(framework::Session *session);

  virtual ~RadsFunction() = default;

  bool Reset() override;

  bool Plan() override;
};

}  // namespace planning