#pragma once

#include "base_function.h"
#include "base_task_pipeline.h"
#include "session.h"

namespace planning {

class SccFunction : public BaseFunction {
 public:
  SccFunction(framework::Session *session);

  virtual ~SccFunction() = default;

  bool Reset() override;

  bool Plan() override;
};

}  // namespace planning