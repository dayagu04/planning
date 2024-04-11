
#pragma once

#include "base_function.h"
#include "base_task_pipeline.h"
#include "session.h"

namespace planning {

class HppFunction : public BaseFunction {
 public:
  HppFunction(framework::Session *session);

  virtual ~HppFunction() = default;

  bool Reset() override;

  bool Plan() override;
};

}  // namespace planning