
#pragma once

#include "base_function.h"
#include "base_task_pipeline.h"
#include "session.h"

namespace planning {

class NsaFunction : public BaseFunction {
 public:
  NsaFunction(framework::Session *session);

  virtual ~NsaFunction() = default;

  bool Reset() override;

  bool Plan() override;
};

}  // namespace planning