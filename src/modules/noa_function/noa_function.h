
#pragma once

#include "base_function.h"
#include "session.h"

namespace planning {

class NoaFunction : public BaseFunction {
 public:
  NoaFunction(framework::Session *session);

  virtual ~NoaFunction() = default;

  bool Reset() override;

  bool Plan() override;
};

}  // namespace planning