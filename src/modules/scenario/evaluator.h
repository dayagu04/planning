#pragma once

#include "scenario/ego_planning_candidate.h"

namespace planning {

class Evaluator {
 public:
  Evaluator(const EgoPlanningConfigBuilder *config_builder,
            framework::Frame *frame)
      : config_builder_(config_builder), frame_(frame) {}
  virtual ~Evaluator() = default;

 private:
  const EgoPlanningConfigBuilder *config_builder_ = nullptr;
  framework::Frame *frame_ = nullptr;
};

}  // namespace planning