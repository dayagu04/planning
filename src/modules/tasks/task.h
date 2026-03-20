#pragma once

#include <assert.h>
#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "config_context.h"
#include "ego_planning_config.h"
#include "session.h"
#include "utils/kd_path.h"

namespace planning {

class Task {
 public:
  explicit Task() = default;
  explicit Task(const EgoPlanningConfigBuilder *config_builder,
                framework::Session *session);

  virtual ~Task() = default;

  const std::string &Name() const { return name_; }

  virtual bool Execute() = 0;

 protected:
  bool PreCheck();

 protected:
  std::string name_;

  framework::Session *session_;
};

}  // namespace planning
