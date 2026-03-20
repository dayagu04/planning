#pragma once
#include <string>

#include "pose2d.h"

namespace planning {

class AstarDecider {
 public:
  AstarDecider() = default;

  virtual ~AstarDecider() = default;

  const std::string &Name() const;

  virtual void Process(const Pose2f &start, const Pose2f &end);

 protected:
  std::string name_;
  Pose2f start_;
  Pose2f end_;
};

}  // namespace planning