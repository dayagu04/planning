#pragma once
#include <string>

#include "pose2d.h"

namespace planning {

class AstarDecider {
 public:
  AstarDecider() = default;

  virtual ~AstarDecider() = default;

  const std::string &Name() const;

  virtual void Process(const Pose2D &start, const Pose2D &end);

 protected:
  std::string name_;
  Pose2D start_;
  Pose2D end_;
};

}  // namespace planning