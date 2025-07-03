#include "astar_decider.h"

namespace planning {
const std::string &AstarDecider::Name() const { return name_; }

void AstarDecider::Process(const Pose2f &start, const Pose2f &end) {
  start_ = start;
  end_ = end;

  return;
}

}  // namespace planning