#include "node_shrink_decider.h"

#include "astar_decider.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {

void NodeShrinkDecider::Process(const Pose2D &start, const Pose2D &end) {
  AstarDecider::Process(start, end);

  heading_shrink_.limit_search_heading_ = false;

  // heading shrink
  double theta_diff = start_.theta - end_.theta;
  theta_diff = IflyUnifyTheta(theta_diff, M_PI);

  double heading_check_bound = 150.0 * M_PI / 180.0;
  double heading_buffer = 20.0 * M_PI / 180.0;

  if (std::fabs(theta_diff) < heading_check_bound) {
    heading_shrink_.limit_search_heading_ = true;

    heading_shrink_.heading_low_bound_ =
        std::min(-heading_check_bound, start_.theta - heading_buffer);

    heading_shrink_.heading_up_bound_ =
        std::max(heading_check_bound, start_.theta + heading_buffer);
  }
  // if theta diff is big, do not shrink node heading.
  // else
  // {

  // }

  return;
}

bool NodeShrinkDecider::IsLegalForHeading(const double heading) {
  if (!heading_shrink_.limit_search_heading_) {
    return true;
  }

  double normalize_heading = IflyUnifyTheta(heading, M_PI);

  if (normalize_heading < heading_shrink_.heading_low_bound_ ||
      normalize_heading > heading_shrink_.heading_up_bound_) {
    return false;
  }

  return true;
}

}  // namespace planning