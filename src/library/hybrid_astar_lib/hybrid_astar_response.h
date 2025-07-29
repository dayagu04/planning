#pragma once

#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "node3d.h"

namespace planning {

// todo, need to check the response is whether an expected result for your
// request.
struct AstarResponse {
  AstarRequest request;

  // local frame
  HybridAStarResult result;
  // local frame
  std::vector<AStarPathPoint> first_seg_path;

  // if published path steering wheel change too much, true.
  // left turn->right turn: true
  // left turn->straight: true
  bool kappa_change_too_much;

  const float GetFirstPathLength() const {
    return first_seg_path.empty() ? 0.0f : first_seg_path.back().accumulated_s;
  }

  void Clear() {
    request.Clear();
    result.Clear();
    first_seg_path.clear();
    return;
  }
};

const bool IsResponseNice(const AstarRequest& request,
                          const AstarResponse& response);

}  // namespace planning