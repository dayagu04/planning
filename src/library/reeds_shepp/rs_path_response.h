#pragma once

#include "pose2d.h"
#include "reeds_shepp_interface.h"

namespace planning {

struct RSPathResponse {
  RSPath rs_path;

  bool has_response;
};

}  // namespace planning