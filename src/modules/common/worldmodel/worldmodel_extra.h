#pragma once

#include <string>
#include <vector>

namespace planning {

enum MSDWorldModelExtraAvailableFlag {
  MSD_WORLD_MODEL_EXTRA_VERSION = 1 << 0,
  MSD_WORLD_MODEL_EXTRA_JSON = 1 << 1,
};

struct MSDWorldModelExtra {
  size_t available;
  std::string version;
  std::string json;
};

} // namespace planning
