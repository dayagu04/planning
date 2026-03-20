#pragma once
#include "common.h"

namespace planning {

enum MSDMapPOIInfoAvailableFlag {
  MSD_MAP_POI_INFO_MAP_POI_INFO_DATA = 1 << 0,
};

struct MSDMapPOIInfoData {
  double distance;
  double length;
  MSDMapPOIType type;
  std::vector<MSDPoint3f> key_points_enu;
};

struct MSDMapPOIInfo {
  size_t available;
  std::vector<MSDMapPOIInfoData> map_pois_data;
};
}  // namespace planning
