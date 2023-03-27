#pragma once

#include <vector>

namespace planning {

enum MSDTargetPositionAvailableFlag {
  MSD_TARGET_POSITION_PARKING_TARGET_POSITION = 1 << 0,
};

enum MSDParkingTargetPositionAvailableFlag {
  MSD_PARKING_TARGET_POSITION_TARGET_MAP_POI = 1 << 0,
  MSD_PARKING_TARGET_POSITION_TRAVERSED_PARKING_LOTS = 1 << 1,
  MSD_PARKING_TARGET_POSITION_PARKING_OUT_POSITION = 1 << 2,
};

struct MSDTargetMapPOI {
  uint16_t id;
  MSDMapPOIType type;
  MSDPolygon3f position;
  double distance;
};

struct MSDTraversedParkingLot {
  uint16_t parking_lot_id;
  MSDPolygon3f parking_lot_position;
  MSDPoint3f parking_lot_center;
  MSDPoint3f parking_lot_entrance_center;
  double available_inner_width;
  double available_outter_width;
  uint16_t next_parking_lot_id;
  uint16_t previous_parking_lot_id;
  double left_extended_distance;
  double right_extended_distance;
  double score;
};

struct MSDParkingOutPosition {
  uint16_t source_parking_lot_id;
  MSDPolygon3f source_parking_lot_position;
  MSDPoint3f target_position;
  double target_heading_yaw;
};

struct MSDParkingTargetPosition {
  size_t available;
  MSDTargetMapPOI target_map_poi;
  std::vector<MSDTraversedParkingLot> traversed_parking_lots;
  MSDParkingOutPosition parking_out_position;
};

struct MSDTargetPosition {
  size_t available;
  MSDParkingTargetPosition parking_target_position;
};

} // namespace planning
