#pragma once

#include "base_convert.h"
#include "c/ifly_parking_map_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::Coordinate &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.coordinate_type, ros_v.coordinate_type, type);
  convert(struct_v.llh, ros_v.llh, type);
  convert(struct_v.enu, ros_v.enu, type);
  convert(struct_v.boot, ros_v.boot, type);
}

template <typename T2>
void convert(iflyauto::AttributeId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.tile_id, ros_v.tile_id, type);
  convert(struct_v.count, ros_v.count, type);
  convert(struct_v.floor_id, ros_v.floor_id, type);
}

template <typename T2>
void convert(iflyauto::RoadId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.tile_id, ros_v.tile_id, type);
  convert(struct_v.count, ros_v.count, type);
  convert(struct_v.ur_id, ros_v.ur_id, type);
  convert(struct_v.floor_id, ros_v.floor_id, type);
}

template <typename T2>
void convert(iflyauto::RoadPortId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.tile_id, ros_v.tile_id, type);
  convert(struct_v.count, ros_v.count, type);
}

template <typename T2>
void convert(iflyauto::LaneGroupId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.road_id, ros_v.road_id, type);
  convert(struct_v.dir_reversed, ros_v.dir_reversed, type);
  convert(struct_v.index, ros_v.index, type);
}

template <typename T2>
void convert(iflyauto::LaneBoundaryGroupId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_group_id, ros_v.lane_group_id, type);
  convert(struct_v.index, ros_v.index, type);
}

template <typename T2>
void convert(iflyauto::LaneId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_group_id, ros_v.lane_group_id, type);
  convert(struct_v.index, ros_v.index, type);
}

template <typename T2>
void convert(iflyauto::LanePortId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.tile_id, ros_v.tile_id, type);
  convert(struct_v.count, ros_v.count, type);
}

template <typename T2>
void convert(iflyauto::ParallelLaneBoundaryId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_boundary_group_id, ros_v.lane_boundary_group_id, type);
  convert(struct_v.index, ros_v.index, type);
}

template <typename T2>
void convert(iflyauto::LaneBoundaryId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.parallel_lane_boundary_id, ros_v.parallel_lane_boundary_id, type);
  convert(struct_v.index, ros_v.index, type);
}

template <typename T2>
void convert(iflyauto::IntersectionId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.tile_id, ros_v.tile_id, type);
  convert(struct_v.count, ros_v.count, type);
  convert(struct_v.ur_id, ros_v.ur_id, type);
}

template <typename T2>
void convert(iflyauto::Polyline &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.points_size, ros_v.points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.points_size >= 0 && struct_v.points_size <= PARKING_MAP_POLYLINE_POINT_MAX_NUN) {
      ros_v.points.resize(struct_v.points_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] points_size=" << struct_v.points_size 
                << " not in range PARKING_MAP_POLYLINE_POINT_MAX_NUN=" << PARKING_MAP_POLYLINE_POINT_MAX_NUN 
                << std::endl;
      ros_v.points_size = PARKING_MAP_POLYLINE_POINT_MAX_NUN;
      ros_v.points.resize(PARKING_MAP_POLYLINE_POINT_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.points.size(); i0++) {
      convert(struct_v.points[i0], ros_v.points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.points_size > PARKING_MAP_POLYLINE_POINT_MAX_NUN || ros_v.points_size < 0 || ros_v.points.size() > PARKING_MAP_POLYLINE_POINT_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] points_size=" << ros_v.points_size 
                << " ros_v.points.size()=" << ros_v.points.size()
                << " not in range PARKING_MAP_POLYLINE_POINT_MAX_NUN=" << PARKING_MAP_POLYLINE_POINT_MAX_NUN 
                << std::endl;
    }
    if (ros_v.points.size() > PARKING_MAP_POLYLINE_POINT_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_POLYLINE_POINT_MAX_NUN; i0++) {
        convert(struct_v.points[i0], ros_v.points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.points.size(); i0++) {
        convert(struct_v.points[i0], ros_v.points[i0], type);
      }
    }
  }
  //
  convert(struct_v.length, ros_v.length, type);
  for (size_t i1 = 0; i1 < ros_v.buffer.size(); i1++) {
	  convert(struct_v.buffer[i1], ros_v.buffer[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::ParkingLaneBoundary &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.line_type, ros_v.line_type, type);
  convert(struct_v.lane_boundary_type, ros_v.lane_boundary_type, type);
  convert(struct_v.geometry, ros_v.geometry, type);
  convert(struct_v.color, ros_v.color, type);
  convert(struct_v.source, ros_v.source, type);
}

template <typename T2>
void convert(iflyauto::ParallelLaneBoundary &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.sequential_lane_boundaries_size, ros_v.sequential_lane_boundaries_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.sequential_lane_boundaries_size >= 0 && struct_v.sequential_lane_boundaries_size <= PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN) {
      ros_v.sequential_lane_boundaries.resize(struct_v.sequential_lane_boundaries_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] sequential_lane_boundaries_size=" << struct_v.sequential_lane_boundaries_size 
                << " not in range PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN=" << PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN 
                << std::endl;
      ros_v.sequential_lane_boundaries_size = PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN;
      ros_v.sequential_lane_boundaries.resize(PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.sequential_lane_boundaries.size(); i0++) {
      convert(struct_v.sequential_lane_boundaries[i0], ros_v.sequential_lane_boundaries[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.sequential_lane_boundaries_size > PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN || ros_v.sequential_lane_boundaries_size < 0 || ros_v.sequential_lane_boundaries.size() > PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] sequential_lane_boundaries_size=" << ros_v.sequential_lane_boundaries_size 
                << " ros_v.sequential_lane_boundaries.size()=" << ros_v.sequential_lane_boundaries.size()
                << " not in range PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN=" << PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN 
                << std::endl;
    }
    if (ros_v.sequential_lane_boundaries.size() > PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN; i0++) {
        convert(struct_v.sequential_lane_boundaries[i0], ros_v.sequential_lane_boundaries[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.sequential_lane_boundaries.size(); i0++) {
        convert(struct_v.sequential_lane_boundaries[i0], ros_v.sequential_lane_boundaries[i0], type);
      }
    }
  }
  //
  convert(struct_v.id, ros_v.id, type);
}

template <typename T2>
void convert(iflyauto::LaneBoundaryGroup &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.parallel_lane_boundaries_size, ros_v.parallel_lane_boundaries_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.parallel_lane_boundaries_size >= 0 && struct_v.parallel_lane_boundaries_size <= PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN) {
      ros_v.parallel_lane_boundaries.resize(struct_v.parallel_lane_boundaries_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] parallel_lane_boundaries_size=" << struct_v.parallel_lane_boundaries_size 
                << " not in range PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN=" << PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN 
                << std::endl;
      ros_v.parallel_lane_boundaries_size = PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN;
      ros_v.parallel_lane_boundaries.resize(PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.parallel_lane_boundaries.size(); i0++) {
      convert(struct_v.parallel_lane_boundaries[i0], ros_v.parallel_lane_boundaries[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.parallel_lane_boundaries_size > PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN || ros_v.parallel_lane_boundaries_size < 0 || ros_v.parallel_lane_boundaries.size() > PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] parallel_lane_boundaries_size=" << ros_v.parallel_lane_boundaries_size 
                << " ros_v.parallel_lane_boundaries.size()=" << ros_v.parallel_lane_boundaries.size()
                << " not in range PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN=" << PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN 
                << std::endl;
    }
    if (ros_v.parallel_lane_boundaries.size() > PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_PARALLEL_LANEBOUNDARY_MAX_NUN; i0++) {
        convert(struct_v.parallel_lane_boundaries[i0], ros_v.parallel_lane_boundaries[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.parallel_lane_boundaries.size(); i0++) {
        convert(struct_v.parallel_lane_boundaries[i0], ros_v.parallel_lane_boundaries[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::Segment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.start_index, ros_v.start_index, type);
  convert(struct_v.end_index, ros_v.end_index, type);
  convert(struct_v.length, ros_v.length, type);
}

template <typename T2>
void convert(iflyauto::LaneSegment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_id, ros_v.lane_id, type);
  convert(struct_v.segs_size, ros_v.segs_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.segs_size >= 0 && struct_v.segs_size <= PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN) {
      ros_v.segs.resize(struct_v.segs_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] segs_size=" << struct_v.segs_size 
                << " not in range PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN=" << PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN 
                << std::endl;
      ros_v.segs_size = PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN;
      ros_v.segs.resize(PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.segs.size(); i0++) {
      convert(struct_v.segs[i0], ros_v.segs[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.segs_size > PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN || ros_v.segs_size < 0 || ros_v.segs.size() > PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] segs_size=" << ros_v.segs_size 
                << " ros_v.segs.size()=" << ros_v.segs.size()
                << " not in range PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN=" << PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN 
                << std::endl;
    }
    if (ros_v.segs.size() > PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_LANE_SEGMENT_SEGS_MAX_NUN; i0++) {
        convert(struct_v.segs[i0], ros_v.segs[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.segs.size(); i0++) {
        convert(struct_v.segs[i0], ros_v.segs[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::Lane &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.lane_type, ros_v.lane_type, type);
  convert(struct_v.center_line, ros_v.center_line, type);
  convert(struct_v.left_boundary_group, ros_v.left_boundary_group, type);
  convert(struct_v.right_boundary_group, ros_v.right_boundary_group, type);
  convert(struct_v.head_id, ros_v.head_id, type);
  convert(struct_v.tail_id, ros_v.tail_id, type);
  convert(struct_v.direction, ros_v.direction, type);
}

template <typename T2>
void convert(iflyauto::LaneGroup &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.lanes_size, ros_v.lanes_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lanes_size >= 0 && struct_v.lanes_size <= PARKING_MAP_LANE_MAX_NUN) {
      ros_v.lanes.resize(struct_v.lanes_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lanes_size=" << struct_v.lanes_size 
                << " not in range PARKING_MAP_LANE_MAX_NUN=" << PARKING_MAP_LANE_MAX_NUN 
                << std::endl;
      ros_v.lanes_size = PARKING_MAP_LANE_MAX_NUN;
      ros_v.lanes.resize(PARKING_MAP_LANE_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.lanes.size(); i0++) {
      convert(struct_v.lanes[i0], ros_v.lanes[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lanes_size > PARKING_MAP_LANE_MAX_NUN || ros_v.lanes_size < 0 || ros_v.lanes.size() > PARKING_MAP_LANE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lanes_size=" << ros_v.lanes_size 
                << " ros_v.lanes.size()=" << ros_v.lanes.size()
                << " not in range PARKING_MAP_LANE_MAX_NUN=" << PARKING_MAP_LANE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.lanes.size() > PARKING_MAP_LANE_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_LANE_MAX_NUN; i0++) {
        convert(struct_v.lanes[i0], ros_v.lanes[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lanes.size(); i0++) {
        convert(struct_v.lanes[i0], ros_v.lanes[i0], type);
      }
    }
  }
  //
  convert(struct_v.lane_boundary_groups_size, ros_v.lane_boundary_groups_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_boundary_groups_size >= 0 && struct_v.lane_boundary_groups_size <= PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN) {
      ros_v.lane_boundary_groups.resize(struct_v.lane_boundary_groups_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_boundary_groups_size=" << struct_v.lane_boundary_groups_size 
                << " not in range PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN=" << PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN 
                << std::endl;
      ros_v.lane_boundary_groups_size = PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN;
      ros_v.lane_boundary_groups.resize(PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN);
    }
    for (size_t i1 = 0; i1 < ros_v.lane_boundary_groups.size(); i1++) {
      convert(struct_v.lane_boundary_groups[i1], ros_v.lane_boundary_groups[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_boundary_groups_size > PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN || ros_v.lane_boundary_groups_size < 0 || ros_v.lane_boundary_groups.size() > PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_boundary_groups_size=" << ros_v.lane_boundary_groups_size 
                << " ros_v.lane_boundary_groups.size()=" << ros_v.lane_boundary_groups.size()
                << " not in range PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN=" << PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN 
                << std::endl;
    }
    if (ros_v.lane_boundary_groups.size() > PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN) {
      for (size_t i1 = 0; i1 < PARKING_MAP_LANE_BOUNDARY_GROUP_MAX_NUN; i1++) {
        convert(struct_v.lane_boundary_groups[i1], ros_v.lane_boundary_groups[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.lane_boundary_groups.size(); i1++) {
        convert(struct_v.lane_boundary_groups[i1], ros_v.lane_boundary_groups[i1], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::Road &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.head_id, ros_v.head_id, type);
  convert(struct_v.tail_id, ros_v.tail_id, type);
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.road_type, ros_v.road_type, type);
  convert(struct_v.road_center, ros_v.road_center, type);
  convert(struct_v.travel_direction, ros_v.travel_direction, type);
  convert(struct_v.ramp, ros_v.ramp, type);
  for (size_t i0 = 0; i0 < ros_v.name.size(); i0++) {
	  convert(struct_v.name[i0], ros_v.name[i0], type);
  }
  convert(struct_v.road_class, ros_v.road_class, type);
  convert(struct_v.road_structure_size, ros_v.road_structure_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.road_structure_size >= 0 && struct_v.road_structure_size <= PARKING_MAP_ROAD_STRUCTURE_MAX_NUN) {
      ros_v.road_structure.resize(struct_v.road_structure_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] road_structure_size=" << struct_v.road_structure_size 
                << " not in range PARKING_MAP_ROAD_STRUCTURE_MAX_NUN=" << PARKING_MAP_ROAD_STRUCTURE_MAX_NUN 
                << std::endl;
      ros_v.road_structure_size = PARKING_MAP_ROAD_STRUCTURE_MAX_NUN;
      ros_v.road_structure.resize(PARKING_MAP_ROAD_STRUCTURE_MAX_NUN);
    }
    for (size_t i1 = 0; i1 < ros_v.road_structure.size(); i1++) {
      convert(struct_v.road_structure[i1], ros_v.road_structure[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.road_structure_size > PARKING_MAP_ROAD_STRUCTURE_MAX_NUN || ros_v.road_structure_size < 0 || ros_v.road_structure.size() > PARKING_MAP_ROAD_STRUCTURE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] road_structure_size=" << ros_v.road_structure_size 
                << " ros_v.road_structure.size()=" << ros_v.road_structure.size()
                << " not in range PARKING_MAP_ROAD_STRUCTURE_MAX_NUN=" << PARKING_MAP_ROAD_STRUCTURE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.road_structure.size() > PARKING_MAP_ROAD_STRUCTURE_MAX_NUN) {
      for (size_t i1 = 0; i1 < PARKING_MAP_ROAD_STRUCTURE_MAX_NUN; i1++) {
        convert(struct_v.road_structure[i1], ros_v.road_structure[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.road_structure.size(); i1++) {
        convert(struct_v.road_structure[i1], ros_v.road_structure[i1], type);
      }
    }
  }
  //
  convert(struct_v.intersection_internal, ros_v.intersection_internal, type);
  convert(struct_v.intersection_id, ros_v.intersection_id, type);
}

template <typename T2>
void convert(iflyauto::Reference &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.road_id, ros_v.road_id, type);
  convert(struct_v.lane_id, ros_v.lane_id, type);
  convert(struct_v.attribute_id, ros_v.attribute_id, type);
  convert(struct_v.road_port, ros_v.road_port, type);
  convert(struct_v.lane_port, ros_v.lane_port, type);
  convert(struct_v.offset, ros_v.offset, type);
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.orientation, ros_v.orientation, type);
  convert(struct_v.id_type, ros_v.id_type, type);
  convert(struct_v.fully_covered, ros_v.fully_covered, type);
}

template <typename T2>
void convert(iflyauto::RoadMark &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.ref_size, ros_v.ref_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ref_size >= 0 && struct_v.ref_size <= PARKING_MAP_REFERENCE_MAX_NUN) {
      ros_v.ref.resize(struct_v.ref_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ref_size=" << struct_v.ref_size 
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
      ros_v.ref_size = PARKING_MAP_REFERENCE_MAX_NUN;
      ros_v.ref.resize(PARKING_MAP_REFERENCE_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
      convert(struct_v.ref[i0], ros_v.ref[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ref_size > PARKING_MAP_REFERENCE_MAX_NUN || ros_v.ref_size < 0 || ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ref_size=" << ros_v.ref_size 
                << " ros_v.ref.size()=" << ros_v.ref.size()
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_REFERENCE_MAX_NUN; i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    }
  }
  //
  convert(struct_v.shape_size, ros_v.shape_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.shape_size >= 0 && struct_v.shape_size <= PARKING_MAP_ROADMARK_POINT_MAX_NUN) {
      ros_v.shape.resize(struct_v.shape_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] shape_size=" << struct_v.shape_size 
                << " not in range PARKING_MAP_ROADMARK_POINT_MAX_NUN=" << PARKING_MAP_ROADMARK_POINT_MAX_NUN 
                << std::endl;
      ros_v.shape_size = PARKING_MAP_ROADMARK_POINT_MAX_NUN;
      ros_v.shape.resize(PARKING_MAP_ROADMARK_POINT_MAX_NUN);
    }
    for (size_t i1 = 0; i1 < ros_v.shape.size(); i1++) {
      convert(struct_v.shape[i1], ros_v.shape[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.shape_size > PARKING_MAP_ROADMARK_POINT_MAX_NUN || ros_v.shape_size < 0 || ros_v.shape.size() > PARKING_MAP_ROADMARK_POINT_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] shape_size=" << ros_v.shape_size 
                << " ros_v.shape.size()=" << ros_v.shape.size()
                << " not in range PARKING_MAP_ROADMARK_POINT_MAX_NUN=" << PARKING_MAP_ROADMARK_POINT_MAX_NUN 
                << std::endl;
    }
    if (ros_v.shape.size() > PARKING_MAP_ROADMARK_POINT_MAX_NUN) {
      for (size_t i1 = 0; i1 < PARKING_MAP_ROADMARK_POINT_MAX_NUN; i1++) {
        convert(struct_v.shape[i1], ros_v.shape[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.shape.size(); i1++) {
        convert(struct_v.shape[i1], ros_v.shape[i1], type);
      }
    }
  }
  //
  convert(struct_v.angle, ros_v.angle, type);
  convert(struct_v.direction, ros_v.direction, type);
  convert(struct_v.painting_type, ros_v.painting_type, type);
}

template <typename T2>
void convert(iflyauto::RoadObstacle &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.ref_size, ros_v.ref_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ref_size >= 0 && struct_v.ref_size <= PARKING_MAP_REFERENCE_MAX_NUN) {
      ros_v.ref.resize(struct_v.ref_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ref_size=" << struct_v.ref_size 
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
      ros_v.ref_size = PARKING_MAP_REFERENCE_MAX_NUN;
      ros_v.ref.resize(PARKING_MAP_REFERENCE_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
      convert(struct_v.ref[i0], ros_v.ref[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ref_size > PARKING_MAP_REFERENCE_MAX_NUN || ros_v.ref_size < 0 || ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ref_size=" << ros_v.ref_size 
                << " ros_v.ref.size()=" << ros_v.ref.size()
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_REFERENCE_MAX_NUN; i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    }
  }
  //
  convert(struct_v.shape_size, ros_v.shape_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.shape_size >= 0 && struct_v.shape_size <= PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN) {
      ros_v.shape.resize(struct_v.shape_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] shape_size=" << struct_v.shape_size 
                << " not in range PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN=" << PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN 
                << std::endl;
      ros_v.shape_size = PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN;
      ros_v.shape.resize(PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN);
    }
    for (size_t i1 = 0; i1 < ros_v.shape.size(); i1++) {
      convert(struct_v.shape[i1], ros_v.shape[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.shape_size > PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN || ros_v.shape_size < 0 || ros_v.shape.size() > PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] shape_size=" << ros_v.shape_size 
                << " ros_v.shape.size()=" << ros_v.shape.size()
                << " not in range PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN=" << PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN 
                << std::endl;
    }
    if (ros_v.shape.size() > PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN) {
      for (size_t i1 = 0; i1 < PARKING_MAP_ROAD_OBSTACLE_POINT_MAX_NUN; i1++) {
        convert(struct_v.shape[i1], ros_v.shape[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.shape.size(); i1++) {
        convert(struct_v.shape[i1], ros_v.shape[i1], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::ParkingMapLimiter &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  for (size_t i0 = 0; i0 < ros_v.end_points.size(); i0++) {
	  convert(struct_v.end_points[i0], ros_v.end_points[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::ParkingSpace &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.parking_space_type, ros_v.parking_space_type, type);
  convert(struct_v.ref_size, ros_v.ref_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ref_size >= 0 && struct_v.ref_size <= PARKING_MAP_REFERENCE_MAX_NUN) {
      ros_v.ref.resize(struct_v.ref_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ref_size=" << struct_v.ref_size 
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
      ros_v.ref_size = PARKING_MAP_REFERENCE_MAX_NUN;
      ros_v.ref.resize(PARKING_MAP_REFERENCE_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
      convert(struct_v.ref[i0], ros_v.ref[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ref_size > PARKING_MAP_REFERENCE_MAX_NUN || ros_v.ref_size < 0 || ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ref_size=" << ros_v.ref_size 
                << " ros_v.ref.size()=" << ros_v.ref.size()
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_REFERENCE_MAX_NUN; i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    }
  }
  //
  for (size_t i1 = 0; i1 < ros_v.shape.size(); i1++) {
	  convert(struct_v.shape[i1], ros_v.shape[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.floor_name.size(); i2++) {
	  convert(struct_v.floor_name[i2], ros_v.floor_name[i2], type);
  }
  convert(struct_v.allow_parking, ros_v.allow_parking, type);
  convert(struct_v.is_turn_corner, ros_v.is_turn_corner, type);
  convert(struct_v.slot_source, ros_v.slot_source, type);
  convert(struct_v.limiters_size, ros_v.limiters_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.limiters_size >= 0 && struct_v.limiters_size <= PARKING_MAP_LIMITER_MAX_NUN) {
      ros_v.limiters.resize(struct_v.limiters_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] limiters_size=" << struct_v.limiters_size 
                << " not in range PARKING_MAP_LIMITER_MAX_NUN=" << PARKING_MAP_LIMITER_MAX_NUN 
                << std::endl;
      ros_v.limiters_size = PARKING_MAP_LIMITER_MAX_NUN;
      ros_v.limiters.resize(PARKING_MAP_LIMITER_MAX_NUN);
    }
    for (size_t i3 = 0; i3 < ros_v.limiters.size(); i3++) {
      convert(struct_v.limiters[i3], ros_v.limiters[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.limiters_size > PARKING_MAP_LIMITER_MAX_NUN || ros_v.limiters_size < 0 || ros_v.limiters.size() > PARKING_MAP_LIMITER_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] limiters_size=" << ros_v.limiters_size 
                << " ros_v.limiters.size()=" << ros_v.limiters.size()
                << " not in range PARKING_MAP_LIMITER_MAX_NUN=" << PARKING_MAP_LIMITER_MAX_NUN 
                << std::endl;
    }
    if (ros_v.limiters.size() > PARKING_MAP_LIMITER_MAX_NUN) {
      for (size_t i3 = 0; i3 < PARKING_MAP_LIMITER_MAX_NUN; i3++) {
        convert(struct_v.limiters[i3], ros_v.limiters[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.limiters.size(); i3++) {
        convert(struct_v.limiters[i3], ros_v.limiters[i3], type);
      }
    }
  }
  //
  convert(struct_v.empty_votes, ros_v.empty_votes, type);
}

template <typename T2>
void convert(iflyauto::TargetParkingId &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.attribute_id, ros_v.attribute_id, type);
  convert(struct_v.ref, ros_v.ref, type);
}

template <typename T2>
void convert(iflyauto::PolygonObject &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.ref_size, ros_v.ref_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ref_size >= 0 && struct_v.ref_size <= PARKING_MAP_REFERENCE_MAX_NUN) {
      ros_v.ref.resize(struct_v.ref_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ref_size=" << struct_v.ref_size 
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
      ros_v.ref_size = PARKING_MAP_REFERENCE_MAX_NUN;
      ros_v.ref.resize(PARKING_MAP_REFERENCE_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
      convert(struct_v.ref[i0], ros_v.ref[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ref_size > PARKING_MAP_REFERENCE_MAX_NUN || ros_v.ref_size < 0 || ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ref_size=" << ros_v.ref_size 
                << " ros_v.ref.size()=" << ros_v.ref.size()
                << " not in range PARKING_MAP_REFERENCE_MAX_NUN=" << PARKING_MAP_REFERENCE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.ref.size() > PARKING_MAP_REFERENCE_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_REFERENCE_MAX_NUN; i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ref.size(); i0++) {
        convert(struct_v.ref[i0], ros_v.ref[i0], type);
      }
    }
  }
  //
  convert(struct_v.corner_size, ros_v.corner_size, type);
  for (size_t i1 = 0; i1 < ros_v.corners.size(); i1++) {
	  convert(struct_v.corners[i1], ros_v.corners[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::RoadTile &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.road_size, ros_v.road_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.road_size >= 0 && struct_v.road_size <= PARKING_MAP_ROAD_MAX_NUN) {
      ros_v.road.resize(struct_v.road_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] road_size=" << struct_v.road_size 
                << " not in range PARKING_MAP_ROAD_MAX_NUN=" << PARKING_MAP_ROAD_MAX_NUN 
                << std::endl;
      ros_v.road_size = PARKING_MAP_ROAD_MAX_NUN;
      ros_v.road.resize(PARKING_MAP_ROAD_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.road.size(); i0++) {
      convert(struct_v.road[i0], ros_v.road[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.road_size > PARKING_MAP_ROAD_MAX_NUN || ros_v.road_size < 0 || ros_v.road.size() > PARKING_MAP_ROAD_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] road_size=" << ros_v.road_size 
                << " ros_v.road.size()=" << ros_v.road.size()
                << " not in range PARKING_MAP_ROAD_MAX_NUN=" << PARKING_MAP_ROAD_MAX_NUN 
                << std::endl;
    }
    if (ros_v.road.size() > PARKING_MAP_ROAD_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_ROAD_MAX_NUN; i0++) {
        convert(struct_v.road[i0], ros_v.road[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.road.size(); i0++) {
        convert(struct_v.road[i0], ros_v.road[i0], type);
      }
    }
  }
  //
  convert(struct_v.lane_segment_size, ros_v.lane_segment_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_segment_size >= 0 && struct_v.lane_segment_size <= PARKING_MAP_LANE_SEGMENT_MAX_NUN) {
      ros_v.lane_segment.resize(struct_v.lane_segment_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_segment_size=" << struct_v.lane_segment_size 
                << " not in range PARKING_MAP_LANE_SEGMENT_MAX_NUN=" << PARKING_MAP_LANE_SEGMENT_MAX_NUN 
                << std::endl;
      ros_v.lane_segment_size = PARKING_MAP_LANE_SEGMENT_MAX_NUN;
      ros_v.lane_segment.resize(PARKING_MAP_LANE_SEGMENT_MAX_NUN);
    }
    for (size_t i1 = 0; i1 < ros_v.lane_segment.size(); i1++) {
      convert(struct_v.lane_segment[i1], ros_v.lane_segment[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_segment_size > PARKING_MAP_LANE_SEGMENT_MAX_NUN || ros_v.lane_segment_size < 0 || ros_v.lane_segment.size() > PARKING_MAP_LANE_SEGMENT_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_segment_size=" << ros_v.lane_segment_size 
                << " ros_v.lane_segment.size()=" << ros_v.lane_segment.size()
                << " not in range PARKING_MAP_LANE_SEGMENT_MAX_NUN=" << PARKING_MAP_LANE_SEGMENT_MAX_NUN 
                << std::endl;
    }
    if (ros_v.lane_segment.size() > PARKING_MAP_LANE_SEGMENT_MAX_NUN) {
      for (size_t i1 = 0; i1 < PARKING_MAP_LANE_SEGMENT_MAX_NUN; i1++) {
        convert(struct_v.lane_segment[i1], ros_v.lane_segment[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.lane_segment.size(); i1++) {
        convert(struct_v.lane_segment[i1], ros_v.lane_segment[i1], type);
      }
    }
  }
  //
  convert(struct_v.road_mark_size, ros_v.road_mark_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.road_mark_size >= 0 && struct_v.road_mark_size <= PARKING_MAP_ROAD_MARK_MAX_NUN) {
      ros_v.road_mark.resize(struct_v.road_mark_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] road_mark_size=" << struct_v.road_mark_size 
                << " not in range PARKING_MAP_ROAD_MARK_MAX_NUN=" << PARKING_MAP_ROAD_MARK_MAX_NUN 
                << std::endl;
      ros_v.road_mark_size = PARKING_MAP_ROAD_MARK_MAX_NUN;
      ros_v.road_mark.resize(PARKING_MAP_ROAD_MARK_MAX_NUN);
    }
    for (size_t i2 = 0; i2 < ros_v.road_mark.size(); i2++) {
      convert(struct_v.road_mark[i2], ros_v.road_mark[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.road_mark_size > PARKING_MAP_ROAD_MARK_MAX_NUN || ros_v.road_mark_size < 0 || ros_v.road_mark.size() > PARKING_MAP_ROAD_MARK_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] road_mark_size=" << ros_v.road_mark_size 
                << " ros_v.road_mark.size()=" << ros_v.road_mark.size()
                << " not in range PARKING_MAP_ROAD_MARK_MAX_NUN=" << PARKING_MAP_ROAD_MARK_MAX_NUN 
                << std::endl;
    }
    if (ros_v.road_mark.size() > PARKING_MAP_ROAD_MARK_MAX_NUN) {
      for (size_t i2 = 0; i2 < PARKING_MAP_ROAD_MARK_MAX_NUN; i2++) {
        convert(struct_v.road_mark[i2], ros_v.road_mark[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.road_mark.size(); i2++) {
        convert(struct_v.road_mark[i2], ros_v.road_mark[i2], type);
      }
    }
  }
  //
  convert(struct_v.road_obstacle_size, ros_v.road_obstacle_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.road_obstacle_size >= 0 && struct_v.road_obstacle_size <= PARKING_MAP_ROAD_OBSTACLE_MAX_NUN) {
      ros_v.road_obstacle.resize(struct_v.road_obstacle_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] road_obstacle_size=" << struct_v.road_obstacle_size 
                << " not in range PARKING_MAP_ROAD_OBSTACLE_MAX_NUN=" << PARKING_MAP_ROAD_OBSTACLE_MAX_NUN 
                << std::endl;
      ros_v.road_obstacle_size = PARKING_MAP_ROAD_OBSTACLE_MAX_NUN;
      ros_v.road_obstacle.resize(PARKING_MAP_ROAD_OBSTACLE_MAX_NUN);
    }
    for (size_t i3 = 0; i3 < ros_v.road_obstacle.size(); i3++) {
      convert(struct_v.road_obstacle[i3], ros_v.road_obstacle[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.road_obstacle_size > PARKING_MAP_ROAD_OBSTACLE_MAX_NUN || ros_v.road_obstacle_size < 0 || ros_v.road_obstacle.size() > PARKING_MAP_ROAD_OBSTACLE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] road_obstacle_size=" << ros_v.road_obstacle_size 
                << " ros_v.road_obstacle.size()=" << ros_v.road_obstacle.size()
                << " not in range PARKING_MAP_ROAD_OBSTACLE_MAX_NUN=" << PARKING_MAP_ROAD_OBSTACLE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.road_obstacle.size() > PARKING_MAP_ROAD_OBSTACLE_MAX_NUN) {
      for (size_t i3 = 0; i3 < PARKING_MAP_ROAD_OBSTACLE_MAX_NUN; i3++) {
        convert(struct_v.road_obstacle[i3], ros_v.road_obstacle[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.road_obstacle.size(); i3++) {
        convert(struct_v.road_obstacle[i3], ros_v.road_obstacle[i3], type);
      }
    }
  }
  //
  convert(struct_v.parking_space_size, ros_v.parking_space_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.parking_space_size >= 0 && struct_v.parking_space_size <= PARKING_MAP_PARKING_SPACE_MAX_NUN) {
      ros_v.parking_space.resize(struct_v.parking_space_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] parking_space_size=" << struct_v.parking_space_size 
                << " not in range PARKING_MAP_PARKING_SPACE_MAX_NUN=" << PARKING_MAP_PARKING_SPACE_MAX_NUN 
                << std::endl;
      ros_v.parking_space_size = PARKING_MAP_PARKING_SPACE_MAX_NUN;
      ros_v.parking_space.resize(PARKING_MAP_PARKING_SPACE_MAX_NUN);
    }
    for (size_t i4 = 0; i4 < ros_v.parking_space.size(); i4++) {
      convert(struct_v.parking_space[i4], ros_v.parking_space[i4], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.parking_space_size > PARKING_MAP_PARKING_SPACE_MAX_NUN || ros_v.parking_space_size < 0 || ros_v.parking_space.size() > PARKING_MAP_PARKING_SPACE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] parking_space_size=" << ros_v.parking_space_size 
                << " ros_v.parking_space.size()=" << ros_v.parking_space.size()
                << " not in range PARKING_MAP_PARKING_SPACE_MAX_NUN=" << PARKING_MAP_PARKING_SPACE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.parking_space.size() > PARKING_MAP_PARKING_SPACE_MAX_NUN) {
      for (size_t i4 = 0; i4 < PARKING_MAP_PARKING_SPACE_MAX_NUN; i4++) {
        convert(struct_v.parking_space[i4], ros_v.parking_space[i4], type);
      }
    } else {
      for (size_t i4 = 0; i4 < ros_v.parking_space.size(); i4++) {
        convert(struct_v.parking_space[i4], ros_v.parking_space[i4], type);
      }
    }
  }
  //
  convert(struct_v.polygon_obstacle_size, ros_v.polygon_obstacle_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.polygon_obstacle_size >= 0 && struct_v.polygon_obstacle_size <= PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN) {
      ros_v.polygon_obstacle.resize(struct_v.polygon_obstacle_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] polygon_obstacle_size=" << struct_v.polygon_obstacle_size 
                << " not in range PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN=" << PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN 
                << std::endl;
      ros_v.polygon_obstacle_size = PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN;
      ros_v.polygon_obstacle.resize(PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN);
    }
    for (size_t i5 = 0; i5 < ros_v.polygon_obstacle.size(); i5++) {
      convert(struct_v.polygon_obstacle[i5], ros_v.polygon_obstacle[i5], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.polygon_obstacle_size > PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN || ros_v.polygon_obstacle_size < 0 || ros_v.polygon_obstacle.size() > PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] polygon_obstacle_size=" << ros_v.polygon_obstacle_size 
                << " ros_v.polygon_obstacle.size()=" << ros_v.polygon_obstacle.size()
                << " not in range PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN=" << PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN 
                << std::endl;
    }
    if (ros_v.polygon_obstacle.size() > PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN) {
      for (size_t i5 = 0; i5 < PARKING_MAP_POLYGON_OBSTACLE_MAX_NUN; i5++) {
        convert(struct_v.polygon_obstacle[i5], ros_v.polygon_obstacle[i5], type);
      }
    } else {
      for (size_t i5 = 0; i5 < ros_v.polygon_obstacle.size(); i5++) {
        convert(struct_v.polygon_obstacle[i5], ros_v.polygon_obstacle[i5], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::FeaturePoint &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.available, ros_v.available, type);
  convert(struct_v.detection_time, ros_v.detection_time, type);
  convert(struct_v.global_pos, ros_v.global_pos, type);
  convert(struct_v.local_pos, ros_v.local_pos, type);
  convert(struct_v.global_quat, ros_v.global_quat, type);
  convert(struct_v.local_quat, ros_v.local_quat, type);
}

template <typename T2>
void convert(iflyauto::RampInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.ramp_type, ros_v.ramp_type, type);
  convert(struct_v.entrance_ramp_point, ros_v.entrance_ramp_point, type);
  convert(struct_v.exit_ramp_point, ros_v.exit_ramp_point, type);
}

template <typename T2>
void convert(iflyauto::SemanticInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.gps_signal_loss_point, ros_v.gps_signal_loss_point, type);
  convert(struct_v.gps_snr, ros_v.gps_snr, type);
  convert(struct_v.gps_signal_available, ros_v.gps_signal_available, type);
  convert(struct_v.ramp_info_size, ros_v.ramp_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ramp_info_size >= 0 && struct_v.ramp_info_size <= PARKING_MAP_RAMP_MAX_NUN) {
      ros_v.ramp_info.resize(struct_v.ramp_info_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ramp_info_size=" << struct_v.ramp_info_size 
                << " not in range PARKING_MAP_RAMP_MAX_NUN=" << PARKING_MAP_RAMP_MAX_NUN 
                << std::endl;
      ros_v.ramp_info_size = PARKING_MAP_RAMP_MAX_NUN;
      ros_v.ramp_info.resize(PARKING_MAP_RAMP_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.ramp_info.size(); i0++) {
      convert(struct_v.ramp_info[i0], ros_v.ramp_info[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ramp_info_size > PARKING_MAP_RAMP_MAX_NUN || ros_v.ramp_info_size < 0 || ros_v.ramp_info.size() > PARKING_MAP_RAMP_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ramp_info_size=" << ros_v.ramp_info_size 
                << " ros_v.ramp_info.size()=" << ros_v.ramp_info.size()
                << " not in range PARKING_MAP_RAMP_MAX_NUN=" << PARKING_MAP_RAMP_MAX_NUN 
                << std::endl;
    }
    if (ros_v.ramp_info.size() > PARKING_MAP_RAMP_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_RAMP_MAX_NUN; i0++) {
        convert(struct_v.ramp_info[i0], ros_v.ramp_info[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ramp_info.size(); i0++) {
        convert(struct_v.ramp_info[i0], ros_v.ramp_info[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::ParkingInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.timestamp, ros_v.timestamp, type);
  convert(struct_v.perception_isp_timestamp, ros_v.perception_isp_timestamp, type);
  convert(struct_v.trajectory_ref_point, ros_v.trajectory_ref_point, type);
  convert(struct_v.trace_start, ros_v.trace_start, type);
  convert(struct_v.trace_dest, ros_v.trace_dest, type);
  for (size_t i0 = 0; i0 < ros_v.target_prk_pos.size(); i0++) {
	  convert(struct_v.target_prk_pos[i0], ros_v.target_prk_pos[i0], type);
  }
  convert(struct_v.target_prk_id, ros_v.target_prk_id, type);
  convert(struct_v.road_tile_info, ros_v.road_tile_info, type);
  convert(struct_v.semantic_info, ros_v.semantic_info, type);
  convert(struct_v.map_id, ros_v.map_id, type);
}

template <typename T2>
void convert(iflyauto::ParkingMapFileInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  for (size_t i0 = 0; i0 < ros_v.file_name.size(); i0++) {
	  convert(struct_v.file_name[i0], ros_v.file_name[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.file_path.size(); i1++) {
	  convert(struct_v.file_path[i1], ros_v.file_path[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.vehicle_vin.size(); i2++) {
	  convert(struct_v.vehicle_vin[i2], ros_v.vehicle_vin[i2], type);
  }
}

template <typename T2>
void convert(iflyauto::EhpfileInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.map_file_id, ros_v.map_file_id, type);
  for (size_t i0 = 0; i0 < ros_v.map_file_name.size(); i0++) {
	  convert(struct_v.map_file_name[i0], ros_v.map_file_name[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::EhpOutput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.map_file_size, ros_v.map_file_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.map_file_size >= 0 && struct_v.map_file_size <= PARKING_MAP_EHP_MAP_MAX_NUN) {
      ros_v.map_file.resize(struct_v.map_file_size);
    } else {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << " [convert][TO_ROS] map_file_size=" << struct_v.map_file_size 
                << " not in range PARKING_MAP_EHP_MAP_MAX_NUN=" << PARKING_MAP_EHP_MAP_MAX_NUN 
                << std::endl;
      ros_v.map_file_size = PARKING_MAP_EHP_MAP_MAX_NUN;
      ros_v.map_file.resize(PARKING_MAP_EHP_MAP_MAX_NUN);
    }
    for (size_t i0 = 0; i0 < ros_v.map_file.size(); i0++) {
      convert(struct_v.map_file[i0], ros_v.map_file[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.map_file_size > PARKING_MAP_EHP_MAP_MAX_NUN || ros_v.map_file_size < 0 || ros_v.map_file.size() > PARKING_MAP_EHP_MAP_MAX_NUN) {
      std::cout << "convert/ifly_parking_map_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] map_file_size=" << ros_v.map_file_size 
                << " ros_v.map_file.size()=" << ros_v.map_file.size()
                << " not in range PARKING_MAP_EHP_MAP_MAX_NUN=" << PARKING_MAP_EHP_MAP_MAX_NUN 
                << std::endl;
    }
    if (ros_v.map_file.size() > PARKING_MAP_EHP_MAP_MAX_NUN) {
      for (size_t i0 = 0; i0 < PARKING_MAP_EHP_MAP_MAX_NUN; i0++) {
        convert(struct_v.map_file[i0], ros_v.map_file[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.map_file.size(); i0++) {
        convert(struct_v.map_file[i0], ros_v.map_file[i0], type);
      }
    }
  }
  //
  convert(struct_v.ehp_status, ros_v.ehp_status, type);
  convert(struct_v.ehp_matching_status, ros_v.ehp_matching_status, type);
  convert(struct_v.ehp_map_status, ros_v.ehp_map_status, type);
  for (size_t i1 = 0; i1 < ros_v.failed_reason.size(); i1++) {
	  convert(struct_v.failed_reason[i1], ros_v.failed_reason[i1], type);
  }
}

