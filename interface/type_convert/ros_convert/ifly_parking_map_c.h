#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/AttributeId.h"
#include "struct_msgs_v2_10/AttributeId.h"
#include "struct_msgs/Coordinate.h"
#include "struct_msgs_v2_10/Coordinate.h"
#include "struct_msgs/EhpOutput.h"
#include "struct_msgs_v2_10/EhpOutput.h"
#include "struct_msgs/EhpfileInfo.h"
#include "struct_msgs_v2_10/EhpfileInfo.h"
#include "struct_msgs/FeaturePoint.h"
#include "struct_msgs_v2_10/FeaturePoint.h"
#include "struct_msgs/IntersectionId.h"
#include "struct_msgs_v2_10/IntersectionId.h"
#include "struct_msgs/Lane.h"
#include "struct_msgs_v2_10/Lane.h"
#include "struct_msgs/LaneBoundaryGroup.h"
#include "struct_msgs_v2_10/LaneBoundaryGroup.h"
#include "struct_msgs/LaneBoundaryGroupId.h"
#include "struct_msgs_v2_10/LaneBoundaryGroupId.h"
#include "struct_msgs/LaneBoundaryId.h"
#include "struct_msgs_v2_10/LaneBoundaryId.h"
#include "struct_msgs/LaneGroup.h"
#include "struct_msgs_v2_10/LaneGroup.h"
#include "struct_msgs/LaneGroupId.h"
#include "struct_msgs_v2_10/LaneGroupId.h"
#include "struct_msgs/LaneId.h"
#include "struct_msgs_v2_10/LaneId.h"
#include "struct_msgs/LanePortId.h"
#include "struct_msgs_v2_10/LanePortId.h"
#include "struct_msgs/LaneSegment.h"
#include "struct_msgs_v2_10/LaneSegment.h"
#include "struct_msgs/ParallelLaneBoundary.h"
#include "struct_msgs_v2_10/ParallelLaneBoundary.h"
#include "struct_msgs/ParallelLaneBoundaryId.h"
#include "struct_msgs_v2_10/ParallelLaneBoundaryId.h"
#include "struct_msgs/ParkingInfo.h"
#include "struct_msgs_v2_10/ParkingInfo.h"
#include "struct_msgs/ParkingLaneBoundary.h"
#include "struct_msgs_v2_10/ParkingLaneBoundary.h"
#include "struct_msgs/ParkingMapFileInfo.h"
#include "struct_msgs_v2_10/ParkingMapFileInfo.h"
#include "struct_msgs/ParkingMapLimiter.h"
#include "struct_msgs_v2_10/ParkingMapLimiter.h"
#include "struct_msgs/ParkingSpace.h"
#include "struct_msgs_v2_10/ParkingSpace.h"
#include "struct_msgs/PolygonObject.h"
#include "struct_msgs_v2_10/PolygonObject.h"
#include "struct_msgs/Polyline.h"
#include "struct_msgs_v2_10/Polyline.h"
#include "struct_msgs/RampInfo.h"
#include "struct_msgs_v2_10/RampInfo.h"
#include "struct_msgs/Reference.h"
#include "struct_msgs_v2_10/Reference.h"
#include "struct_msgs/Road.h"
#include "struct_msgs_v2_10/Road.h"
#include "struct_msgs/RoadId.h"
#include "struct_msgs_v2_10/RoadId.h"
#include "struct_msgs/RoadMark.h"
#include "struct_msgs_v2_10/RoadMark.h"
#include "struct_msgs/RoadObstacle.h"
#include "struct_msgs_v2_10/RoadObstacle.h"
#include "struct_msgs/RoadPortId.h"
#include "struct_msgs_v2_10/RoadPortId.h"
#include "struct_msgs/RoadTile.h"
#include "struct_msgs_v2_10/RoadTile.h"
#include "struct_msgs/Segment.h"
#include "struct_msgs_v2_10/Segment.h"
#include "struct_msgs/SemanticInfo.h"
#include "struct_msgs_v2_10/SemanticInfo.h"
#include "struct_msgs/TargetParkingId.h"
#include "struct_msgs_v2_10/TargetParkingId.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::AttributeId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.tile_id, ros_v.tile_id, type);
	convert(old_ros_v.count, ros_v.count, type);
	convert(old_ros_v.floor_id, ros_v.floor_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Coordinate &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.coordinate_type, ros_v.coordinate_type, type);
	convert(old_ros_v.llh, ros_v.llh, type);
	convert(old_ros_v.enu, ros_v.enu, type);
	convert(old_ros_v.boot, ros_v.boot, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::EhpOutput &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.map_file_size, ros_v.map_file_size, type);
	ros_v.map_file.resize(old_ros_v.map_file.size());
	for (int i = 0; i < ros_v.map_file.size(); i++) {
	    convert(old_ros_v.map_file[i], ros_v.map_file[i], type);
	}
	convert(old_ros_v.ehp_status, ros_v.ehp_status, type);
	for (int i = 0; i < 128; i++) {
	    convert(old_ros_v.failed_reason[i], ros_v.failed_reason[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::EhpfileInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.map_file_id, ros_v.map_file_id, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.map_file_name[i], ros_v.map_file_name[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::FeaturePoint &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.available, ros_v.available, type);
	convert(old_ros_v.global_pos, ros_v.global_pos, type);
	convert(old_ros_v.local_pos, ros_v.local_pos, type);
	convert(old_ros_v.global_quat, ros_v.global_quat, type);
	convert(old_ros_v.local_quat, ros_v.local_quat, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::IntersectionId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.tile_id, ros_v.tile_id, type);
	convert(old_ros_v.count, ros_v.count, type);
	convert(old_ros_v.ur_id, ros_v.ur_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Lane &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.lane_type, ros_v.lane_type, type);
	convert(old_ros_v.center_line, ros_v.center_line, type);
	convert(old_ros_v.left_boundary_group, ros_v.left_boundary_group, type);
	convert(old_ros_v.right_boundary_group, ros_v.right_boundary_group, type);
	convert(old_ros_v.head_id, ros_v.head_id, type);
	convert(old_ros_v.tail_id, ros_v.tail_id, type);
	convert(old_ros_v.direction, ros_v.direction, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneBoundaryGroup &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.parallel_lane_boundaries_size, ros_v.parallel_lane_boundaries_size, type);
	ros_v.parallel_lane_boundaries.resize(old_ros_v.parallel_lane_boundaries.size());
	for (int i = 0; i < ros_v.parallel_lane_boundaries.size(); i++) {
	    convert(old_ros_v.parallel_lane_boundaries[i], ros_v.parallel_lane_boundaries[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneBoundaryGroupId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_group_id, ros_v.lane_group_id, type);
	convert(old_ros_v.index, ros_v.index, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneBoundaryId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.parallel_lane_boundary_id, ros_v.parallel_lane_boundary_id, type);
	convert(old_ros_v.index, ros_v.index, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneGroup &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.lanes_size, ros_v.lanes_size, type);
	ros_v.lanes.resize(old_ros_v.lanes.size());
	for (int i = 0; i < ros_v.lanes.size(); i++) {
	    convert(old_ros_v.lanes[i], ros_v.lanes[i], type);
	}
	convert(old_ros_v.lane_boundary_groups_size, ros_v.lane_boundary_groups_size, type);
	ros_v.lane_boundary_groups.resize(old_ros_v.lane_boundary_groups.size());
	for (int i = 0; i < ros_v.lane_boundary_groups.size(); i++) {
	    convert(old_ros_v.lane_boundary_groups[i], ros_v.lane_boundary_groups[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneGroupId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.road_id, ros_v.road_id, type);
	convert(old_ros_v.dir_reversed, ros_v.dir_reversed, type);
	convert(old_ros_v.index, ros_v.index, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_group_id, ros_v.lane_group_id, type);
	convert(old_ros_v.index, ros_v.index, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LanePortId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.tile_id, ros_v.tile_id, type);
	convert(old_ros_v.count, ros_v.count, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::LaneSegment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_id, ros_v.lane_id, type);
	convert(old_ros_v.segs_size, ros_v.segs_size, type);
	ros_v.segs.resize(old_ros_v.segs.size());
	for (int i = 0; i < ros_v.segs.size(); i++) {
	    convert(old_ros_v.segs[i], ros_v.segs[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParallelLaneBoundary &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.sequential_lane_boundaries_size, ros_v.sequential_lane_boundaries_size, type);
	ros_v.sequential_lane_boundaries.resize(old_ros_v.sequential_lane_boundaries.size());
	for (int i = 0; i < ros_v.sequential_lane_boundaries.size(); i++) {
	    convert(old_ros_v.sequential_lane_boundaries[i], ros_v.sequential_lane_boundaries[i], type);
	}
	convert(old_ros_v.id, ros_v.id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParallelLaneBoundaryId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.lane_boundary_group_id, ros_v.lane_boundary_group_id, type);
	convert(old_ros_v.index, ros_v.index, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.timestamp, ros_v.timestamp, type);
	convert(old_ros_v.perception_isp_timestamp, ros_v.perception_isp_timestamp, type);
	convert(old_ros_v.trajectory_ref_point, ros_v.trajectory_ref_point, type);
	convert(old_ros_v.trace_start, ros_v.trace_start, type);
	convert(old_ros_v.trace_dest, ros_v.trace_dest, type);
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.target_prk_pos[i], ros_v.target_prk_pos[i], type);
	}
	convert(old_ros_v.target_prk_id, ros_v.target_prk_id, type);
	convert(old_ros_v.road_tile_info, ros_v.road_tile_info, type);
	convert(old_ros_v.semantic_info, ros_v.semantic_info, type);
	convert(old_ros_v.map_id, ros_v.map_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingLaneBoundary &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.line_type, ros_v.line_type, type);
	convert(old_ros_v.lane_boundary_type, ros_v.lane_boundary_type, type);
	convert(old_ros_v.geometry, ros_v.geometry, type);
	convert(old_ros_v.color, ros_v.color, type);
	convert(old_ros_v.source, ros_v.source, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingMapFileInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.file_name[i], ros_v.file_name[i], type);
	}
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.file_path[i], ros_v.file_path[i], type);
	}
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.vehicle_vin[i], ros_v.vehicle_vin[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingMapLimiter &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	for (int i = 0; i < 2; i++) {
	    convert(old_ros_v.end_points[i], ros_v.end_points[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::ParkingSpace &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.parking_space_type, ros_v.parking_space_type, type);
	convert(old_ros_v.ref_size, ros_v.ref_size, type);
	ros_v.ref.resize(old_ros_v.ref.size());
	for (int i = 0; i < ros_v.ref.size(); i++) {
	    convert(old_ros_v.ref[i], ros_v.ref[i], type);
	}
	for (int i = 0; i < 4; i++) {
	    convert(old_ros_v.shape[i], ros_v.shape[i], type);
	}
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.floor_name[i], ros_v.floor_name[i], type);
	}
	convert(old_ros_v.allow_parking, ros_v.allow_parking, type);
	convert(old_ros_v.slot_source, ros_v.slot_source, type);
	convert(old_ros_v.limiters_size, ros_v.limiters_size, type);
	ros_v.limiters.resize(old_ros_v.limiters.size());
	for (int i = 0; i < ros_v.limiters.size(); i++) {
	    convert(old_ros_v.limiters[i], ros_v.limiters[i], type);
	}
	convert(old_ros_v.empty_votes, ros_v.empty_votes, type);
	convert(old_ros_v.is_turn_corner, ros_v.is_turn_corner, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::PolygonObject &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.ref_size, ros_v.ref_size, type);
	ros_v.ref.resize(old_ros_v.ref.size());
	for (int i = 0; i < ros_v.ref.size(); i++) {
	    convert(old_ros_v.ref[i], ros_v.ref[i], type);
	}
	convert(old_ros_v.corner_size, ros_v.corner_size, type);
	for (int i = 0; i < 10; i++) {
	    convert(old_ros_v.corners[i], ros_v.corners[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Polyline &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.points_size, ros_v.points_size, type);
	ros_v.points.resize(old_ros_v.points.size());
	for (int i = 0; i < ros_v.points.size(); i++) {
	    convert(old_ros_v.points[i], ros_v.points[i], type);
	}
	convert(old_ros_v.length, ros_v.length, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.buffer[i], ros_v.buffer[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RampInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.ramp_type, ros_v.ramp_type, type);
	convert(old_ros_v.entrance_ramp_point, ros_v.entrance_ramp_point, type);
	convert(old_ros_v.exit_ramp_point, ros_v.exit_ramp_point, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Reference &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.road_id, ros_v.road_id, type);
	convert(old_ros_v.lane_id, ros_v.lane_id, type);
	convert(old_ros_v.attribute_id, ros_v.attribute_id, type);
	convert(old_ros_v.road_port, ros_v.road_port, type);
	convert(old_ros_v.lane_port, ros_v.lane_port, type);
	convert(old_ros_v.offset, ros_v.offset, type);
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.orientation, ros_v.orientation, type);
	convert(old_ros_v.id_type, ros_v.id_type, type);
	convert(old_ros_v.fully_covered, ros_v.fully_covered, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Road &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.head_id, ros_v.head_id, type);
	convert(old_ros_v.tail_id, ros_v.tail_id, type);
	convert(old_ros_v.length, ros_v.length, type);
	convert(old_ros_v.road_type, ros_v.road_type, type);
	convert(old_ros_v.road_center, ros_v.road_center, type);
	convert(old_ros_v.travel_direction, ros_v.travel_direction, type);
	convert(old_ros_v.ramp, ros_v.ramp, type);
	for (int i = 0; i < 64; i++) {
	    convert(old_ros_v.name[i], ros_v.name[i], type);
	}
	convert(old_ros_v.road_class, ros_v.road_class, type);
	convert(old_ros_v.road_structure_size, ros_v.road_structure_size, type);
	ros_v.road_structure.resize(old_ros_v.road_structure.size());
	for (int i = 0; i < ros_v.road_structure.size(); i++) {
	    convert(old_ros_v.road_structure[i], ros_v.road_structure[i], type);
	}
	convert(old_ros_v.intersection_internal, ros_v.intersection_internal, type);
	convert(old_ros_v.intersection_id, ros_v.intersection_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RoadId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.tile_id, ros_v.tile_id, type);
	convert(old_ros_v.count, ros_v.count, type);
	convert(old_ros_v.ur_id, ros_v.ur_id, type);
	convert(old_ros_v.floor_id, ros_v.floor_id, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RoadMark &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.ref_size, ros_v.ref_size, type);
	ros_v.ref.resize(old_ros_v.ref.size());
	for (int i = 0; i < ros_v.ref.size(); i++) {
	    convert(old_ros_v.ref[i], ros_v.ref[i], type);
	}
	convert(old_ros_v.shape_size, ros_v.shape_size, type);
	ros_v.shape.resize(old_ros_v.shape.size());
	for (int i = 0; i < ros_v.shape.size(); i++) {
	    convert(old_ros_v.shape[i], ros_v.shape[i], type);
	}
	convert(old_ros_v.angle, ros_v.angle, type);
	convert(old_ros_v.direction, ros_v.direction, type);
	convert(old_ros_v.painting_type, ros_v.painting_type, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RoadObstacle &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.type, ros_v.type, type);
	convert(old_ros_v.ref_size, ros_v.ref_size, type);
	ros_v.ref.resize(old_ros_v.ref.size());
	for (int i = 0; i < ros_v.ref.size(); i++) {
	    convert(old_ros_v.ref[i], ros_v.ref[i], type);
	}
	convert(old_ros_v.shape_size, ros_v.shape_size, type);
	ros_v.shape.resize(old_ros_v.shape.size());
	for (int i = 0; i < ros_v.shape.size(); i++) {
	    convert(old_ros_v.shape[i], ros_v.shape[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RoadPortId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.tile_id, ros_v.tile_id, type);
	convert(old_ros_v.count, ros_v.count, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RoadTile &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.id, ros_v.id, type);
	convert(old_ros_v.road_size, ros_v.road_size, type);
	ros_v.road.resize(old_ros_v.road.size());
	for (int i = 0; i < ros_v.road.size(); i++) {
	    convert(old_ros_v.road[i], ros_v.road[i], type);
	}
	convert(old_ros_v.lane_segment_size, ros_v.lane_segment_size, type);
	ros_v.lane_segment.resize(old_ros_v.lane_segment.size());
	for (int i = 0; i < ros_v.lane_segment.size(); i++) {
	    convert(old_ros_v.lane_segment[i], ros_v.lane_segment[i], type);
	}
	convert(old_ros_v.road_mark_size, ros_v.road_mark_size, type);
	ros_v.road_mark.resize(old_ros_v.road_mark.size());
	for (int i = 0; i < ros_v.road_mark.size(); i++) {
	    convert(old_ros_v.road_mark[i], ros_v.road_mark[i], type);
	}
	convert(old_ros_v.road_obstacle_size, ros_v.road_obstacle_size, type);
	ros_v.road_obstacle.resize(old_ros_v.road_obstacle.size());
	for (int i = 0; i < ros_v.road_obstacle.size(); i++) {
	    convert(old_ros_v.road_obstacle[i], ros_v.road_obstacle[i], type);
	}
	convert(old_ros_v.parking_space_size, ros_v.parking_space_size, type);
	ros_v.parking_space.resize(old_ros_v.parking_space.size());
	for (int i = 0; i < ros_v.parking_space.size(); i++) {
	    convert(old_ros_v.parking_space[i], ros_v.parking_space[i], type);
	}
	convert(old_ros_v.polygon_obstacle_size, ros_v.polygon_obstacle_size, type);
	ros_v.polygon_obstacle.resize(old_ros_v.polygon_obstacle.size());
	for (int i = 0; i < ros_v.polygon_obstacle.size(); i++) {
	    convert(old_ros_v.polygon_obstacle[i], ros_v.polygon_obstacle[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::Segment &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.start_index, ros_v.start_index, type);
	convert(old_ros_v.end_index, ros_v.end_index, type);
	convert(old_ros_v.length, ros_v.length, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SemanticInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.msg_meta, ros_v.msg_meta, type);
	convert(old_ros_v.gps_signal_loss_point, ros_v.gps_signal_loss_point, type);
	convert(old_ros_v.ramp_info_size, ros_v.ramp_info_size, type);
	ros_v.ramp_info.resize(old_ros_v.ramp_info.size());
	for (int i = 0; i < ros_v.ramp_info.size(); i++) {
	    convert(old_ros_v.ramp_info[i], ros_v.ramp_info[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::TargetParkingId &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.attribute_id, ros_v.attribute_id, type);
	convert(old_ros_v.ref, ros_v.ref, type);
}

REG_CONVERT_SINGLE(_iflytek_ehp_map_manager_converter, "/iflytek/ehp/map_manager", EhpOutput);
REG_CONVERT_SINGLE(_iflytek_mega_local_map_converter, "/iflytek/mega/local_map", ParkingInfo);
REG_CONVERT_SINGLE(_iflytek_ehp_parking_map_file_info_converter, "/iflytek/ehp/parking_map_file_info", ParkingMapFileInfo);
REG_CONVERT_SINGLE(_iflytek_localization_semantic_info_converter, "/iflytek/localization/semantic_info", SemanticInfo);
