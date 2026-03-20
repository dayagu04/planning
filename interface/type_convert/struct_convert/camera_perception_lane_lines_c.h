#pragma once

#include "base_convert.h"
#include "c/camera_perception_lane_lines_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::SpeedInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.speed_limit, ros_v.speed_limit, type);
  convert(struct_v.lane_assignment, ros_v.lane_assignment, type);
  convert(struct_v.source, ros_v.source, type);
}

template <typename T2>
void convert(iflyauto::LineSegment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.a0, ros_v.a0, type);
  convert(struct_v.a1, ros_v.a1, type);
  convert(struct_v.a2, ros_v.a2, type);
  convert(struct_v.a3, ros_v.a3, type);
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::ColorSegment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.color, ros_v.color, type);
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
}

template <typename T2>
void convert(iflyauto::MarkingSegment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.marking, ros_v.marking, type);
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
}

template <typename T2>
void convert(iflyauto::LanePointAttr &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_point_marking, ros_v.lane_point_marking, type);
  convert(struct_v.lane_point_color, ros_v.lane_point_color, type);
  convert(struct_v.lane_point_confidence, ros_v.lane_point_confidence, type);
  convert(struct_v.lane_point_coordinate, ros_v.lane_point_coordinate, type);
}

template <typename T2>
void convert(iflyauto::LaneLine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.life_time, ros_v.life_time, type);
  convert(struct_v.source, ros_v.source, type);
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.pos_type, ros_v.pos_type, type);
  convert(struct_v.lane_points_attr_set_size, ros_v.lane_points_attr_set_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_points_attr_set_size >= 0 && struct_v.lane_points_attr_set_size <= CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM) {
      ros_v.lane_points_attr_set.resize(struct_v.lane_points_attr_set_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_points_attr_set_size=" << struct_v.lane_points_attr_set_size 
                << " not in range CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM=" << CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM 
                << std::endl;
      ros_v.lane_points_attr_set_size = CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM;
      ros_v.lane_points_attr_set.resize(CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lane_points_attr_set.size(); i0++) {
      convert(struct_v.lane_points_attr_set[i0], ros_v.lane_points_attr_set[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_points_attr_set_size > CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM || ros_v.lane_points_attr_set_size < 0 || ros_v.lane_points_attr_set.size() > CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_points_attr_set_size=" << ros_v.lane_points_attr_set_size 
                << " ros_v.lane_points_attr_set.size()=" << ros_v.lane_points_attr_set.size()
                << " not in range CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM=" << CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_points_attr_set.size() > CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM; i0++) {
        convert(struct_v.lane_points_attr_set[i0], ros_v.lane_points_attr_set[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lane_points_attr_set.size(); i0++) {
        convert(struct_v.lane_points_attr_set[i0], ros_v.lane_points_attr_set[i0], type);
      }
    }
  }
  //
  convert(struct_v.line_segments_size, ros_v.line_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.line_segments_size >= 0 && struct_v.line_segments_size <= CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM) {
      ros_v.line_segments.resize(struct_v.line_segments_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] line_segments_size=" << struct_v.line_segments_size 
                << " not in range CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM 
                << std::endl;
      ros_v.line_segments_size = CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM;
      ros_v.line_segments.resize(CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.line_segments.size(); i1++) {
      convert(struct_v.line_segments[i1], ros_v.line_segments[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.line_segments_size > CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM || ros_v.line_segments_size < 0 || ros_v.line_segments.size() > CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] line_segments_size=" << ros_v.line_segments_size 
                << " ros_v.line_segments.size()=" << ros_v.line_segments.size()
                << " not in range CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.line_segments.size() > CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_LINE_SEGMENTS_MAX_NUM; i1++) {
        convert(struct_v.line_segments[i1], ros_v.line_segments[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.line_segments.size(); i1++) {
        convert(struct_v.line_segments[i1], ros_v.line_segments[i1], type);
      }
    }
  }
  //
  convert(struct_v.marking_segments_size, ros_v.marking_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.marking_segments_size >= 0 && struct_v.marking_segments_size <= CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM) {
      ros_v.marking_segments.resize(struct_v.marking_segments_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] marking_segments_size=" << struct_v.marking_segments_size 
                << " not in range CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM 
                << std::endl;
      ros_v.marking_segments_size = CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM;
      ros_v.marking_segments.resize(CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.marking_segments.size(); i2++) {
      convert(struct_v.marking_segments[i2], ros_v.marking_segments[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.marking_segments_size > CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM || ros_v.marking_segments_size < 0 || ros_v.marking_segments.size() > CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] marking_segments_size=" << ros_v.marking_segments_size 
                << " ros_v.marking_segments.size()=" << ros_v.marking_segments.size()
                << " not in range CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.marking_segments.size() > CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM) {
      for (size_t i2 = 0; i2 < CAMERA_PERCEPTION_MARKING_SEGMENTS_MAX_NUM; i2++) {
        convert(struct_v.marking_segments[i2], ros_v.marking_segments[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.marking_segments.size(); i2++) {
        convert(struct_v.marking_segments[i2], ros_v.marking_segments[i2], type);
      }
    }
  }
  //
  convert(struct_v.color_segments_size, ros_v.color_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.color_segments_size >= 0 && struct_v.color_segments_size <= CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM) {
      ros_v.color_segments.resize(struct_v.color_segments_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] color_segments_size=" << struct_v.color_segments_size 
                << " not in range CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM 
                << std::endl;
      ros_v.color_segments_size = CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM;
      ros_v.color_segments.resize(CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.color_segments.size(); i3++) {
      convert(struct_v.color_segments[i3], ros_v.color_segments[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.color_segments_size > CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM || ros_v.color_segments_size < 0 || ros_v.color_segments.size() > CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] color_segments_size=" << ros_v.color_segments_size 
                << " ros_v.color_segments.size()=" << ros_v.color_segments.size()
                << " not in range CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM=" << CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.color_segments.size() > CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM) {
      for (size_t i3 = 0; i3 < CAMERA_PERCEPTION_COLOR_SEGMENTS_MAX_NUM; i3++) {
        convert(struct_v.color_segments[i3], ros_v.color_segments[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.color_segments.size(); i3++) {
        convert(struct_v.color_segments[i3], ros_v.color_segments[i3], type);
      }
    }
  }
  //
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::LanePositionSeg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.left_to_right_id, ros_v.left_to_right_id, type);
  convert(struct_v.right_to_left_id, ros_v.right_to_left_id, type);
}

template <typename T2>
void convert(iflyauto::LaneMatchFlag &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.match_current_lane, ros_v.match_current_lane, type);
}

template <typename T2>
void convert(iflyauto::LaneData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.id, ros_v.id, type);
  convert(struct_v.order_id, ros_v.order_id, type);
  convert(struct_v.central_line, ros_v.central_line, type);
  convert(struct_v.lane_types_size, ros_v.lane_types_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_types_size >= 0 && struct_v.lane_types_size <= CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      ros_v.lane_types.resize(struct_v.lane_types_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_types_size=" << struct_v.lane_types_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
      ros_v.lane_types_size = CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM;
      ros_v.lane_types.resize(CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lane_types.size(); i0++) {
      convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_types_size > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM || ros_v.lane_types_size < 0 || ros_v.lane_types.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_types_size=" << ros_v.lane_types_size 
                << " ros_v.lane_types.size()=" << ros_v.lane_types.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_types.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM; i0++) {
        convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lane_types.size(); i0++) {
        convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
      }
    }
  }
  //
  convert(struct_v.turn_types_size, ros_v.turn_types_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.turn_types_size >= 0 && struct_v.turn_types_size <= CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      ros_v.turn_types.resize(struct_v.turn_types_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] turn_types_size=" << struct_v.turn_types_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
      ros_v.turn_types_size = CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM;
      ros_v.turn_types.resize(CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.turn_types.size(); i1++) {
      convert(struct_v.turn_types[i1], ros_v.turn_types[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.turn_types_size > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM || ros_v.turn_types_size < 0 || ros_v.turn_types.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] turn_types_size=" << ros_v.turn_types_size 
                << " ros_v.turn_types.size()=" << ros_v.turn_types.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
    }
    if (ros_v.turn_types.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM; i1++) {
        convert(struct_v.turn_types[i1], ros_v.turn_types[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.turn_types.size(); i1++) {
        convert(struct_v.turn_types[i1], ros_v.turn_types[i1], type);
      }
    }
  }
  //
  convert(struct_v.position_segs_size, ros_v.position_segs_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.position_segs_size >= 0 && struct_v.position_segs_size <= CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      ros_v.position_segs.resize(struct_v.position_segs_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] position_segs_size=" << struct_v.position_segs_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
      ros_v.position_segs_size = CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM;
      ros_v.position_segs.resize(CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.position_segs.size(); i2++) {
      convert(struct_v.position_segs[i2], ros_v.position_segs[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.position_segs_size > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM || ros_v.position_segs_size < 0 || ros_v.position_segs.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] position_segs_size=" << ros_v.position_segs_size 
                << " ros_v.position_segs.size()=" << ros_v.position_segs.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
    }
    if (ros_v.position_segs.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      for (size_t i2 = 0; i2 < CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM; i2++) {
        convert(struct_v.position_segs[i2], ros_v.position_segs[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.position_segs.size(); i2++) {
        convert(struct_v.position_segs[i2], ros_v.position_segs[i2], type);
      }
    }
  }
  //
  convert(struct_v.match_flags_size, ros_v.match_flags_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.match_flags_size >= 0 && struct_v.match_flags_size <= CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      ros_v.match_flags.resize(struct_v.match_flags_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] match_flags_size=" << struct_v.match_flags_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
      ros_v.match_flags_size = CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM;
      ros_v.match_flags.resize(CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.match_flags.size(); i3++) {
      convert(struct_v.match_flags[i3], ros_v.match_flags[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.match_flags_size > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM || ros_v.match_flags_size < 0 || ros_v.match_flags.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] match_flags_size=" << ros_v.match_flags_size 
                << " ros_v.match_flags.size()=" << ros_v.match_flags.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
    }
    if (ros_v.match_flags.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      for (size_t i3 = 0; i3 < CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM; i3++) {
        convert(struct_v.match_flags[i3], ros_v.match_flags[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.match_flags.size(); i3++) {
        convert(struct_v.match_flags[i3], ros_v.match_flags[i3], type);
      }
    }
  }
  //
  convert(struct_v.left_lane_boundary_id, ros_v.left_lane_boundary_id, type);
  convert(struct_v.right_lane_boundary_id, ros_v.right_lane_boundary_id, type);
  convert(struct_v.left_road_boundary_id, ros_v.left_road_boundary_id, type);
  convert(struct_v.right_road_boundary_id, ros_v.right_road_boundary_id, type);
  convert(struct_v.stop_line_id, ros_v.stop_line_id, type);
  convert(struct_v.merge_split_points_size, ros_v.merge_split_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.merge_split_points_size >= 0 && struct_v.merge_split_points_size <= CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      ros_v.merge_split_points.resize(struct_v.merge_split_points_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] merge_split_points_size=" << struct_v.merge_split_points_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
      ros_v.merge_split_points_size = CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM;
      ros_v.merge_split_points.resize(CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM);
    }
    for (size_t i4 = 0; i4 < ros_v.merge_split_points.size(); i4++) {
      convert(struct_v.merge_split_points[i4], ros_v.merge_split_points[i4], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.merge_split_points_size > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM || ros_v.merge_split_points_size < 0 || ros_v.merge_split_points.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] merge_split_points_size=" << ros_v.merge_split_points_size 
                << " ros_v.merge_split_points.size()=" << ros_v.merge_split_points.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM 
                << std::endl;
    }
    if (ros_v.merge_split_points.size() > CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM) {
      for (size_t i4 = 0; i4 < CAMERA_PERCEPTION_LANE_LINE_ATTR_MAX_NUM; i4++) {
        convert(struct_v.merge_split_points[i4], ros_v.merge_split_points[i4], type);
      }
    } else {
      for (size_t i4 = 0; i4 < ros_v.merge_split_points.size(); i4++) {
        convert(struct_v.merge_split_points[i4], ros_v.merge_split_points[i4], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::LaneGroundMarking &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ground_marking_points_set_size, ros_v.ground_marking_points_set_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ground_marking_points_set_size >= 0 && struct_v.ground_marking_points_set_size <= CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM) {
      ros_v.ground_marking_points_set.resize(struct_v.ground_marking_points_set_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ground_marking_points_set_size=" << struct_v.ground_marking_points_set_size 
                << " not in range CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM=" << CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM 
                << std::endl;
      ros_v.ground_marking_points_set_size = CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM;
      ros_v.ground_marking_points_set.resize(CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.ground_marking_points_set.size(); i0++) {
      convert(struct_v.ground_marking_points_set[i0], ros_v.ground_marking_points_set[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ground_marking_points_set_size > CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM || ros_v.ground_marking_points_set_size < 0 || ros_v.ground_marking_points_set.size() > CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ground_marking_points_set_size=" << ros_v.ground_marking_points_set_size 
                << " ros_v.ground_marking_points_set.size()=" << ros_v.ground_marking_points_set.size()
                << " not in range CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM=" << CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM 
                << std::endl;
    }
    if (ros_v.ground_marking_points_set.size() > CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_LANE_POINTS_SET_MAX_NUM; i0++) {
        convert(struct_v.ground_marking_points_set[i0], ros_v.ground_marking_points_set[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ground_marking_points_set.size(); i0++) {
        convert(struct_v.ground_marking_points_set[i0], ros_v.ground_marking_points_set[i0], type);
      }
    }
  }
  //
  convert(struct_v.orientation_angle, ros_v.orientation_angle, type);
  convert(struct_v.turn_type, ros_v.turn_type, type);
  convert(struct_v.track_id, ros_v.track_id, type);
}

template <typename T2>
void convert(iflyauto::LaneLineSet &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.lanes_size, ros_v.lanes_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lanes_size >= 0 && struct_v.lanes_size <= CAMERA_PERCEPTION_LANE_MAX_NUM) {
      ros_v.lanes.resize(struct_v.lanes_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lanes_size=" << struct_v.lanes_size 
                << " not in range CAMERA_PERCEPTION_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_MAX_NUM 
                << std::endl;
      ros_v.lanes_size = CAMERA_PERCEPTION_LANE_MAX_NUM;
      ros_v.lanes.resize(CAMERA_PERCEPTION_LANE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lanes.size(); i0++) {
      convert(struct_v.lanes[i0], ros_v.lanes[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lanes_size > CAMERA_PERCEPTION_LANE_MAX_NUM || ros_v.lanes_size < 0 || ros_v.lanes.size() > CAMERA_PERCEPTION_LANE_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lanes_size=" << ros_v.lanes_size 
                << " ros_v.lanes.size()=" << ros_v.lanes.size()
                << " not in range CAMERA_PERCEPTION_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lanes.size() > CAMERA_PERCEPTION_LANE_MAX_NUM) {
      for (size_t i0 = 0; i0 < CAMERA_PERCEPTION_LANE_MAX_NUM; i0++) {
        convert(struct_v.lanes[i0], ros_v.lanes[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lanes.size(); i0++) {
        convert(struct_v.lanes[i0], ros_v.lanes[i0], type);
      }
    }
  }
  //
  convert(struct_v.lane_line_size, ros_v.lane_line_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_line_size >= 0 && struct_v.lane_line_size <= CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM) {
      ros_v.lane_line.resize(struct_v.lane_line_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_line_size=" << struct_v.lane_line_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM 
                << std::endl;
      ros_v.lane_line_size = CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM;
      ros_v.lane_line.resize(CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.lane_line.size(); i1++) {
      convert(struct_v.lane_line[i1], ros_v.lane_line[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_line_size > CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM || ros_v.lane_line_size < 0 || ros_v.lane_line.size() > CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_line_size=" << ros_v.lane_line_size 
                << " ros_v.lane_line.size()=" << ros_v.lane_line.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_line.size() > CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM) {
      for (size_t i1 = 0; i1 < CAMERA_PERCEPTION_LANE_LINE_SET_LANE_MAX_NUM; i1++) {
        convert(struct_v.lane_line[i1], ros_v.lane_line[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.lane_line.size(); i1++) {
        convert(struct_v.lane_line[i1], ros_v.lane_line[i1], type);
      }
    }
  }
  //
  convert(struct_v.lane_ground_markings_size, ros_v.lane_ground_markings_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_ground_markings_size >= 0 && struct_v.lane_ground_markings_size <= CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM) {
      ros_v.lane_ground_markings.resize(struct_v.lane_ground_markings_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_ground_markings_size=" << struct_v.lane_ground_markings_size 
                << " not in range CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM=" << CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM 
                << std::endl;
      ros_v.lane_ground_markings_size = CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM;
      ros_v.lane_ground_markings.resize(CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.lane_ground_markings.size(); i2++) {
      convert(struct_v.lane_ground_markings[i2], ros_v.lane_ground_markings[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_ground_markings_size > CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM || ros_v.lane_ground_markings_size < 0 || ros_v.lane_ground_markings.size() > CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_ground_markings_size=" << ros_v.lane_ground_markings_size 
                << " ros_v.lane_ground_markings.size()=" << ros_v.lane_ground_markings.size()
                << " not in range CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM=" << CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_ground_markings.size() > CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM) {
      for (size_t i2 = 0; i2 < CAMERA_PERCEPTION_LANE_GROUND_MARKING_MAX_NUM; i2++) {
        convert(struct_v.lane_ground_markings[i2], ros_v.lane_ground_markings[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.lane_ground_markings.size(); i2++) {
        convert(struct_v.lane_ground_markings[i2], ros_v.lane_ground_markings[i2], type);
      }
    }
  }
  //
  convert(struct_v.stop_line_size, ros_v.stop_line_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.stop_line_size >= 0 && struct_v.stop_line_size <= CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM) {
      ros_v.stop_line.resize(struct_v.stop_line_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] stop_line_size=" << struct_v.stop_line_size 
                << " not in range CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM 
                << std::endl;
      ros_v.stop_line_size = CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM;
      ros_v.stop_line.resize(CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.stop_line.size(); i3++) {
      convert(struct_v.stop_line[i3], ros_v.stop_line[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.stop_line_size > CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM || ros_v.stop_line_size < 0 || ros_v.stop_line.size() > CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] stop_line_size=" << ros_v.stop_line_size 
                << " ros_v.stop_line.size()=" << ros_v.stop_line.size()
                << " not in range CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.stop_line.size() > CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM) {
      for (size_t i3 = 0; i3 < CAMERA_PERCEPTION_STOP_LANE_LINE_MAX_NUM; i3++) {
        convert(struct_v.stop_line[i3], ros_v.stop_line[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.stop_line.size(); i3++) {
        convert(struct_v.stop_line[i3], ros_v.stop_line[i3], type);
      }
    }
  }
  //
  convert(struct_v.inhibit_line_size, ros_v.inhibit_line_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.inhibit_line_size >= 0 && struct_v.inhibit_line_size <= CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM) {
      ros_v.inhibit_line.resize(struct_v.inhibit_line_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] inhibit_line_size=" << struct_v.inhibit_line_size 
                << " not in range CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM 
                << std::endl;
      ros_v.inhibit_line_size = CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM;
      ros_v.inhibit_line.resize(CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM);
    }
    for (size_t i4 = 0; i4 < ros_v.inhibit_line.size(); i4++) {
      convert(struct_v.inhibit_line[i4], ros_v.inhibit_line[i4], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.inhibit_line_size > CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM || ros_v.inhibit_line_size < 0 || ros_v.inhibit_line.size() > CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] inhibit_line_size=" << ros_v.inhibit_line_size 
                << " ros_v.inhibit_line.size()=" << ros_v.inhibit_line.size()
                << " not in range CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM=" << CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.inhibit_line.size() > CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM) {
      for (size_t i4 = 0; i4 < CAMERA_PERCEPTION_INHIBIT_LANE_LINE_MAX_NUM; i4++) {
        convert(struct_v.inhibit_line[i4], ros_v.inhibit_line[i4], type);
      }
    } else {
      for (size_t i4 = 0; i4 < ros_v.inhibit_line.size(); i4++) {
        convert(struct_v.inhibit_line[i4], ros_v.inhibit_line[i4], type);
      }
    }
  }
  //
  convert(struct_v.speed_info_size, ros_v.speed_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.speed_info_size >= 0 && struct_v.speed_info_size <= CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM) {
      ros_v.speed_info.resize(struct_v.speed_info_size);
    } else {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << " [convert][TO_ROS] speed_info_size=" << struct_v.speed_info_size 
                << " not in range CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM 
                << std::endl;
      ros_v.speed_info_size = CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM;
      ros_v.speed_info.resize(CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM);
    }
    for (size_t i5 = 0; i5 < ros_v.speed_info.size(); i5++) {
      convert(struct_v.speed_info[i5], ros_v.speed_info[i5], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.speed_info_size > CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM || ros_v.speed_info_size < 0 || ros_v.speed_info.size() > CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM) {
      std::cout << "convert/camera_perception_lane_lines_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] speed_info_size=" << ros_v.speed_info_size 
                << " ros_v.speed_info.size()=" << ros_v.speed_info.size()
                << " not in range CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM=" << CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM 
                << std::endl;
    }
    if (ros_v.speed_info.size() > CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM) {
      for (size_t i5 = 0; i5 < CAMERA_PERCEPTION_LANE_LINE_SET_SEED_MAX_NUM; i5++) {
        convert(struct_v.speed_info[i5], ros_v.speed_info[i5], type);
      }
    } else {
      for (size_t i5 = 0; i5 < ros_v.speed_info.size(); i5++) {
        convert(struct_v.speed_info[i5], ros_v.speed_info[i5], type);
      }
    }
  }
  //
  convert(struct_v.camera_perception_input_timestamp, ros_v.camera_perception_input_timestamp, type);
}

