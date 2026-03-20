#pragma once

#include "base_convert.h"
#include "c/fusion_road_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::ReferencePoint &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.track_id, ros_v.track_id, type);
  convert(struct_v.car_point, ros_v.car_point, type);
  convert(struct_v.enu_point, ros_v.enu_point, type);
  convert(struct_v.local_point, ros_v.local_point, type);
  convert(struct_v.curvature, ros_v.curvature, type);
  convert(struct_v.car_heading, ros_v.car_heading, type);
  convert(struct_v.enu_heading, ros_v.enu_heading, type);
  convert(struct_v.local_heading, ros_v.local_heading, type);
  convert(struct_v.distance_to_left_road_border, ros_v.distance_to_left_road_border, type);
  convert(struct_v.distance_to_right_road_border, ros_v.distance_to_right_road_border, type);
  convert(struct_v.distance_to_left_lane_border, ros_v.distance_to_left_lane_border, type);
  convert(struct_v.distance_to_right_lane_border, ros_v.distance_to_right_lane_border, type);
  convert(struct_v.lane_width, ros_v.lane_width, type);
  convert(struct_v.speed_limit_max, ros_v.speed_limit_max, type);
  convert(struct_v.speed_limit_min, ros_v.speed_limit_min, type);
  convert(struct_v.left_road_border_type, ros_v.left_road_border_type, type);
  convert(struct_v.right_road_border_type, ros_v.right_road_border_type, type);
  convert(struct_v.left_lane_border_type, ros_v.left_lane_border_type, type);
  convert(struct_v.right_lane_border_type, ros_v.right_lane_border_type, type);
  convert(struct_v.is_in_intersection, ros_v.is_in_intersection, type);
  convert(struct_v.lane_type, ros_v.lane_type, type);
  convert(struct_v.s, ros_v.s, type);
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::LaneReferenceLine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.virtual_lane_refline_points_size, ros_v.virtual_lane_refline_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.virtual_lane_refline_points_size >= 0 && struct_v.virtual_lane_refline_points_size <= FUSION_ROAD_REFLINE_POINT_MAX_NUM) {
      ros_v.virtual_lane_refline_points.resize(struct_v.virtual_lane_refline_points_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] virtual_lane_refline_points_size=" << struct_v.virtual_lane_refline_points_size 
                << " not in range FUSION_ROAD_REFLINE_POINT_MAX_NUM=" << FUSION_ROAD_REFLINE_POINT_MAX_NUM 
                << std::endl;
      ros_v.virtual_lane_refline_points_size = FUSION_ROAD_REFLINE_POINT_MAX_NUM;
      ros_v.virtual_lane_refline_points.resize(FUSION_ROAD_REFLINE_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.virtual_lane_refline_points.size(); i0++) {
      convert(struct_v.virtual_lane_refline_points[i0], ros_v.virtual_lane_refline_points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.virtual_lane_refline_points_size > FUSION_ROAD_REFLINE_POINT_MAX_NUM || ros_v.virtual_lane_refline_points_size < 0 || ros_v.virtual_lane_refline_points.size() > FUSION_ROAD_REFLINE_POINT_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] virtual_lane_refline_points_size=" << ros_v.virtual_lane_refline_points_size 
                << " ros_v.virtual_lane_refline_points.size()=" << ros_v.virtual_lane_refline_points.size()
                << " not in range FUSION_ROAD_REFLINE_POINT_MAX_NUM=" << FUSION_ROAD_REFLINE_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.virtual_lane_refline_points.size() > FUSION_ROAD_REFLINE_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_ROAD_REFLINE_POINT_MAX_NUM; i0++) {
        convert(struct_v.virtual_lane_refline_points[i0], ros_v.virtual_lane_refline_points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.virtual_lane_refline_points.size(); i0++) {
        convert(struct_v.virtual_lane_refline_points[i0], ros_v.virtual_lane_refline_points[i0], type);
      }
    }
  }
  //
  for (size_t i1 = 0; i1 < ros_v.poly_coefficient_car.size(); i1++) {
	  convert(struct_v.poly_coefficient_car[i1], ros_v.poly_coefficient_car[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::FusionLineSegment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.a0, ros_v.a0, type);
  convert(struct_v.a1, ros_v.a1, type);
  convert(struct_v.a2, ros_v.a2, type);
  convert(struct_v.a3, ros_v.a3, type);
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.confidence, ros_v.confidence, type);
}

template <typename T2>
void convert(iflyauto::LaneBoundaryColorSegment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.color, ros_v.color, type);
}

template <typename T2>
void convert(iflyauto::LaneBoundaryTypeSegment &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.type, ros_v.type, type);
}

template <typename T2>
void convert(iflyauto::LaneBoundary &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.existence, ros_v.existence, type);
  convert(struct_v.life_time, ros_v.life_time, type);
  convert(struct_v.track_id, ros_v.track_id, type);
  convert(struct_v.type, ros_v.type, type);
  for (size_t i0 = 0; i0 < ros_v.poly_coefficient.size(); i0++) {
	  convert(struct_v.poly_coefficient[i0], ros_v.poly_coefficient[i0], type);
  }
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.line_segments_size, ros_v.line_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.line_segments_size >= 0 && struct_v.line_segments_size <= FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      ros_v.line_segments.resize(struct_v.line_segments_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] line_segments_size=" << struct_v.line_segments_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_MAX_NUM 
                << std::endl;
      ros_v.line_segments_size = FUSION_ROAD_LANE_BOUNDARY_MAX_NUM;
      ros_v.line_segments.resize(FUSION_ROAD_LANE_BOUNDARY_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.line_segments.size(); i1++) {
      convert(struct_v.line_segments[i1], ros_v.line_segments[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.line_segments_size > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM || ros_v.line_segments_size < 0 || ros_v.line_segments.size() > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] line_segments_size=" << ros_v.line_segments_size 
                << " ros_v.line_segments.size()=" << ros_v.line_segments.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_MAX_NUM 
                << std::endl;
    }
    if (ros_v.line_segments.size() > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      for (size_t i1 = 0; i1 < FUSION_ROAD_LANE_BOUNDARY_MAX_NUM; i1++) {
        convert(struct_v.line_segments[i1], ros_v.line_segments[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.line_segments.size(); i1++) {
        convert(struct_v.line_segments[i1], ros_v.line_segments[i1], type);
      }
    }
  }
  //
  convert(struct_v.type_segments_size, ros_v.type_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.type_segments_size >= 0 && struct_v.type_segments_size <= FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      ros_v.type_segments.resize(struct_v.type_segments_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] type_segments_size=" << struct_v.type_segments_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_MAX_NUM 
                << std::endl;
      ros_v.type_segments_size = FUSION_ROAD_LANE_BOUNDARY_MAX_NUM;
      ros_v.type_segments.resize(FUSION_ROAD_LANE_BOUNDARY_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.type_segments.size(); i2++) {
      convert(struct_v.type_segments[i2], ros_v.type_segments[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.type_segments_size > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM || ros_v.type_segments_size < 0 || ros_v.type_segments.size() > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] type_segments_size=" << ros_v.type_segments_size 
                << " ros_v.type_segments.size()=" << ros_v.type_segments.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_MAX_NUM 
                << std::endl;
    }
    if (ros_v.type_segments.size() > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      for (size_t i2 = 0; i2 < FUSION_ROAD_LANE_BOUNDARY_MAX_NUM; i2++) {
        convert(struct_v.type_segments[i2], ros_v.type_segments[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.type_segments.size(); i2++) {
        convert(struct_v.type_segments[i2], ros_v.type_segments[i2], type);
      }
    }
  }
  //
  convert(struct_v.color_segments_size, ros_v.color_segments_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.color_segments_size >= 0 && struct_v.color_segments_size <= FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      ros_v.color_segments.resize(struct_v.color_segments_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] color_segments_size=" << struct_v.color_segments_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_MAX_NUM 
                << std::endl;
      ros_v.color_segments_size = FUSION_ROAD_LANE_BOUNDARY_MAX_NUM;
      ros_v.color_segments.resize(FUSION_ROAD_LANE_BOUNDARY_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.color_segments.size(); i3++) {
      convert(struct_v.color_segments[i3], ros_v.color_segments[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.color_segments_size > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM || ros_v.color_segments_size < 0 || ros_v.color_segments.size() > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] color_segments_size=" << ros_v.color_segments_size 
                << " ros_v.color_segments.size()=" << ros_v.color_segments.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_MAX_NUM 
                << std::endl;
    }
    if (ros_v.color_segments.size() > FUSION_ROAD_LANE_BOUNDARY_MAX_NUM) {
      for (size_t i3 = 0; i3 < FUSION_ROAD_LANE_BOUNDARY_MAX_NUM; i3++) {
        convert(struct_v.color_segments[i3], ros_v.color_segments[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.color_segments.size(); i3++) {
        convert(struct_v.color_segments[i3], ros_v.color_segments[i3], type);
      }
    }
  }
  //
  convert(struct_v.car_points_size, ros_v.car_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.car_points_size >= 0 && struct_v.car_points_size <= FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      ros_v.car_points.resize(struct_v.car_points_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] car_points_size=" << struct_v.car_points_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
      ros_v.car_points_size = FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM;
      ros_v.car_points.resize(FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM);
    }
    for (size_t i4 = 0; i4 < ros_v.car_points.size(); i4++) {
      convert(struct_v.car_points[i4], ros_v.car_points[i4], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.car_points_size > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM || ros_v.car_points_size < 0 || ros_v.car_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] car_points_size=" << ros_v.car_points_size 
                << " ros_v.car_points.size()=" << ros_v.car_points.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.car_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      for (size_t i4 = 0; i4 < FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM; i4++) {
        convert(struct_v.car_points[i4], ros_v.car_points[i4], type);
      }
    } else {
      for (size_t i4 = 0; i4 < ros_v.car_points.size(); i4++) {
        convert(struct_v.car_points[i4], ros_v.car_points[i4], type);
      }
    }
  }
  //
  convert(struct_v.enu_points_size, ros_v.enu_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.enu_points_size >= 0 && struct_v.enu_points_size <= FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      ros_v.enu_points.resize(struct_v.enu_points_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] enu_points_size=" << struct_v.enu_points_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
      ros_v.enu_points_size = FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM;
      ros_v.enu_points.resize(FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM);
    }
    for (size_t i5 = 0; i5 < ros_v.enu_points.size(); i5++) {
      convert(struct_v.enu_points[i5], ros_v.enu_points[i5], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.enu_points_size > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM || ros_v.enu_points_size < 0 || ros_v.enu_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] enu_points_size=" << ros_v.enu_points_size 
                << " ros_v.enu_points.size()=" << ros_v.enu_points.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.enu_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      for (size_t i5 = 0; i5 < FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM; i5++) {
        convert(struct_v.enu_points[i5], ros_v.enu_points[i5], type);
      }
    } else {
      for (size_t i5 = 0; i5 < ros_v.enu_points.size(); i5++) {
        convert(struct_v.enu_points[i5], ros_v.enu_points[i5], type);
      }
    }
  }
  //
  convert(struct_v.local_points_size, ros_v.local_points_size, type);
  for (size_t i6 = 0; i6 < ros_v.local_point.size(); i6++) {
	  convert(struct_v.local_point[i6], ros_v.local_point[i6], type);
  }
  for (size_t i7 = 0; i7 < ros_v.point_confidence_list.size(); i7++) {
	  convert(struct_v.point_confidence_list[i7], ros_v.point_confidence_list[i7], type);
  }
  convert(struct_v.line_width, ros_v.line_width, type);
}

template <typename T2>
void convert(iflyauto::LaneMergeSplitPoint &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.existence, ros_v.existence, type);
  convert(struct_v.merge_split_point_data_size, ros_v.merge_split_point_data_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.merge_split_point_data_size >= 0 && struct_v.merge_split_point_data_size <= FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM) {
      ros_v.merge_split_point_data.resize(struct_v.merge_split_point_data_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] merge_split_point_data_size=" << struct_v.merge_split_point_data_size 
                << " not in range FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM=" << FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM 
                << std::endl;
      ros_v.merge_split_point_data_size = FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM;
      ros_v.merge_split_point_data.resize(FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.merge_split_point_data.size(); i0++) {
      convert(struct_v.merge_split_point_data[i0], ros_v.merge_split_point_data[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.merge_split_point_data_size > FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM || ros_v.merge_split_point_data_size < 0 || ros_v.merge_split_point_data.size() > FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] merge_split_point_data_size=" << ros_v.merge_split_point_data_size 
                << " ros_v.merge_split_point_data.size()=" << ros_v.merge_split_point_data.size()
                << " not in range FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM=" << FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.merge_split_point_data.size() > FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_ROAD_LANE_SPLIT_POINT_MAX_NUM; i0++) {
        convert(struct_v.merge_split_point_data[i0], ros_v.merge_split_point_data[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.merge_split_point_data.size(); i0++) {
        convert(struct_v.merge_split_point_data[i0], ros_v.merge_split_point_data[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::LaneSourceMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.source, ros_v.source, type);
}

template <typename T2>
void convert(iflyauto::LaneNumMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.begin, ros_v.begin, type);
  convert(struct_v.end, ros_v.end, type);
  convert(struct_v.left_lane_num, ros_v.left_lane_num, type);
  convert(struct_v.right_lane_num, ros_v.right_lane_num, type);
}

template <typename T2>
void convert(iflyauto::ReferenceLineMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.order_id, ros_v.order_id, type);
  convert(struct_v.relative_id, ros_v.relative_id, type);
  convert(struct_v.lane_types_size, ros_v.lane_types_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_types_size >= 0 && struct_v.lane_types_size <= FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      ros_v.lane_types.resize(struct_v.lane_types_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_types_size=" << struct_v.lane_types_size 
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
      ros_v.lane_types_size = FUSION_ROAD_REFLANE_MSG_MAX_NUM;
      ros_v.lane_types.resize(FUSION_ROAD_REFLANE_MSG_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lane_types.size(); i0++) {
      convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_types_size > FUSION_ROAD_REFLANE_MSG_MAX_NUM || ros_v.lane_types_size < 0 || ros_v.lane_types.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_types_size=" << ros_v.lane_types_size 
                << " ros_v.lane_types.size()=" << ros_v.lane_types.size()
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_types.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_ROAD_REFLANE_MSG_MAX_NUM; i0++) {
        convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lane_types.size(); i0++) {
        convert(struct_v.lane_types[i0], ros_v.lane_types[i0], type);
      }
    }
  }
  //
  convert(struct_v.lane_marks_size, ros_v.lane_marks_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_marks_size >= 0 && struct_v.lane_marks_size <= FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      ros_v.lane_marks.resize(struct_v.lane_marks_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_marks_size=" << struct_v.lane_marks_size 
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
      ros_v.lane_marks_size = FUSION_ROAD_REFLANE_MSG_MAX_NUM;
      ros_v.lane_marks.resize(FUSION_ROAD_REFLANE_MSG_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.lane_marks.size(); i1++) {
      convert(struct_v.lane_marks[i1], ros_v.lane_marks[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_marks_size > FUSION_ROAD_REFLANE_MSG_MAX_NUM || ros_v.lane_marks_size < 0 || ros_v.lane_marks.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_marks_size=" << ros_v.lane_marks_size 
                << " ros_v.lane_marks.size()=" << ros_v.lane_marks.size()
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_marks.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      for (size_t i1 = 0; i1 < FUSION_ROAD_REFLANE_MSG_MAX_NUM; i1++) {
        convert(struct_v.lane_marks[i1], ros_v.lane_marks[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.lane_marks.size(); i1++) {
        convert(struct_v.lane_marks[i1], ros_v.lane_marks[i1], type);
      }
    }
  }
  //
  convert(struct_v.lane_sources_size, ros_v.lane_sources_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_sources_size >= 0 && struct_v.lane_sources_size <= FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      ros_v.lane_sources.resize(struct_v.lane_sources_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_sources_size=" << struct_v.lane_sources_size 
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
      ros_v.lane_sources_size = FUSION_ROAD_REFLANE_MSG_MAX_NUM;
      ros_v.lane_sources.resize(FUSION_ROAD_REFLANE_MSG_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.lane_sources.size(); i2++) {
      convert(struct_v.lane_sources[i2], ros_v.lane_sources[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_sources_size > FUSION_ROAD_REFLANE_MSG_MAX_NUM || ros_v.lane_sources_size < 0 || ros_v.lane_sources.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_sources_size=" << ros_v.lane_sources_size 
                << " ros_v.lane_sources.size()=" << ros_v.lane_sources.size()
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_sources.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      for (size_t i2 = 0; i2 < FUSION_ROAD_REFLANE_MSG_MAX_NUM; i2++) {
        convert(struct_v.lane_sources[i2], ros_v.lane_sources[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.lane_sources.size(); i2++) {
        convert(struct_v.lane_sources[i2], ros_v.lane_sources[i2], type);
      }
    }
  }
  //
  convert(struct_v.lane_num_size, ros_v.lane_num_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_num_size >= 0 && struct_v.lane_num_size <= FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      ros_v.lane_num.resize(struct_v.lane_num_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_num_size=" << struct_v.lane_num_size 
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
      ros_v.lane_num_size = FUSION_ROAD_REFLANE_MSG_MAX_NUM;
      ros_v.lane_num.resize(FUSION_ROAD_REFLANE_MSG_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.lane_num.size(); i3++) {
      convert(struct_v.lane_num[i3], ros_v.lane_num[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_num_size > FUSION_ROAD_REFLANE_MSG_MAX_NUM || ros_v.lane_num_size < 0 || ros_v.lane_num.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_num_size=" << ros_v.lane_num_size 
                << " ros_v.lane_num.size()=" << ros_v.lane_num.size()
                << " not in range FUSION_ROAD_REFLANE_MSG_MAX_NUM=" << FUSION_ROAD_REFLANE_MSG_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_num.size() > FUSION_ROAD_REFLANE_MSG_MAX_NUM) {
      for (size_t i3 = 0; i3 < FUSION_ROAD_REFLANE_MSG_MAX_NUM; i3++) {
        convert(struct_v.lane_num[i3], ros_v.lane_num[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.lane_num.size(); i3++) {
        convert(struct_v.lane_num[i3], ros_v.lane_num[i3], type);
      }
    }
  }
  //
  convert(struct_v.lane_reference_line, ros_v.lane_reference_line, type);
  convert(struct_v.lane_merge_split_point, ros_v.lane_merge_split_point, type);
  convert(struct_v.left_lane_boundary, ros_v.left_lane_boundary, type);
  convert(struct_v.right_lane_boundary, ros_v.right_lane_boundary, type);
  convert(struct_v.stop_line, ros_v.stop_line, type);
}

template <typename T2>
void convert(iflyauto::FusionLaneGroundMarking &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.ground_marking_points_set_size, ros_v.ground_marking_points_set_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.ground_marking_points_set_size >= 0 && struct_v.ground_marking_points_set_size <= FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      ros_v.ground_marking_points_set.resize(struct_v.ground_marking_points_set_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] ground_marking_points_set_size=" << struct_v.ground_marking_points_set_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
      ros_v.ground_marking_points_set_size = FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM;
      ros_v.ground_marking_points_set.resize(FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.ground_marking_points_set.size(); i0++) {
      convert(struct_v.ground_marking_points_set[i0], ros_v.ground_marking_points_set[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.ground_marking_points_set_size > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM || ros_v.ground_marking_points_set_size < 0 || ros_v.ground_marking_points_set.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] ground_marking_points_set_size=" << ros_v.ground_marking_points_set_size 
                << " ros_v.ground_marking_points_set.size()=" << ros_v.ground_marking_points_set.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.ground_marking_points_set.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM; i0++) {
        convert(struct_v.ground_marking_points_set[i0], ros_v.ground_marking_points_set[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.ground_marking_points_set.size(); i0++) {
        convert(struct_v.ground_marking_points_set[i0], ros_v.ground_marking_points_set[i0], type);
      }
    }
  }
  //
  convert(struct_v.local_ground_marking_points_set_size, ros_v.local_ground_marking_points_set_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.local_ground_marking_points_set_size >= 0 && struct_v.local_ground_marking_points_set_size <= FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      ros_v.local_ground_marking_points_set.resize(struct_v.local_ground_marking_points_set_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] local_ground_marking_points_set_size=" << struct_v.local_ground_marking_points_set_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
      ros_v.local_ground_marking_points_set_size = FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM;
      ros_v.local_ground_marking_points_set.resize(FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.local_ground_marking_points_set.size(); i1++) {
      convert(struct_v.local_ground_marking_points_set[i1], ros_v.local_ground_marking_points_set[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.local_ground_marking_points_set_size > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM || ros_v.local_ground_marking_points_set_size < 0 || ros_v.local_ground_marking_points_set.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] local_ground_marking_points_set_size=" << ros_v.local_ground_marking_points_set_size 
                << " ros_v.local_ground_marking_points_set.size()=" << ros_v.local_ground_marking_points_set.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.local_ground_marking_points_set.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      for (size_t i1 = 0; i1 < FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM; i1++) {
        convert(struct_v.local_ground_marking_points_set[i1], ros_v.local_ground_marking_points_set[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.local_ground_marking_points_set.size(); i1++) {
        convert(struct_v.local_ground_marking_points_set[i1], ros_v.local_ground_marking_points_set[i1], type);
      }
    }
  }
  //
  convert(struct_v.orientation_angle, ros_v.orientation_angle, type);
  convert(struct_v.turn_type, ros_v.turn_type, type);
}

template <typename T2>
void convert(iflyauto::FusionPolyLine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.type, ros_v.type, type);
  convert(struct_v.local_points_size, ros_v.local_points_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.local_points_size >= 0 && struct_v.local_points_size <= FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      ros_v.local_points.resize(struct_v.local_points_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] local_points_size=" << struct_v.local_points_size 
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
      ros_v.local_points_size = FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM;
      ros_v.local_points.resize(FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.local_points.size(); i0++) {
      convert(struct_v.local_points[i0], ros_v.local_points[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.local_points_size > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM || ros_v.local_points_size < 0 || ros_v.local_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] local_points_size=" << ros_v.local_points_size 
                << " ros_v.local_points.size()=" << ros_v.local_points.size()
                << " not in range FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM=" << FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM 
                << std::endl;
    }
    if (ros_v.local_points.size() > FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_ROAD_LANE_BOUNDARY_POINT_MAX_NUM; i0++) {
        convert(struct_v.local_points[i0], ros_v.local_points[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.local_points.size(); i0++) {
        convert(struct_v.local_points[i0], ros_v.local_points[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::RoadInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.isp_timestamp, ros_v.isp_timestamp, type);
  convert(struct_v.reference_line_msg_size, ros_v.reference_line_msg_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.reference_line_msg_size >= 0 && struct_v.reference_line_msg_size <= FUSION_ROAD_REFLINE_MAX_NUM) {
      ros_v.reference_line_msg.resize(struct_v.reference_line_msg_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] reference_line_msg_size=" << struct_v.reference_line_msg_size 
                << " not in range FUSION_ROAD_REFLINE_MAX_NUM=" << FUSION_ROAD_REFLINE_MAX_NUM 
                << std::endl;
      ros_v.reference_line_msg_size = FUSION_ROAD_REFLINE_MAX_NUM;
      ros_v.reference_line_msg.resize(FUSION_ROAD_REFLINE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.reference_line_msg.size(); i0++) {
      convert(struct_v.reference_line_msg[i0], ros_v.reference_line_msg[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.reference_line_msg_size > FUSION_ROAD_REFLINE_MAX_NUM || ros_v.reference_line_msg_size < 0 || ros_v.reference_line_msg.size() > FUSION_ROAD_REFLINE_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] reference_line_msg_size=" << ros_v.reference_line_msg_size 
                << " ros_v.reference_line_msg.size()=" << ros_v.reference_line_msg.size()
                << " not in range FUSION_ROAD_REFLINE_MAX_NUM=" << FUSION_ROAD_REFLINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.reference_line_msg.size() > FUSION_ROAD_REFLINE_MAX_NUM) {
      for (size_t i0 = 0; i0 < FUSION_ROAD_REFLINE_MAX_NUM; i0++) {
        convert(struct_v.reference_line_msg[i0], ros_v.reference_line_msg[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.reference_line_msg.size(); i0++) {
        convert(struct_v.reference_line_msg[i0], ros_v.reference_line_msg[i0], type);
      }
    }
  }
  //
  convert(struct_v.lane_ground_markings_size, ros_v.lane_ground_markings_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lane_ground_markings_size >= 0 && struct_v.lane_ground_markings_size <= FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM) {
      ros_v.lane_ground_markings.resize(struct_v.lane_ground_markings_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lane_ground_markings_size=" << struct_v.lane_ground_markings_size 
                << " not in range FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM=" << FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM 
                << std::endl;
      ros_v.lane_ground_markings_size = FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM;
      ros_v.lane_ground_markings.resize(FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.lane_ground_markings.size(); i1++) {
      convert(struct_v.lane_ground_markings[i1], ros_v.lane_ground_markings[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lane_ground_markings_size > FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM || ros_v.lane_ground_markings_size < 0 || ros_v.lane_ground_markings.size() > FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lane_ground_markings_size=" << ros_v.lane_ground_markings_size 
                << " ros_v.lane_ground_markings.size()=" << ros_v.lane_ground_markings.size()
                << " not in range FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM=" << FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lane_ground_markings.size() > FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM) {
      for (size_t i1 = 0; i1 < FUSION_ROAD_LANE_GROUND_MARKINGS_MAX_NUM; i1++) {
        convert(struct_v.lane_ground_markings[i1], ros_v.lane_ground_markings[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.lane_ground_markings.size(); i1++) {
        convert(struct_v.lane_ground_markings[i1], ros_v.lane_ground_markings[i1], type);
      }
    }
  }
  //
  convert(struct_v.local_point_valid, ros_v.local_point_valid, type);
  convert(struct_v.fusion_polyline_size, ros_v.fusion_polyline_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.fusion_polyline_size >= 0 && struct_v.fusion_polyline_size <= FUSION_ROAD_POLYLINE_MAX_NUM) {
      ros_v.fusion_polyline.resize(struct_v.fusion_polyline_size);
    } else {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << " [convert][TO_ROS] fusion_polyline_size=" << struct_v.fusion_polyline_size 
                << " not in range FUSION_ROAD_POLYLINE_MAX_NUM=" << FUSION_ROAD_POLYLINE_MAX_NUM 
                << std::endl;
      ros_v.fusion_polyline_size = FUSION_ROAD_POLYLINE_MAX_NUM;
      ros_v.fusion_polyline.resize(FUSION_ROAD_POLYLINE_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.fusion_polyline.size(); i2++) {
      convert(struct_v.fusion_polyline[i2], ros_v.fusion_polyline[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.fusion_polyline_size > FUSION_ROAD_POLYLINE_MAX_NUM || ros_v.fusion_polyline_size < 0 || ros_v.fusion_polyline.size() > FUSION_ROAD_POLYLINE_MAX_NUM) {
      std::cout << "convert/fusion_road_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] fusion_polyline_size=" << ros_v.fusion_polyline_size 
                << " ros_v.fusion_polyline.size()=" << ros_v.fusion_polyline.size()
                << " not in range FUSION_ROAD_POLYLINE_MAX_NUM=" << FUSION_ROAD_POLYLINE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.fusion_polyline.size() > FUSION_ROAD_POLYLINE_MAX_NUM) {
      for (size_t i2 = 0; i2 < FUSION_ROAD_POLYLINE_MAX_NUM; i2++) {
        convert(struct_v.fusion_polyline[i2], ros_v.fusion_polyline[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.fusion_polyline.size(); i2++) {
        convert(struct_v.fusion_polyline[i2], ros_v.fusion_polyline[i2], type);
      }
    }
  }
  //
}

