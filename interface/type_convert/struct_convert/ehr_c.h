#pragma once

#include "base_convert.h"
#include "c/ehr_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::PostionData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.path_id, ros_v.path_id, type);
  convert(struct_v.offset, ros_v.offset, type);
  convert(struct_v.accuracy, ros_v.accuracy, type);
  convert(struct_v.lateral_accuracy, ros_v.lateral_accuracy, type);
  convert(struct_v.longitudinal_accuracy, ros_v.longitudinal_accuracy, type);
  convert(struct_v.original_lon, ros_v.original_lon, type);
  convert(struct_v.original_lat, ros_v.original_lat, type);
  convert(struct_v.deviation, ros_v.deviation, type);
  convert(struct_v.vehicle_speed, ros_v.vehicle_speed, type);
  convert(struct_v.relative_heading, ros_v.relative_heading, type);
  convert(struct_v.probability, ros_v.probability, type);
  convert(struct_v.current_lane, ros_v.current_lane, type);
  convert(struct_v.preferred_path, ros_v.preferred_path, type);
}

template <typename T2>
void convert(iflyauto::RelativePostion &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.road_id, ros_v.road_id, type);
  convert(struct_v.lane_id, ros_v.lane_id, type);
  convert(struct_v.lane_seq, ros_v.lane_seq, type);
  convert(struct_v.dis_left, ros_v.dis_left, type);
  convert(struct_v.dis_right, ros_v.dis_right, type);
  convert(struct_v.head_left, ros_v.head_left, type);
  convert(struct_v.head_right, ros_v.head_right, type);
  convert(struct_v.confi_lane_loc, ros_v.confi_lane_loc, type);
}

template <typename T2>
void convert(iflyauto::AbsolutePostion &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.original_loc_timestamp, ros_v.original_loc_timestamp, type);
  convert(struct_v.lon, ros_v.lon, type);
  convert(struct_v.lat, ros_v.lat, type);
  convert(struct_v.heading, ros_v.heading, type);
  convert(struct_v.velocity_x, ros_v.velocity_x, type);
  convert(struct_v.velocity_y, ros_v.velocity_y, type);
  convert(struct_v.velocity_z, ros_v.velocity_z, type);
  convert(struct_v.x_acc, ros_v.x_acc, type);
  convert(struct_v.y_acc, ros_v.y_acc, type);
  convert(struct_v.z_acc, ros_v.z_acc, type);
  convert(struct_v.angular_velocity_x, ros_v.angular_velocity_x, type);
  convert(struct_v.angular_velocity_y, ros_v.angular_velocity_y, type);
  convert(struct_v.angular_velocity_z, ros_v.angular_velocity_z, type);
}

template <typename T2>
void convert(iflyauto::PositionWarning &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.warning_judge_status, ros_v.warning_judge_status, type);
  convert(struct_v.warning_judge_type, ros_v.warning_judge_type, type);
  convert(struct_v.warning_bounding_distance, ros_v.warning_bounding_distance, type);
}

template <typename T2>
void convert(iflyauto::PositionGeofence &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.geofence_judge_status, ros_v.geofence_judge_status, type);
  convert(struct_v.geofence_judge_type, ros_v.geofence_judge_type, type);
  convert(struct_v.geofence_bounding_distance, ros_v.geofence_bounding_distance, type);
}

template <typename T2>
void convert(iflyauto::PostionFailSafe &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.failsafe_loc_status, ros_v.failsafe_loc_status, type);
  convert(struct_v.failsafe_gnss_status, ros_v.failsafe_gnss_status, type);
  convert(struct_v.failsafe_camera_status, ros_v.failsafe_camera_status, type);
  convert(struct_v.failsafe_hdmap_status, ros_v.failsafe_hdmap_status, type);
  convert(struct_v.failsafe_vehicle_status, ros_v.failsafe_vehicle_status, type);
  convert(struct_v.failsafe_imu_status, ros_v.failsafe_imu_status, type);
}

template <typename T2>
void convert(iflyauto::LaneInRoute &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_id, ros_v.lane_id, type);
  convert(struct_v.entry_lane_ids_size, ros_v.entry_lane_ids_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.entry_lane_ids_size >= 0 && struct_v.entry_lane_ids_size <= EHR_LANE_GROUPS_MAX_NUM) {
      ros_v.entry_lane_ids.resize(struct_v.entry_lane_ids_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] entry_lane_ids_size=" << struct_v.entry_lane_ids_size 
                << " not in range EHR_LANE_GROUPS_MAX_NUM=" << EHR_LANE_GROUPS_MAX_NUM 
                << std::endl;
      ros_v.entry_lane_ids_size = EHR_LANE_GROUPS_MAX_NUM;
      ros_v.entry_lane_ids.resize(EHR_LANE_GROUPS_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.entry_lane_ids.size(); i0++) {
      convert(struct_v.entry_lane_ids[i0], ros_v.entry_lane_ids[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.entry_lane_ids_size > EHR_LANE_GROUPS_MAX_NUM || ros_v.entry_lane_ids_size < 0 || ros_v.entry_lane_ids.size() > EHR_LANE_GROUPS_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] entry_lane_ids_size=" << ros_v.entry_lane_ids_size 
                << " ros_v.entry_lane_ids.size()=" << ros_v.entry_lane_ids.size()
                << " not in range EHR_LANE_GROUPS_MAX_NUM=" << EHR_LANE_GROUPS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.entry_lane_ids.size() > EHR_LANE_GROUPS_MAX_NUM) {
      for (size_t i0 = 0; i0 < EHR_LANE_GROUPS_MAX_NUM; i0++) {
        convert(struct_v.entry_lane_ids[i0], ros_v.entry_lane_ids[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.entry_lane_ids.size(); i0++) {
        convert(struct_v.entry_lane_ids[i0], ros_v.entry_lane_ids[i0], type);
      }
    }
  }
  //
  convert(struct_v.exit_lane_ids_size, ros_v.exit_lane_ids_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.exit_lane_ids_size >= 0 && struct_v.exit_lane_ids_size <= EHR_LANE_GROUPS_MAX_NUM) {
      ros_v.exit_lane_ids.resize(struct_v.exit_lane_ids_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] exit_lane_ids_size=" << struct_v.exit_lane_ids_size 
                << " not in range EHR_LANE_GROUPS_MAX_NUM=" << EHR_LANE_GROUPS_MAX_NUM 
                << std::endl;
      ros_v.exit_lane_ids_size = EHR_LANE_GROUPS_MAX_NUM;
      ros_v.exit_lane_ids.resize(EHR_LANE_GROUPS_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.exit_lane_ids.size(); i1++) {
      convert(struct_v.exit_lane_ids[i1], ros_v.exit_lane_ids[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.exit_lane_ids_size > EHR_LANE_GROUPS_MAX_NUM || ros_v.exit_lane_ids_size < 0 || ros_v.exit_lane_ids.size() > EHR_LANE_GROUPS_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] exit_lane_ids_size=" << ros_v.exit_lane_ids_size 
                << " ros_v.exit_lane_ids.size()=" << ros_v.exit_lane_ids.size()
                << " not in range EHR_LANE_GROUPS_MAX_NUM=" << EHR_LANE_GROUPS_MAX_NUM 
                << std::endl;
    }
    if (ros_v.exit_lane_ids.size() > EHR_LANE_GROUPS_MAX_NUM) {
      for (size_t i1 = 0; i1 < EHR_LANE_GROUPS_MAX_NUM; i1++) {
        convert(struct_v.exit_lane_ids[i1], ros_v.exit_lane_ids[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.exit_lane_ids.size(); i1++) {
        convert(struct_v.exit_lane_ids[i1], ros_v.exit_lane_ids[i1], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::LaneGroupInRoute &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.lane_group_id, ros_v.lane_group_id, type);
  convert(struct_v.length, ros_v.length, type);
  convert(struct_v.lanes_in_route_size, ros_v.lanes_in_route_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.lanes_in_route_size >= 0 && struct_v.lanes_in_route_size <= EHR_LANE_GROUP_ID_MAX_NUM) {
      ros_v.lanes_in_route.resize(struct_v.lanes_in_route_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] lanes_in_route_size=" << struct_v.lanes_in_route_size 
                << " not in range EHR_LANE_GROUP_ID_MAX_NUM=" << EHR_LANE_GROUP_ID_MAX_NUM 
                << std::endl;
      ros_v.lanes_in_route_size = EHR_LANE_GROUP_ID_MAX_NUM;
      ros_v.lanes_in_route.resize(EHR_LANE_GROUP_ID_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.lanes_in_route.size(); i0++) {
      convert(struct_v.lanes_in_route[i0], ros_v.lanes_in_route[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.lanes_in_route_size > EHR_LANE_GROUP_ID_MAX_NUM || ros_v.lanes_in_route_size < 0 || ros_v.lanes_in_route.size() > EHR_LANE_GROUP_ID_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] lanes_in_route_size=" << ros_v.lanes_in_route_size 
                << " ros_v.lanes_in_route.size()=" << ros_v.lanes_in_route.size()
                << " not in range EHR_LANE_GROUP_ID_MAX_NUM=" << EHR_LANE_GROUP_ID_MAX_NUM 
                << std::endl;
    }
    if (ros_v.lanes_in_route.size() > EHR_LANE_GROUP_ID_MAX_NUM) {
      for (size_t i0 = 0; i0 < EHR_LANE_GROUP_ID_MAX_NUM; i0++) {
        convert(struct_v.lanes_in_route[i0], ros_v.lanes_in_route[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.lanes_in_route.size(); i0++) {
        convert(struct_v.lanes_in_route[i0], ros_v.lanes_in_route[i0], type);
      }
    }
  }
  //
  convert(struct_v.start_point, ros_v.start_point, type);
  convert(struct_v.end_point, ros_v.end_point, type);
}

template <typename T2>
void convert(iflyauto::Position &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.position_age, ros_v.position_age, type);
  convert(struct_v.positions_size, ros_v.positions_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.positions_size >= 0 && struct_v.positions_size <= EHR_POSITION_MAX_NUM) {
      ros_v.positions.resize(struct_v.positions_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] positions_size=" << struct_v.positions_size 
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
      ros_v.positions_size = EHR_POSITION_MAX_NUM;
      ros_v.positions.resize(EHR_POSITION_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.positions.size(); i0++) {
      convert(struct_v.positions[i0], ros_v.positions[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.positions_size > EHR_POSITION_MAX_NUM || ros_v.positions_size < 0 || ros_v.positions.size() > EHR_POSITION_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] positions_size=" << ros_v.positions_size 
                << " ros_v.positions.size()=" << ros_v.positions.size()
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
    }
    if (ros_v.positions.size() > EHR_POSITION_MAX_NUM) {
      for (size_t i0 = 0; i0 < EHR_POSITION_MAX_NUM; i0++) {
        convert(struct_v.positions[i0], ros_v.positions[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.positions.size(); i0++) {
        convert(struct_v.positions[i0], ros_v.positions[i0], type);
      }
    }
  }
  //
  convert(struct_v.relative_pos_size, ros_v.relative_pos_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.relative_pos_size >= 0 && struct_v.relative_pos_size <= EHR_POSITION_MAX_NUM) {
      ros_v.relative_pos.resize(struct_v.relative_pos_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] relative_pos_size=" << struct_v.relative_pos_size 
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
      ros_v.relative_pos_size = EHR_POSITION_MAX_NUM;
      ros_v.relative_pos.resize(EHR_POSITION_MAX_NUM);
    }
    for (size_t i1 = 0; i1 < ros_v.relative_pos.size(); i1++) {
      convert(struct_v.relative_pos[i1], ros_v.relative_pos[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.relative_pos_size > EHR_POSITION_MAX_NUM || ros_v.relative_pos_size < 0 || ros_v.relative_pos.size() > EHR_POSITION_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] relative_pos_size=" << ros_v.relative_pos_size 
                << " ros_v.relative_pos.size()=" << ros_v.relative_pos.size()
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
    }
    if (ros_v.relative_pos.size() > EHR_POSITION_MAX_NUM) {
      for (size_t i1 = 0; i1 < EHR_POSITION_MAX_NUM; i1++) {
        convert(struct_v.relative_pos[i1], ros_v.relative_pos[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.relative_pos.size(); i1++) {
        convert(struct_v.relative_pos[i1], ros_v.relative_pos[i1], type);
      }
    }
  }
  //
  convert(struct_v.absolute_pos_size, ros_v.absolute_pos_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.absolute_pos_size >= 0 && struct_v.absolute_pos_size <= EHR_POSITION_MAX_NUM) {
      ros_v.absolute_pos.resize(struct_v.absolute_pos_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] absolute_pos_size=" << struct_v.absolute_pos_size 
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
      ros_v.absolute_pos_size = EHR_POSITION_MAX_NUM;
      ros_v.absolute_pos.resize(EHR_POSITION_MAX_NUM);
    }
    for (size_t i2 = 0; i2 < ros_v.absolute_pos.size(); i2++) {
      convert(struct_v.absolute_pos[i2], ros_v.absolute_pos[i2], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.absolute_pos_size > EHR_POSITION_MAX_NUM || ros_v.absolute_pos_size < 0 || ros_v.absolute_pos.size() > EHR_POSITION_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] absolute_pos_size=" << ros_v.absolute_pos_size 
                << " ros_v.absolute_pos.size()=" << ros_v.absolute_pos.size()
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
    }
    if (ros_v.absolute_pos.size() > EHR_POSITION_MAX_NUM) {
      for (size_t i2 = 0; i2 < EHR_POSITION_MAX_NUM; i2++) {
        convert(struct_v.absolute_pos[i2], ros_v.absolute_pos[i2], type);
      }
    } else {
      for (size_t i2 = 0; i2 < ros_v.absolute_pos.size(); i2++) {
        convert(struct_v.absolute_pos[i2], ros_v.absolute_pos[i2], type);
      }
    }
  }
  //
  convert(struct_v.position_warning_size, ros_v.position_warning_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.position_warning_size >= 0 && struct_v.position_warning_size <= EHR_POSITION_MAX_NUM) {
      ros_v.position_warning.resize(struct_v.position_warning_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] position_warning_size=" << struct_v.position_warning_size 
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
      ros_v.position_warning_size = EHR_POSITION_MAX_NUM;
      ros_v.position_warning.resize(EHR_POSITION_MAX_NUM);
    }
    for (size_t i3 = 0; i3 < ros_v.position_warning.size(); i3++) {
      convert(struct_v.position_warning[i3], ros_v.position_warning[i3], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.position_warning_size > EHR_POSITION_MAX_NUM || ros_v.position_warning_size < 0 || ros_v.position_warning.size() > EHR_POSITION_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] position_warning_size=" << ros_v.position_warning_size 
                << " ros_v.position_warning.size()=" << ros_v.position_warning.size()
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
    }
    if (ros_v.position_warning.size() > EHR_POSITION_MAX_NUM) {
      for (size_t i3 = 0; i3 < EHR_POSITION_MAX_NUM; i3++) {
        convert(struct_v.position_warning[i3], ros_v.position_warning[i3], type);
      }
    } else {
      for (size_t i3 = 0; i3 < ros_v.position_warning.size(); i3++) {
        convert(struct_v.position_warning[i3], ros_v.position_warning[i3], type);
      }
    }
  }
  //
  convert(struct_v.position_geofence_size, ros_v.position_geofence_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.position_geofence_size >= 0 && struct_v.position_geofence_size <= EHR_POSITION_MAX_NUM) {
      ros_v.position_geofence.resize(struct_v.position_geofence_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] position_geofence_size=" << struct_v.position_geofence_size 
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
      ros_v.position_geofence_size = EHR_POSITION_MAX_NUM;
      ros_v.position_geofence.resize(EHR_POSITION_MAX_NUM);
    }
    for (size_t i4 = 0; i4 < ros_v.position_geofence.size(); i4++) {
      convert(struct_v.position_geofence[i4], ros_v.position_geofence[i4], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.position_geofence_size > EHR_POSITION_MAX_NUM || ros_v.position_geofence_size < 0 || ros_v.position_geofence.size() > EHR_POSITION_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] position_geofence_size=" << ros_v.position_geofence_size 
                << " ros_v.position_geofence.size()=" << ros_v.position_geofence.size()
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
    }
    if (ros_v.position_geofence.size() > EHR_POSITION_MAX_NUM) {
      for (size_t i4 = 0; i4 < EHR_POSITION_MAX_NUM; i4++) {
        convert(struct_v.position_geofence[i4], ros_v.position_geofence[i4], type);
      }
    } else {
      for (size_t i4 = 0; i4 < ros_v.position_geofence.size(); i4++) {
        convert(struct_v.position_geofence[i4], ros_v.position_geofence[i4], type);
      }
    }
  }
  //
  convert(struct_v.fail_safe_size, ros_v.fail_safe_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.fail_safe_size >= 0 && struct_v.fail_safe_size <= EHR_POSITION_MAX_NUM) {
      ros_v.fail_safe.resize(struct_v.fail_safe_size);
    } else {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << " [convert][TO_ROS] fail_safe_size=" << struct_v.fail_safe_size 
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
      ros_v.fail_safe_size = EHR_POSITION_MAX_NUM;
      ros_v.fail_safe.resize(EHR_POSITION_MAX_NUM);
    }
    for (size_t i5 = 0; i5 < ros_v.fail_safe.size(); i5++) {
      convert(struct_v.fail_safe[i5], ros_v.fail_safe[i5], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.fail_safe_size > EHR_POSITION_MAX_NUM || ros_v.fail_safe_size < 0 || ros_v.fail_safe.size() > EHR_POSITION_MAX_NUM) {
      std::cout << "convert/ehr_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] fail_safe_size=" << ros_v.fail_safe_size 
                << " ros_v.fail_safe.size()=" << ros_v.fail_safe.size()
                << " not in range EHR_POSITION_MAX_NUM=" << EHR_POSITION_MAX_NUM 
                << std::endl;
    }
    if (ros_v.fail_safe.size() > EHR_POSITION_MAX_NUM) {
      for (size_t i5 = 0; i5 < EHR_POSITION_MAX_NUM; i5++) {
        convert(struct_v.fail_safe[i5], ros_v.fail_safe[i5], type);
      }
    } else {
      for (size_t i5 = 0; i5 < ros_v.fail_safe.size(); i5++) {
        convert(struct_v.fail_safe[i5], ros_v.fail_safe[i5], type);
      }
    }
  }
  //
}

