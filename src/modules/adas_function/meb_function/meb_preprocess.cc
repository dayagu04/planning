#include "meb_preprocess.h"

#include <array>
#include <cmath>

using namespace planning;
namespace adas_function {

void MebPreprocess::InitOnce(void) {
  meb_input_ = {};
  meb_input_.obj_logger_.objects.clear();
  meb_input_.obj_logger_.objects.reserve(OBJECT_LOGGER_MAX_RECORDS);
}

bool MebPreprocess::CheckSpeedReduction(double speed_reduction_threshold,
                                        int32 check_length_nums) {
  double current_frame_velocity = GetHistoryFrame(0)->vehicle_speed;

  // 检查是否有足够的历史数据
  int32 length = max_int32(
      min_int32(MEB_VEHICLE_SERVICE_FRAME_SIZE, check_length_nums) - 1, 0);

  // 30kph
  if (current_frame_velocity > 4.16) {
    return FALSE;
  }
  // 如果历史数据不足，返回FALSE
  if (length < check_length_nums - 1) {
    return FALSE;
  }

  // 遍历前N帧，查找速度降低超过阈值的情况
  for (int i = length; i >= 0; i--) {
    double historical_velocity = GetHistoryFrame(0)->vehicle_speed;
    double speed_reduction = historical_velocity - current_frame_velocity;

    // 如果发现速度降低超过阈值，返回TRUE
    if (speed_reduction > speed_reduction_threshold) {
      return TRUE;
    }
  }

  // 前N帧内没有发现速度降低超过阈值的情况
  return FALSE;
}
void MebPreprocess::UpdateSingleFrameVehicleService(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto &vehicle_service = GetContext.mutable_session()
                              ->mutable_environmental_model()
                              ->get_local_view()
                              .vehicle_service_output_info;
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;

  double vel_speed_preprocess =
      std::max(GetContext.get_state_info()->vehicle_speed, 0.01);
  double shift_direction_index = 1.0;
  auto &history_head_idx_ = meb_input_.history_head_idx_;
  auto &history_full_ = meb_input_.history_full_;

  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverState_R) {
    shift_direction_index = -1.0;
    vel_speed_preprocess =
        std::min((shift_direction_index * vel_speed_preprocess), -0.01);
  }

  meb_input_.signed_ego_vel_mps = vel_speed_preprocess;
  meb_input_.shift_direction_index = shift_direction_index;

  if (meb_input_.history_full_ == false) {
    meb_input_.brake_pedal_pos_rate = 0;
  } else {
    int brake_pos_rate_index = 2;
    double brake_pedal_pos_diff =
        vehicle_service.brake_pedal_pos -
        GetHistoryFrame(brake_pos_rate_index - 1)->brake_pedal_pos;
    meb_input_.brake_pedal_pos_rate =
        brake_pedal_pos_diff / (MEB_CYCLE_TIME_SEC * brake_pos_rate_index);
  }

  if ((function_state_machine_info_ptr->current_state >=
           iflyauto::FunctionalState_PARK_STANDBY &&
       function_state_machine_info_ptr->current_state <=
           iflyauto::FunctionalState_PARK_COMPLETED) ||
      (function_state_machine_info_ptr->current_state >=
           iflyauto::FunctionalState_HPP_STANDBY &&
       function_state_machine_info_ptr->current_state <=
           iflyauto::FunctionalState_HPP_ERROR)) {
    meb_input_.park_mode = true;
  } else {
    meb_input_.park_mode = false;
  }

  // 通过 meb_input_.history_head_idx_ 获取当前帧
  auto &single_frame_vehicle_service =
      meb_input_.single_frame_vehicle_service[history_head_idx_];

  single_frame_vehicle_service.stamp = vehicle_service.sensor_meta.stamp;
  single_frame_vehicle_service.ego_acceleration =
      vehicle_service.long_acceleration;
  single_frame_vehicle_service.steering_angle =
      vehicle_service.steering_wheel_angle;
  single_frame_vehicle_service.steering_angle_speed =
      vehicle_service.steering_wheel_angle_speed;
  single_frame_vehicle_service.yaw_rate = vehicle_service.yaw_rate;
  single_frame_vehicle_service.yaw_rate_available = TRUE;
  single_frame_vehicle_service.vehicle_speed = vehicle_service.vehicle_speed;
  single_frame_vehicle_service.accelerator_pedal_pos =
      vehicle_service.accelerator_pedal_pos;
  single_frame_vehicle_service.brake_pedal_pressed =
      vehicle_service.brake_pedal_pressed;
  single_frame_vehicle_service.brake_pedal_pos =
      vehicle_service.brake_pedal_pos;

  if (history_full_ == false) {
    // 历史帧未填满时
    single_frame_vehicle_service.yaw_rate_jerk = 0.0;
    single_frame_vehicle_service.yaw_rate_jerk_available = FALSE;
    single_frame_vehicle_service.start_turning_index = -1;
    single_frame_vehicle_service.start_turning_timestamp = 0;
    single_frame_vehicle_service.latest_straight_running_timestamp = 0;
    single_frame_vehicle_service.start_straight_running_timestamp = 0;
    single_frame_vehicle_service.turning_count = 0;
    single_frame_vehicle_service.need_update_backright = FALSE;
    single_frame_vehicle_service.driver_linear_state_available = FALSE;
    single_frame_vehicle_service.driver_linear_state = FALSE;
    single_frame_vehicle_service.drive_slow_down = FALSE;
  } else {
    //历史帧填满时
    double diff_stamp =
        (GetHistoryFrame(0)->stamp - GetHistoryFrame(3)->stamp) / 1e6;
    single_frame_vehicle_service.yaw_rate_jerk =
        (GetHistoryFrame(0)->yaw_rate - GetHistoryFrame(3)->yaw_rate) /
        diff_stamp;
    single_frame_vehicle_service.yaw_rate_jerk_available = TRUE;

    single_frame_vehicle_service.start_turning_index =
        GetHistoryFrame(1)->start_turning_index;
    single_frame_vehicle_service.turning_count =
        GetHistoryFrame(1)->turning_count;
    single_frame_vehicle_service.straight_count =
        GetHistoryFrame(1)->straight_count;
    single_frame_vehicle_service.start_turning_timestamp =
        GetHistoryFrame(1)->start_turning_timestamp;
    single_frame_vehicle_service.start_straight_running_timestamp =
        GetHistoryFrame(1)->start_straight_running_timestamp;
    single_frame_vehicle_service.latest_straight_running_timestamp =
        GetHistoryFrame(1)->latest_straight_running_timestamp;
    single_frame_vehicle_service.need_update_backright =
        GetHistoryFrame(1)->need_update_backright;

    bool is_turning = false;
    bool is_straight = false;

    double cur_abs_yaw_vel = fabs(GetHistoryFrame(0)->yaw_rate);
    if (cur_abs_yaw_vel < 0.02) {
      is_straight = TRUE;
    } else if (cur_abs_yaw_vel > 0.03) {
      is_turning = TRUE;
    }

    if (single_frame_vehicle_service.start_turning_index >= 0) {
      single_frame_vehicle_service.start_turning_index =
          min_int32(single_frame_vehicle_service.start_turning_index + 1,
                    MEB_VEHICLE_SERVICE_FRAME_SIZE);
    }

    if (is_straight == TRUE) {
      single_frame_vehicle_service.turning_count = 0;
      single_frame_vehicle_service.straight_count += 1;
      if (single_frame_vehicle_service.straight_count > 5) {
        single_frame_vehicle_service.start_turning_index = -1;
        single_frame_vehicle_service.latest_straight_running_timestamp =
            single_frame_vehicle_service.stamp;

        if (single_frame_vehicle_service.need_update_backright == TRUE) {
          single_frame_vehicle_service.start_straight_running_timestamp =
              single_frame_vehicle_service.stamp;
          single_frame_vehicle_service.need_update_backright = FALSE;
        }
      }
    }

    int check_site_turn_threshold = 3;
    if (is_turning == TRUE) {
      single_frame_vehicle_service.turning_count += 1;
      single_frame_vehicle_service.straight_count = 0;

      if (single_frame_vehicle_service.turning_count >
              check_site_turn_threshold &&
          single_frame_vehicle_service.start_turning_index == -1) {
        single_frame_vehicle_service.start_turning_index =
            check_site_turn_threshold;
        single_frame_vehicle_service.start_turning_timestamp =
            single_frame_vehicle_service.stamp;
        single_frame_vehicle_service.need_update_backright = TRUE;
      }
    }

    int32 check_length = 30;
    int32 linear_ego_length_threshold = max_int32(
        min_int32(check_length, MEB_VEHICLE_SERVICE_FRAME_SIZE) - 1, 0);
    if (GetHistoryFrame(linear_ego_length_threshold)->yaw_rate_available ==
        TRUE) {
      single_frame_vehicle_service.driver_linear_state_available = TRUE;
      double turn_yaw_vel_threshold = 0.025;  // rad /s

      double linear_steer_angle_diff = 0.174;  // rad

      single_frame_vehicle_service.driver_linear_state =
          CheckDriveLinear(turn_yaw_vel_threshold, linear_steer_angle_diff,
                           linear_ego_length_threshold);
    }

    check_length = 50;
    linear_ego_length_threshold = max_int32(
        min_int32(check_length, MEB_VEHICLE_SERVICE_FRAME_SIZE) - 1, 0);
    if (GetHistoryFrame(linear_ego_length_threshold)->yaw_rate_available ==
        TRUE) {
      double ego_velocity_threshold = 2.22;
      int32 last_unpass_index_threshold = MEB_VEHICLE_SERVICE_FRAME_SIZE;
      bool slow_down_check = CheckSpeedReduction(ego_velocity_threshold,
                                                 last_unpass_index_threshold);
      single_frame_vehicle_service.drive_slow_down = slow_down_check;
    }
  }

  // 数据填充完毕，指针指向下一个
  history_head_idx_++;

  // 3. 处理循环回绕 (Ring Buffer 逻辑)
  if (history_head_idx_ >= MEB_VEHICLE_SERVICE_FRAME_SIZE) {
    history_head_idx_ = 0;  // 回到开头
    history_full_ = true;   // 标记已满
  }
}

// 实现 GetHistoryFrame
const SelfVehicleServiceFrameInfo *MebPreprocess::GetHistoryFrame(
    size_t delay_cnt) const {
  auto &history_head_idx_ = meb_input_.history_head_idx_;
  auto &history_full_ = meb_input_.history_full_;

  // 1. 检查请求是否超出 buffer 容量
  if (delay_cnt >= MEB_VEHICLE_SERVICE_FRAME_SIZE) {
    return nullptr;
  }

  // 2. 检查请求是否超出已有数据范围 (针对刚启动还没填满的情况)
  // 如果 buffer
  // 还没满，且请求的“倒数第N帧”索引超出了当前已写入的帧数，说明历史不够
  if (!history_full_ && delay_cnt >= history_head_idx_) {
    return nullptr;
  }

  // 3. 计算实际索引
  // history_head_idx_ 指向的是"未来要写的位置"，所以最新帧是
  // history_head_idx_
  // - 1 加上 SIZE 是为了防止减法结果为负数导致取模错误
  size_t target_idx =
      (history_head_idx_ - 1 - delay_cnt + MEB_VEHICLE_SERVICE_FRAME_SIZE) %
      MEB_VEHICLE_SERVICE_FRAME_SIZE;

  return &meb_input_.single_frame_vehicle_service[target_idx];
}

void MebPreprocess::UpdateObjectLogger(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto fusion_objects_info = GetContext.get_session()
                                 ->environmental_model()
                                 .get_local_view()
                                 .fusion_objects_info;
  if (meb_input_.obj_logger_.is_full == false) {
    if (meb_input_.obj_logger_.current_index >= 1 &&
        fusion_objects_info.msg_header.stamp <=
            meb_input_.obj_logger_
                .objects[meb_input_.obj_logger_.current_index - 1]
                .msg_header.stamp) {
      return;
    }

    meb_input_.obj_logger_.objects.emplace_back(fusion_objects_info);
    meb_input_.obj_logger_.current_index++;
    if (meb_input_.obj_logger_.current_index >= OBJECT_LOGGER_MAX_RECORDS) {
      meb_input_.obj_logger_.is_full = true;
      meb_input_.obj_logger_.current_index = 0;
    }
  } else {
    const size_t prev_idx =
        (meb_input_.obj_logger_.current_index + OBJECT_LOGGER_MAX_RECORDS - 1) %
        OBJECT_LOGGER_MAX_RECORDS;
    if (fusion_objects_info.msg_header.stamp <=
        meb_input_.obj_logger_.objects[prev_idx].msg_header.stamp) {
      return;
    }
    meb_input_.obj_logger_.objects[meb_input_.obj_logger_.current_index] =
        fusion_objects_info;
    meb_input_.obj_logger_.current_index =
        (meb_input_.obj_logger_.current_index + 1) % OBJECT_LOGGER_MAX_RECORDS;
  }
}

const iflyauto::FusionObjectsInfo *MebPreprocess::GetHistoryObjectLogger(
    size_t delay_cnt) const {
  const auto &obj_logger = meb_input_.obj_logger_;

  if (delay_cnt >= OBJECT_LOGGER_MAX_RECORDS) {
    return nullptr;
  }

  if (!obj_logger.is_full && delay_cnt >= obj_logger.current_index) {
    return nullptr;
  }

  size_t target_idx =
      (obj_logger.current_index - 1 - delay_cnt + OBJECT_LOGGER_MAX_RECORDS) %
      OBJECT_LOGGER_MAX_RECORDS;

  return &obj_logger.objects[target_idx];
}

bool MebPreprocess::CheckDriveLinear(double turn_yaw_vel_threshold,
                                     double linear_steer_angle_diff,
                                     int linear_ego_length_threshold) {
  // turn_yaw_vel_threshold  rad
  // linear_steer_angle_diff rad

  if (GetHistoryFrame(0)->yaw_rate_available == TRUE &&
      fabs(GetHistoryFrame(0)->yaw_rate) > turn_yaw_vel_threshold) {
    return FALSE;
  }

  int32 length = max_int32(
      min_int32(linear_ego_length_threshold, MEB_VEHICLE_SERVICE_FRAME_SIZE) -
          1,
      0);
  // const float eps = 1e-7;
  float min_raw = pnc::mathlib::Deg2Rad(942.0);   // deg
  float max_raw = pnc::mathlib::Deg2Rad(-942.0);  // deg

  int32 yaw_vel_unpass_count = 0;
  int32 yaw_vel_last_unpass_index = 20;
  int32 unpass_threshold = 3;
  int32 before_frames = -1;
  float yaw_vel_diff = 0.025;
  // printf("****check_drive_linear Stamp: %f, diff_speed:\n",

  for (int i = 0; i < length; i++) {
    float cur_steering_angle = GetHistoryFrame(i)->steering_angle;
    float cur_yaw_rate = GetHistoryFrame(i)->yaw_rate;
    // printf("\t%.4f:%.4f", cur_steering_angle, cur_yaw_rate);

    /* check current yaw */
    max_raw = max_float(cur_steering_angle, max_raw);
    min_raw = min_float(cur_steering_angle, min_raw);

    /* check every yaw rate */
    if (fabs(cur_yaw_rate) > yaw_vel_diff) {
      yaw_vel_unpass_count += 1;
      yaw_vel_last_unpass_index = min_int32(i, yaw_vel_last_unpass_index);
    }

    if (yaw_vel_unpass_count >= unpass_threshold ||
        yaw_vel_last_unpass_index < before_frames) {
      // printf("\nyaw_vel_unpass_count FALSE\n");
      return FALSE;
    }
  }

  float diff_angle = fabs(max_raw - min_raw);
  // printf("\nMax diff_angle:%.4f\n", diff_angle);
  if (diff_angle > linear_steer_angle_diff) {
    // printf("\ndiff_angle FALSE\n");
    return FALSE;
  }

  return TRUE;
};

double MebPreprocess::FindClosestVehicleSpeed(uint64 target_timestamp,
                                              uint16 max_search_frames) {
  if (!meb_input_.history_full_ || target_timestamp == 0) {
    return 0.0;
  }

  uint64 min_time_diff = 1000000;
  double closest_vehicle_speed = 0.0f;
  bool found = FALSE;

  // 遍历自车历史帧，找到时间戳最接近的一帧
  for (uint16 i = 0; i < max_search_frames; i++) {
    if (GetHistoryFrame(i)->stamp == 0) {
      continue;  // 跳过无效帧
    }

    // 计算时间差（微秒）
    uint64 time_diff;
    if (target_timestamp >= GetHistoryFrame(i)->stamp) {
      time_diff = target_timestamp - GetHistoryFrame(i)->stamp;
    } else {
      time_diff = GetHistoryFrame(i)->stamp - target_timestamp;
    }

    // 如果时间差更小，更新最近帧
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      closest_vehicle_speed = GetHistoryFrame(i)->vehicle_speed;
      found = TRUE;
    }
  }

  return found ? closest_vehicle_speed : 0.0;
}

/**
 * @brief 检查指定障碍物历史帧中的横向和纵向速度是否稳定（基于趋势检测）
 * @param track_id 目标障碍物的track_id
 * @param check_frames 检查的历史帧数
 * @param velocity_x_threshold
 * 纵向速度变化阈值（米/秒），相邻帧速度变化超过该值才计入趋势判断
 * @param velocity_y_threshold
 * 横向速度变化阈值（米/秒），相邻帧速度变化超过该值才计入趋势判断
 * @return TRUE:速度稳定（无明显持续增大或减小趋势）,
 * FALSE:速度不稳定（有持续变化趋势）或数据不足
 */
bool MebPreprocess::ObjectLoggerCheckVelocityStability(
    int track_id, int check_frames, double velocity_x_threshold,
    double velocity_y_threshold, double speed_angle_thres) {
  if (!meb_input_.obj_logger_.is_full || check_frames <= 0 ||
      !meb_input_.history_full_) {
    return false;
  }

  struct Velocity2D {
    float vx;
    float vy;
  };
  std::array<Velocity2D, OBJECT_LOGGER_MAX_RECORDS> vel_history{};
  int found_count = 0;

  const int max_frames =
      std::min(static_cast<int>(OBJECT_LOGGER_MAX_RECORDS), check_frames);

  for (int frame_idx = 0; frame_idx < max_frames; frame_idx++) {
    const auto *frame = GetHistoryObjectLogger(frame_idx);
    if (frame == nullptr) {
      continue;
    }

    const float ego_speed = FindClosestVehicleSpeed(
        frame->msg_header.stamp, MEB_VEHICLE_SERVICE_FRAME_SIZE);

    for (int obj_idx = 0; obj_idx < frame->fusion_object_size; obj_idx++) {
      if (frame->fusion_object[obj_idx].additional_info.track_id == track_id) {
        const auto &obj = frame->fusion_object[obj_idx];
        vel_history[found_count] = {
            obj.common_info.relative_velocity.x + ego_speed,
            obj.common_info.relative_velocity.y};
        found_count++;
        break;
      }
    }
  }

  if (found_count < check_frames) {
    return false;
  }

  constexpr float kHorizontalAngle = 1.57f;    // π/2
  constexpr float kMinCrossingSpeed = 0.833f;  // 3 kph
  constexpr float kEps = 1e-3f;
  const float angle_diff_thres = static_cast<float>(speed_angle_thres);

  for (int i = 0; i < found_count; i++) {
    const auto &v = vel_history[i];

    const float speed = std::hypot(v.vx, v.vy);
    if (speed < kMinCrossingSpeed) {
      return false;
    }

    const float direction = std::fabs(std::atan2(v.vy + kEps, v.vx + kEps));
    if (std::fabs(direction - kHorizontalAngle) > angle_diff_thres) {
      return false;
    }
  }

  return true;
}

uint8 MebPreprocess::GetRearKeyObjSelectIntersetCode(
    iflyauto::FusionObject &obj) {
  uint8 interest_code = 0;
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto fusion_objects_info = GetContext.get_session()
                                 ->environmental_model()
                                 .get_local_view()
                                 .fusion_objects_info;
  /*bit_0*/
  if ((obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_COUPE) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_MINIBUS) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_VAN) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_BUS) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_TRUCK) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_TRAILER) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_BICYCLE) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE) &&
      (obj.common_info.type != iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE) &&
      (obj.common_info.type !=
       iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING) &&
      (obj.common_info.type !=
       iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE_RIDING) &&
      (obj.common_info.type !=
       iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING)) {
    interest_code += uint16_bit[0];
    return interest_code;
  } else {
    // nothing
  }

  /*bit_1*/
  if ((obj.common_info.relative_center_position.x +
       obj.common_info.shape.length / 2.0F) >
      GetContext.get_param()->origin_2_rear_bumper) {
    interest_code += uint16_bit[1];
    return interest_code;
  } else {
    // nothing
  }

  /*bit_2*/
  double ego_curvature = meb_input_.ego_curvature;
  double left_boundary_c0 = 1.8;
  double left_boundary_c1 = 0.0;
  double left_boundary_c2 = 0.5 * ego_curvature;
  double left_boundary_c3 = 0.0;
  double right_boundary_c0 = -1.8;
  double right_boundary_c1 = 0.0;
  double right_boundary_c2 = 0.5 * ego_curvature;
  double right_boundary_c3 = 0.0;
  double left_line_y =
      left_boundary_c0 +
      left_boundary_c1 * obj.common_info.relative_center_position.x +
      left_boundary_c2 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x +
      left_boundary_c3 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x;

  double right_line_y =
      right_boundary_c0 +
      right_boundary_c1 * obj.common_info.relative_center_position.x +
      right_boundary_c2 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x +
      right_boundary_c3 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x;
  if ((obj.common_info.relative_center_position.y > left_line_y) ||
      (obj.common_info.relative_center_position.y < right_line_y)) {
    interest_code += uint16_bit[2];
    return interest_code;
  } else {
    // nothing
  }

  return interest_code;
}

void MebPreprocess::UpdateRearKeyObjInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto fusion_objects_info = GetContext.get_session()
                                 ->environmental_model()
                                 .get_local_view()
                                 .fusion_objects_info;

  auto &meb_pre = adas_function::MebPreprocess::GetInstance();

  double vel_speed_preprocess =
      meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;

  rear_key_obj_info_ = {};

  // 目标筛选
  uint8 cipv_1nd_index = 255;
  uint8 cipv_2nd_index = 255;
  uint8 interest_code;
  for (uint8 i = 0; i < fusion_objects_info.fusion_object_size; i++) {
    interest_code =
        GetRearKeyObjSelectIntersetCode(fusion_objects_info.fusion_object[i]);

    if (interest_code == 0) {
      if (cipv_1nd_index >= FUSION_OBJECT_MAX_NUM) {
        cipv_1nd_index = i;
      } else {
        if (fusion_objects_info.fusion_object[i]
                .common_info.relative_center_position.x >
            fusion_objects_info.fusion_object[cipv_1nd_index]
                .common_info.relative_center_position.x) {
          cipv_2nd_index = cipv_1nd_index;
          cipv_1nd_index = i;
        } else if ((cipv_2nd_index < FUSION_OBJECT_MAX_NUM) &&
                   (fusion_objects_info.fusion_object[i]
                        .common_info.relative_center_position.x >
                    fusion_objects_info.fusion_object[cipv_2nd_index]
                        .common_info.relative_center_position.x)) {
          cipv_2nd_index = i;
        } else if (cipv_2nd_index >= FUSION_OBJECT_MAX_NUM) {
          cipv_2nd_index = i;
        } else {
        }
      }
    }
  }

  rear_key_obj_info_.key_obj_index = cipv_1nd_index;

  if (rear_key_obj_info_.key_obj_index >= FUSION_OBJECT_MAX_NUM) {
    return;
  } else {
    rear_key_obj_info_.key_obj_id =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .additional_info.track_id;
    rear_key_obj_info_.key_obj_relative_x =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_center_position.x;
    rear_key_obj_info_.key_obj_relative_y =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_center_position.y;
    rear_key_obj_info_.key_obj_relative_heading_angle =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_heading_angle;
    rear_key_obj_info_.key_obj_relative_v_x =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.x;
    rear_key_obj_info_.key_obj_relative_v_y =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.y;
    rear_key_obj_info_.key_obj_relative_lon_acc =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_acceleration.x;
    rear_key_obj_info_.key_obj_relative_lat_acc =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_acceleration.y;
    rear_key_obj_info_.key_obj_length =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.shape.length;
    rear_key_obj_info_.key_obj_width =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.shape.width;
    rear_key_obj_info_.obj_absolute_v_x =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.x +
        vel_speed_preprocess;
    rear_key_obj_info_.obj_absolute_v_y =
        fusion_objects_info.fusion_object[rear_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.y;
  }
}

uint8 MebPreprocess::GetFrontRadarKeyObjSelectIntersetCode(
    iflyauto::FusionObject &obj) {
  uint8 interest_code = 0;

  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  /*bit_0*/
  if (obj.additional_info.fusion_source != 2) {
    interest_code += uint16_bit[0];
    return interest_code;
  } else {
    // nothing
  }

  /*bit_1*/
  if ((obj.common_info.relative_center_position.x > 200.0) ||
      (obj.common_info.relative_center_position.x <
       (obj.common_info.shape.length / 2.0F +
        GetContext.get_param()->origin_2_front_bumper + 0.0F))) {
    interest_code += uint16_bit[1];
    return interest_code;
  } else {
    // nothing
  }

  /*bit_2*/
  double ego_curvature = meb_input_.ego_curvature;
  double left_boundary_c0 = 1.8;
  double left_boundary_c1 = 0.0;
  double left_boundary_c2 = 0.5 * ego_curvature;
  double left_boundary_c3 = 0.0;
  double right_boundary_c0 = -1.8;
  double right_boundary_c1 = 0.0;
  double right_boundary_c2 = 0.5 * ego_curvature;
  double right_boundary_c3 = 0.0;
  double left_line_y =
      left_boundary_c0 +
      left_boundary_c1 * obj.common_info.relative_center_position.x +
      left_boundary_c2 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x +
      left_boundary_c3 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x;

  double right_line_y =
      right_boundary_c0 +
      right_boundary_c1 * obj.common_info.relative_center_position.x +
      right_boundary_c2 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x +
      right_boundary_c3 * obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x *
          obj.common_info.relative_center_position.x;
  if ((obj.common_info.relative_center_position.y > left_line_y) ||
      (obj.common_info.relative_center_position.y < right_line_y)) {
    interest_code += uint16_bit[2];
    return interest_code;
  } else {
    // nothing
  }

  return interest_code;
}

void MebPreprocess::UpdateFrontRadarKeyObjInfo(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto fusion_objects_info = GetContext.get_session()
                                 ->environmental_model()
                                 .get_local_view()
                                 .fusion_objects_info;

  auto &meb_pre = adas_function::MebPreprocess::GetInstance();

  double vel_speed_preprocess =
      meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;

  front_radar_key_obj_info_ = {};

  // 目标筛选
  uint8 cipv_1nd_index = 255;
  uint8 cipv_2nd_index = 255;
  uint8 interest_code;
  for (uint8 i = 0; i < fusion_objects_info.fusion_object_size; i++) {
    interest_code = GetFrontRadarKeyObjSelectIntersetCode(
        fusion_objects_info.fusion_object[i]);

    if (interest_code == 0) {
      if (cipv_1nd_index >= FUSION_OBJECT_MAX_NUM) {
        cipv_1nd_index = i;
      } else {
        if (fusion_objects_info.fusion_object[i]
                .common_info.relative_center_position.x <
            fusion_objects_info.fusion_object[cipv_1nd_index]
                .common_info.relative_center_position.x) {
          cipv_2nd_index = cipv_1nd_index;
          cipv_1nd_index = i;
        } else if ((cipv_2nd_index < FUSION_OBJECT_MAX_NUM) &&
                   (fusion_objects_info.fusion_object[i]
                        .common_info.relative_center_position.x <
                    fusion_objects_info.fusion_object[cipv_2nd_index]
                        .common_info.relative_center_position.x)) {
          cipv_2nd_index = i;
        } else if (cipv_2nd_index >= FUSION_OBJECT_MAX_NUM) {
          cipv_2nd_index = i;
        } else {
        }
      }
    }
  }

  front_radar_key_obj_info_.key_obj_index = cipv_1nd_index;

  if (front_radar_key_obj_info_.key_obj_index >= FUSION_OBJECT_MAX_NUM) {
    return;
  } else {
    front_radar_key_obj_info_.key_obj_id =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .additional_info.track_id;
    front_radar_key_obj_info_.key_obj_relative_x =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_center_position.x;
    front_radar_key_obj_info_.key_obj_relative_y =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_center_position.y;
    front_radar_key_obj_info_.key_obj_relative_heading_angle =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_heading_angle;
    front_radar_key_obj_info_.key_obj_relative_v_x =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.x;
    front_radar_key_obj_info_.key_obj_relative_v_y =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.y;
    front_radar_key_obj_info_.key_obj_relative_lon_acc =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_acceleration.x;
    front_radar_key_obj_info_.key_obj_relative_lat_acc =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_acceleration.y;
    front_radar_key_obj_info_.key_obj_length =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.shape.length;
    front_radar_key_obj_info_.key_obj_width =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.shape.width;
    front_radar_key_obj_info_.obj_absolute_v_x =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.x +
        vel_speed_preprocess;
    front_radar_key_obj_info_.obj_absolute_v_y =
        fusion_objects_info
            .fusion_object[front_radar_key_obj_info_.key_obj_index]
            .common_info.relative_velocity.y;
  }
}

void MebPreprocess::Log(void) {
  JSON_DEBUG_VALUE("meb_main_switch", meb_input_.meb_main_switch);
  JSON_DEBUG_VALUE("meb_input_.ego_radius", meb_input_.ego_radius);
  JSON_DEBUG_VALUE("meb_input_.signed_ego_vel_mps",
                   meb_input_.signed_ego_vel_mps);
  JSON_DEBUG_VALUE("meb_input_.shift_direction_index",
                   meb_input_.shift_direction_index);
}

void MebPreprocess::UpdateMebInput(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  auto function_state_machine_info_ptr = GetContext.mutable_session()
                                             ->mutable_environmental_model()
                                             ->get_local_view()
                                             .function_state_machine_info;

  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  float vehicle_speed = vehicle_service_output_info_ptr->vehicle_speed;
  float yaw_rate = vehicle_service_output_info_ptr->yaw_rate;
  float steering_wheel_angle =
      vehicle_service_output_info_ptr->steering_wheel_angle;
  float ego_steer_ratio = GetContext.get_param()->steer_ratio;
  float ego_wheel_base = GetContext.get_param()->wheel_base;

  // ego_radius
  double ego_curvature =
      GetEgoCurvature(vehicle_speed, yaw_rate, steering_wheel_angle,
                      ego_steer_ratio, ego_wheel_base);

  meb_input_.ego_curvature = ego_curvature;
  if (fabs(ego_curvature) < 0.00001F) {
    meb_input_.ego_radius = 100000.0F;
  } else {
    meb_input_.ego_radius = 1.0F / ego_curvature;
  }

// meb_main_switch
#if defined(ADAS_IN_SIMULATION)
  meb_input_.meb_main_switch = true;
#else
  if (GetContext.get_param()->meb_main_switch) {
    meb_input_.meb_main_switch = GetContext.get_param()->meb_main_switch;
  } else {
    meb_input_.meb_main_switch =
        function_state_machine_info_ptr.switch_sts.meb_main_switch;
  }
#endif

  // esp_active_time
  if (vehicle_service_output_info_ptr->esp_active) {
    meb_input_.esp_active_time += MEB_CYCLE_TIME_SEC;
  } else {
    meb_input_.esp_active_time = 0.0;
  }

  // 记录多帧底盘信息
  UpdateSingleFrameVehicleService();

  // 记录多帧障碍物信息
  UpdateObjectLogger();

  // 记录后向关键障碍物信息
  UpdateRearKeyObjInfo();

  // 记录前向关键雷达障碍物信息
  UpdateFrontRadarKeyObjInfo();

  Log();
}

}  // namespace adas_function