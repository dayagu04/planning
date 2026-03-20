#include "meb_core.h"

#include <algorithm>
#include <iostream>

#include "adas_function_lib.h"
#include "geometry_math.h"
#include "polygon_base.h"

namespace {
enum class OdObjGroup { kPeople, kCar, kMotor, kDefault };

inline OdObjGroup GetOdObjGroup(const iflyauto::ObjectType type) {
  switch (type) {
    case iflyauto::ObjectType::OBJECT_TYPE_ADULT:
    case iflyauto::ObjectType::OBJECT_TYPE_CHILD:
    case iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN:
    case iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_POLICE:
      return OdObjGroup::kPeople;
    case iflyauto::ObjectType::OBJECT_TYPE_COUPE:
    case iflyauto::ObjectType::OBJECT_TYPE_MINIBUS:
    case iflyauto::ObjectType::OBJECT_TYPE_VAN:
    case iflyauto::ObjectType::OBJECT_TYPE_BUS:
    case iflyauto::ObjectType::OBJECT_TYPE_TRUCK:
    case iflyauto::ObjectType::OBJECT_TYPE_TRAILER:
      return OdObjGroup::kCar;
    case iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING:
    case iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE_RIDING:
    case iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING:
    case iflyauto::ObjectType::OBJECT_TYPE_BICYCLE:
    case iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE:
    case iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE:
      return OdObjGroup::kMotor;
    default:
      return OdObjGroup::kDefault;
  }
}

inline const std::vector<double> &GetOdBufferTable(
    const adas_function::context::Parameters &param, const OdObjGroup group,
    const bool is_reverse, const bool is_static) {
  if (is_reverse) {
    if (is_static) {
      switch (group) {
        case OdObjGroup::kPeople:
          return param.meb_odbox_dis_buffer_r_static_people;
        case OdObjGroup::kCar:
          return param.meb_odbox_dis_buffer_r_static_car;
        case OdObjGroup::kMotor:
          return param.meb_odbox_dis_buffer_r_static_motor;
        default:
          return param.meb_odbox_dis_buffer_r_static_default;
      }
    }
    switch (group) {
      case OdObjGroup::kPeople:
        return param.meb_odbox_dis_buffer_r_dynamic_people;
      case OdObjGroup::kCar:
        return param.meb_odbox_dis_buffer_r_dynamic_car;
      case OdObjGroup::kMotor:
        return param.meb_odbox_dis_buffer_r_dynamic_motor;
      default:
        return param.meb_odbox_dis_buffer_r_dynamic_default;
    }
  }
  if (is_static) {
    switch (group) {
      case OdObjGroup::kPeople:
        return param.meb_odbox_dis_buffer_d_static_people;
      case OdObjGroup::kCar:
        return param.meb_odbox_dis_buffer_d_static_car;
      case OdObjGroup::kMotor:
        return param.meb_odbox_dis_buffer_d_static_motor;
      default:
        return param.meb_odbox_dis_buffer_d_static_default;
    }
  }
  switch (group) {
    case OdObjGroup::kPeople:
      return param.meb_odbox_dis_buffer_d_dynamic_people;
    case OdObjGroup::kCar:
      return param.meb_odbox_dis_buffer_d_dynamic_car;
    case OdObjGroup::kMotor:
      return param.meb_odbox_dis_buffer_d_dynamic_motor;
    default:
      return param.meb_odbox_dis_buffer_d_dynamic_default;
  }
}
}  // namespace

namespace adas_function {
namespace meb_core {

void MebCore::Init(void) {
  // const ApaParameters &param = apa_param.GetParam();
  Eigen::Vector2d vertex;

  double max_car_width;
  std::vector<double> car_vertex_x_vec, car_vertex_y_vec;

  max_car_width = 2.212;
  // 带后视镜
  //  car_vertex_x_vec = {3.259,  3.490,  3.724, 3.724,  3.490,  3.259,  2.121,
  //                      2.121,  1.916,  1.916, -0.530, -0.862, -1.030, -1.030,
  //                      -0.862, -0.530, 1.916, 1.916,  2.121,  2.121};
  //  car_vertex_y_vec = {0.934,  0.844,  0.438,  -0.438, -0.844, -0.934,
  //  -0.934,
  //                      -1.106, -1.106, -0.934, -0.934, -0.761, -0.398, 0.398,
  //                      0.761,  0.934,  0.934,  1.106,  1.106,  0.934};
  // 不带后视镜
  // car_vertex_x_vec = {3.417,  3.730,  3.790, 3.790,  3.730,  3.417,  2.217,
  //                     2.217,  2.030,  2.030, -0.625, -0.996, -1.097, -1.097,
  //                     -0.996, -0.625, 2.030, 2.030,  2.217,  2.217};
  // car_vertex_y_vec = {0.958,  0.628,  0.345,  -0.345, -0.628, -0.958, -0.958,
  //                     -0.968, -0.968, -0.958, -0.958, -0.742, -0.361, 0.361,
  //                     0.742,  0.958,  0.958,  0.968,  0.968,  0.958};

  // 修正后车身轮廓
  car_vertex_x_vec = {3.417,  3.730,  3.790,  3.790,  3.730,  3.417,
                      -0.625, -0.996, -1.097, -1.097, -0.996, -0.625};
  car_vertex_y_vec = {0.958,  0.628,  0.345,  -0.345, -0.628, -0.958,
                      -0.958, -0.742, -0.361, 0.361,  0.742,  0.958};
  // 只有前后保险杠
  car_with_mirror_polygon_vertex_.clear();
  car_with_mirror_polygon_vertex_.reserve(car_vertex_x_vec.size());
  for (size_t i = 0; i < car_vertex_x_vec.size(); ++i) {
    vertex << car_vertex_x_vec[i], car_vertex_y_vec[i];
    car_with_mirror_polygon_vertex_.emplace_back(vertex);
  }
  car_line_local_vec_.clear();
  car_line_local_vec_.reserve(car_with_mirror_polygon_vertex_.size() - 2);
  pnc::geometry_lib::LineSegment car_line;
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;
  for (size_t i = 0; i < car_with_mirror_polygon_vertex_.size(); ++i) {
    if (i == 5 || i == 11) {
      continue;
    }
    p1[0] = car_with_mirror_polygon_vertex_[i][0];
    p1[1] = car_with_mirror_polygon_vertex_[i][1];
    if (i < car_with_mirror_polygon_vertex_.size() - 1) {
      p2[0] = car_with_mirror_polygon_vertex_[i + 1][0];
      p2[1] = car_with_mirror_polygon_vertex_[i + 1][1];
      // p2 << car_with_mirror_polygon_vertex_[i + 1],
      //     car_with_mirror_polygon_vertex_[i + 1];
    } else {
      p2[0] = car_with_mirror_polygon_vertex_[0][0];
      p2[1] = car_with_mirror_polygon_vertex_[0][1];
      // p2 << car_with_mirror_polygon_vertex_[0],
      //     car_with_mirror_polygon_vertex_[0];
    }
    car_line.SetPoints(p1, p2);
    car_line_local_vec_.emplace_back(car_line);

    // car_local_vertex_vec_.emplace_back(p1);
  }

  return;
}

bool MebCore::UpdateMebMainSwitch(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;
  if (GetContext.get_param()->meb_main_switch) {
    return GetContext.get_param()->meb_main_switch;
  }
  return function_state_machine_info_ptr->switch_sts.meb_main_switch;
}

uint32 MebCore::UpdateMebEnableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  uint32 enable_code = 0;
  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <
      meb_param_.enable_vehspd_display_min) {
    enable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >
             meb_param_.enable_vehspd_display_max) {
    enable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断挡位是否处于D档
  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_P) {
    enable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 判断四门两盖,只要有一个门/一个舱盖打开，enable条件置1
  if ((vehicle_service_output_info_ptr->fl_door_state == true) ||
      (vehicle_service_output_info_ptr->fr_door_state == true) ||
      (vehicle_service_output_info_ptr->rl_door_state == true) ||
      (vehicle_service_output_info_ptr->rr_door_state == true) ||
      (vehicle_service_output_info_ptr->hood_state == true) ||
      (vehicle_service_output_info_ptr->trunk_door_state == true) ||
      (vehicle_service_output_info_ptr->fl_door_state_available == false) ||
      (vehicle_service_output_info_ptr->fr_door_state_available == false) ||
      (vehicle_service_output_info_ptr->rl_door_state_available == false) ||
      (vehicle_service_output_info_ptr->rr_door_state_available == false) ||
      (vehicle_service_output_info_ptr->hood_state_available == false) ||
      (vehicle_service_output_info_ptr->trunk_door_state_available == false)) {
    enable_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  if (vehicle_service_output_info_ptr->fl_seat_belt_state == false ||
      vehicle_service_output_info_ptr->fl_seat_belt_state_available == false) {
    enable_code += uint16_bit[3];
  } else {
  }

  if (vehicle_service_output_info_ptr->rearview_mirror_sts !=
      iflyauto::RearviewMirrorSts ::Rearview_Mirror_Unfold) {
    enable_code += uint16_bit[4];
  } else {
  }

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;
  if ((function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_PARK_GUIDANCE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_PARKING_IN) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_PARKING_OUT) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_RADS_TRACING) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_CRUISE_ROUTING) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_SCC_ACTIVATE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_ACC_ACTIVATE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_NOA_ACTIVATE)) {
    enable_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  return enable_code;
}

uint32 MebCore::UpdateMebDisableCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  uint32 disable_code = 0;
  // bit 0
  // 判断车速是否处于工作车速范围内
  if (vehicle_service_output_info_ptr->vehicle_speed_display <=
      meb_param_.disable_vehspd_display_min) {
    disable_code += uint16_bit[0];
  } else if (vehicle_service_output_info_ptr->vehicle_speed_display >=
             meb_param_.disable_vehspd_display_max) {
    disable_code += uint16_bit[0];
  } else {
    /*do nothing*/
  }

  // bit 1
  // 判断挡位是否处于D档
  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_P) {
    disable_code += uint16_bit[1];
  } else {
    /*do nothing*/
  }

  // bit 4
  // 判断四门两盖,只要有一个门/一个舱盖打开，enable条件置1
  if ((vehicle_service_output_info_ptr->fl_door_state == true) ||
      (vehicle_service_output_info_ptr->fr_door_state == true) ||
      (vehicle_service_output_info_ptr->rl_door_state == true) ||
      (vehicle_service_output_info_ptr->rr_door_state == true) ||
      (vehicle_service_output_info_ptr->hood_state == true) ||
      (vehicle_service_output_info_ptr->trunk_door_state == true) ||
      (vehicle_service_output_info_ptr->fl_door_state_available == false) ||
      (vehicle_service_output_info_ptr->fr_door_state_available == false) ||
      (vehicle_service_output_info_ptr->rl_door_state_available == false) ||
      (vehicle_service_output_info_ptr->rr_door_state_available == false) ||
      (vehicle_service_output_info_ptr->hood_state_available == false) ||
      (vehicle_service_output_info_ptr->trunk_door_state_available == false)) {
    disable_code += uint16_bit[2];
  } else {
    /*do nothing*/
  }

  if (vehicle_service_output_info_ptr->fl_seat_belt_state == false ||
      vehicle_service_output_info_ptr->fl_seat_belt_state_available == false) {
    disable_code += uint16_bit[3];
  } else {
  }

  if (vehicle_service_output_info_ptr->rearview_mirror_sts !=
      iflyauto::RearviewMirrorSts ::Rearview_Mirror_Unfold) {
    disable_code += uint16_bit[4];
  } else {
  }

  auto function_state_machine_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .function_state_machine_info;
  if ((function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_PARK_GUIDANCE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_PARKING_IN) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_PARKING_OUT) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_RADS_TRACING) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_HPP_CRUISE_ROUTING) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_SCC_ACTIVATE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_ACC_ACTIVATE) ||
      (function_state_machine_info_ptr->current_state ==
       iflyauto::FunctionalState_NOA_ACTIVATE)) {
    disable_code += uint16_bit[5];
  } else {
    /*do nothing*/
  }

  return disable_code;
}

uint32 MebCore::UpdateMebFaultCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  uint32 fault_code = 0;

  return fault_code;
}

bool MebCore::UpdateIntervention() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  bool meb_intervention_flag = false;
  UpdateObstacles();
  CollisionCalculate();
  if (occ_meb_result_.collision_flag || od_meb_result_.collision_flag ||
      uss_meb_result_.collision_flag || od_box_collision_flag_) {
    meb_intervention_flag = true;
  }

  return meb_intervention_flag;
}

uint32 MebCore::UpdateMebKickdownCode(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  uint32 kickdown_code = 0;
  if ((vehicle_service_output_info_ptr->veh_standstill_available == true &&
       vehicle_service_output_info_ptr->veh_standstill == false) ||
      (vehicle_service_output_info_ptr->esp_pressure_available == true &&
       vehicle_service_output_info_ptr->esp_pressure < 1.0)) {
    return kickdown_code;
  }

  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_P) {
    kickdown_code += uint16_bit[1];
  }

  if (vehicle_service_output_info_ptr->epb_state_available == true &&
      vehicle_service_output_info_ptr->epb_state == 3) {
    kickdown_code += uint16_bit[2];
  }

  if (vehicle_service_output_info_ptr->brake_pedal_pressed_available == true &&
      vehicle_service_output_info_ptr->brake_pedal_pressed == true &&
      vehicle_service_output_info_ptr->brake_pedal_pos > 10.0 &&
      vehicle_service_output_info_ptr->brake_pedal_pos_available == true) {
    kickdown_code += uint16_bit[3];
  }

  if (vehicle_service_output_info_ptr->accelerator_pedal_pos_available ==
          true &&
      vehicle_service_output_info_ptr->accelerator_pedal_pos > 10.0) {
    kickdown_code += uint16_bit[4];
  }

  return kickdown_code;
}

MebInnerState MebCore::MebStateMachine(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  MebInnerState meb_state;
  if (meb_state_machine_init_flag_ == false) {
    // 状态机处于初始化状态 根据开关状态,决定第一个周期是输出OFF还是STANDBY
    meb_state_machine_init_flag_ = true;
    if (meb_main_switch_ == false) {
      meb_state = MebInnerState::MEB_STATE_OFF;
    } else {
      meb_state = MebInnerState::MEB_STATE_PASSIVE;
    }
    return meb_state;
  }

  // 状态机处于完成过初始化的状态
  if (meb_main_switch_ == false) {
    meb_state = MebInnerState::MEB_STATE_OFF;
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_FAULT) {
    if (meb_fault_code_ == 0) {
      meb_state = MebInnerState::MEB_STATE_PASSIVE;
    } else {
      // do nothing
    }
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_OFF) {
    if (meb_main_switch_ == true) {
      meb_state = MebInnerState::MEB_STATE_PASSIVE;
    } else {
      meb_state = MebInnerState::MEB_STATE_OFF;
    }
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_PASSIVE) {
    if (meb_fault_code_) {
      meb_state = MebInnerState::MEB_STATE_FAULT;
    } else if (meb_enable_code_ == 0) {
      meb_state = MebInnerState::MEB_STATE_STANDBY;
    } else {
      meb_state = meb_inner_state_;
    }
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_STANDBY) {
    if (meb_fault_code_) {
      meb_state = MebInnerState::MEB_STATE_FAULT;
    } else if (meb_intervention_flag_ == true) {
      meb_state = MebInnerState::MEB_STATE_ACTIVE;
    } else if (meb_disable_code_ != 0) {
      meb_state = MebInnerState::MEB_STATE_PASSIVE;
    } else {
      meb_state = meb_inner_state_;
    }
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_ACTIVE) {
    // 上一时刻处于LDW_FUNCTION_FSM_WORK_STATE_ACTIVE状态
    if (meb_fault_code_) {
      meb_state = MebInnerState::MEB_STATE_FAULT;
    } else if (meb_kickdown_code_) {
      meb_state = MebInnerState::MEB_STATE_PASSIVE;
    } else {
      meb_state = meb_inner_state_;
    }
  } else {
    // 处于异常状态
    meb_state = MebInnerState::MEB_STATE_OFF;
  }
  return meb_state;
}

void adas_function::meb_core::MebCore::SetMebOutputInfo() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 状态机映射
  if (meb_inner_state_ == MebInnerState::MEB_STATE_OFF) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_OFF;
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_FAULT) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_FAULT;
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_PASSIVE ||
             meb_inner_state_ == MebInnerState::MEB_STATE_STANDBY) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_STANDBY;
  } else if (meb_inner_state_ == MebInnerState::MEB_STATE_ACTIVE) {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_ACTIVE;
  } else {
    meb_state_ =
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_OFF;
  }

  if (GetContext.get_param()->meb_hmi_test_switch == true) {
    GetContext.mutable_output_info()->meb_output_info_.meb_state =
        (iflyauto::MEBFunctionFSMWorkState)GetContext.get_param()
            ->meb_hmi_state;
    GetContext.mutable_output_info()->meb_output_info_.meb_request_status =
        GetContext.get_param()->meb_hmi_status;
    GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
        GetContext.get_param()->meb_hmi_vaule;
    GetContext.mutable_output_info()->meb_output_info_.meb_request_direction =
        (iflyauto::MEBInterventionDirection)GetContext.get_param()
            ->meb_hmi_direction;
  } else {
    GetContext.mutable_output_info()->meb_output_info_.meb_state = meb_state_;
    if (GetContext.get_output_info()->meb_output_info_.meb_state ==
        iflyauto::MEBFunctionFSMWorkState::MEB_FUNCTION_FSM_WORK_STATE_ACTIVE) {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status = 1;
      if (GetContext.get_state_info()->shift_lever_state ==
          iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
        GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
            GetContext.get_param()->meb_request_acc *
            GetContext.get_param()->meb_reverse_acc_gain;
        GetContext.mutable_output_info()
            ->meb_output_info_.meb_request_direction =
            iflyauto::MEBInterventionDirection::MEB_INTERVENTION_DIRECTION_REAR;
      } else {
        GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
            GetContext.get_param()->meb_request_acc;
        GetContext.mutable_output_info()
            ->meb_output_info_.meb_request_direction = iflyauto::
            MEBInterventionDirection::MEB_INTERVENTION_DIRECTION_FRONT;
      }
    } else {
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status = 0;
      GetContext.mutable_output_info()->meb_output_info_.meb_request_value =
          0.0;
      GetContext.mutable_output_info()->meb_output_info_.meb_request_direction =
          iflyauto::MEBInterventionDirection::MEB_INTERVENTION_DIRECTION_NONE;
    }
  }

  JSON_DEBUG_VALUE(
      "meb_output_status",
      GetContext.get_output_info()->meb_output_info_.meb_request_status);
  JSON_DEBUG_VALUE(
      "meb_output_state",
      (int)GetContext.get_output_info()->meb_output_info_.meb_state);
  JSON_DEBUG_VALUE(
      "meb_output_value",
      GetContext.get_output_info()->meb_output_info_.meb_request_value);
  JSON_DEBUG_VALUE("meb_output_direction",
                   (int)GetContext.get_output_info()
                       ->meb_output_info_.meb_request_direction);
}

void adas_function::meb_core::MebCore::CollisionCalculate(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  double predict_t = meb_param_.predict_t;
  // double vel_radius = 10000.0;
  // double ego_curvature = GetContext.get_state_info()->ego_curvature;
  double ego_curvature =
      std::tan(vehicle_service_output_info_ptr->steering_wheel_angle /
               GetContext.get_param()->steer_ratio) /
      GetContext.get_param()->wheel_base;
  if (std::fabs(ego_curvature) < 0.0001) {
    meb_radius_ = 10000.0;
  } else {
    meb_radius_ = std::fabs(1.0 / ego_curvature);
  }
  bool vel_reverse_flag = false;
  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
    vel_reverse_flag = true;
  } else if (GetContext.get_state_info()->shift_lever_state ==
             iflyauto::ShiftLeverStateEnum::ShiftLeverState_N) {
    // Todo:空档下车辆溜坡判断，通过轮速传感器判断
  }
  predict_point_ << 0.0, 0.0;
  if (meb_radius_ > 5000.0) {
    // 按直线处理
    LineSegMent line_seg;
    line_seg.pA = {0.0, 0.0};
    double pb_x =
        std::max(GetContext.get_state_info()->vehicle_speed * predict_t, 0.5);
    double reverse_index = 1.0;
    if (vel_reverse_flag == true) {
      reverse_index = -1.0;
    }
    pb_x = reverse_index * pb_x;
    line_seg.pB = {pb_x, 0.0};
    line_seg.length =
        std::max(GetContext.get_state_info()->vehicle_speed * predict_t, 0.5);
    if (GetContext.get_param()->meb_od_obs_switch == true) {
      od_meb_result_ =
          LineCollisionUpdateCommonOd(line_seg, od_obs_vec_, od_obs_index_vec_);
    } else {
      od_meb_result_.collision_flag = false;
    }
    if (GetContext.get_param()->meb_occ_obs_switch == true) {
      occ_meb_result_ = LineCollisionUpdateCommon(
          line_seg, GetContext.get_param()->meb_dis_buffer, occ_obs_vec_);
    } else {
      occ_meb_result_.collision_flag = false;
    }
    if (GetContext.get_param()->meb_uss_obs_switch == true) {
      uss_meb_result_ = LineCollisionUpdateCommon(
          line_seg, GetContext.get_param()->meb_dis_buffer, uss_obs_vec_);
    } else {
      uss_meb_result_.collision_flag = false;
    }
    predict_point_ = line_seg.pB;
  } else {
    // 按圆弧处理
    ArcSegMent arc;
    arc.pA = {0.0, 0.0};
    arc.length =
        std::max(GetContext.get_state_info()->vehicle_speed * predict_t, 0.5);
    double delat_theta = arc.length / meb_radius_;
    double pb_x = std::sin(delat_theta) * meb_radius_;
    double reverse_index = 1.0;
    if (vel_reverse_flag == true) {
      reverse_index = -1.0;
    }
    pb_x = reverse_index * pb_x;
    double pb_y = 0.0;
    double pCenter_x = 0.0;
    double pCenter_y = 0.0;
    if (ego_curvature > 0.0) {
      // 方向盘向左打
      pb_y = 1.0 * (1.0 - std::cos(delat_theta)) * meb_radius_;
      pCenter_y = meb_radius_;
    } else {
      // 方向盘向右打
      pb_y = -1.0 * (1.0 - std::cos(delat_theta)) * meb_radius_;
      pCenter_y = -1.0 * meb_radius_;
    }
    arc.pB = {pb_x, pb_y};
    arc.circle_center = {0.0, pCenter_y};
    arc.circle_radius = meb_radius_;
    if (GetContext.get_param()->meb_od_obs_switch == true) {
      od_meb_result_ =
          ArcCollisionUpdateCommonOd(arc, od_obs_vec_, od_obs_index_vec_);
    } else {
      od_meb_result_.collision_flag = false;
    }
    if (GetContext.get_param()->meb_occ_obs_switch == true) {
      occ_meb_result_ = ArcCollisionUpdateCommon(
          arc, GetContext.get_param()->meb_dis_buffer, occ_obs_vec_);
    } else {
      occ_meb_result_.collision_flag = false;
    }
    if (GetContext.get_param()->meb_uss_obs_switch == true) {
      uss_meb_result_ = ArcCollisionUpdateCommon(
          arc, GetContext.get_param()->meb_dis_buffer, uss_obs_vec_);
    } else {
      uss_meb_result_.collision_flag = false;
    }
    predict_point_ = arc.pB;
  }

  // od boc collision calculate
  od_box_collision_flag_ = false;
  od_box_id_ = -1;
  if (GetContext.get_param()->meb_od_box_switch == true) {
    CollisionOdBoxCalculate();
  }
  // uss 碰撞计算
  // if (GetContext.get_param()->meb_uss_obs_switch) {
  //   uss_meb_result_ = CollisionUpdateUss();
  // } else {
  //   uss_meb_result_.collision_flag = false;
  // }
  return;
}

void MebCore::UpdateUssDistanceFunction() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  const auto &uss_dis_info_buf = GetContext.mutable_session()
                                     ->mutable_environmental_model()
                                     ->get_local_view()
                                     .uss_wave_info.sonar_distance_data;
  //  .uss_percept_info.dis_from_car_to_obj;
  uss_distance_vec_.clear();
  uss_distance_vec_.resize(meb_param_.uss_pos_index_vector.size());
  for (int i = 0; i < meb_param_.uss_pos_index_vector.size(); i++) {
    uss_distance_vec_[i] = uss_dis_info_buf[meb_param_.uss_pos_index_vector[i]]
                               .pas_sonarx_distance;
  }
  // JSON_DEBUG_VALUE("meb_uss_ttc_min", uss_meb_result_.ttc_min);
  return;
}

double MebCore::CollisionCalculateAcc(double remain_dist) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  double actuator_dis_buffer = 0.0;
  if (GetContext.get_param()->meb_dynamic_buffer_switch) {
    actuator_dis_buffer = GetContext.get_param()->meb_actuator_act_time *
                          GetContext.get_state_info()->vehicle_speed;
    // actuator_dis_buffer =
    //     actuator_dis_buffer + GetContext.get_param()->meb_dis_buffer;
  }
  double vel_fabs = std::fabs(GetContext.get_state_info()->vehicle_speed);
  double acc = vel_fabs * vel_fabs /
               (2.0 * std::max((remain_dist - actuator_dis_buffer), 0.05));

  acc = std::max(-1.0 * acc, -10.0);
  return acc;
}

void MebCore::CollisionOdBoxCalculate() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 整理输入
  const auto fusion_objects_info = GetContext.mutable_session()
                                       ->mutable_environmental_model()
                                       ->get_local_view()
                                       .fusion_objects_info;
  const uint8 fusion_od_obs_size =
      std::min(fusion_objects_info.fusion_object_size,
               static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
  double shift_direction_index = 1.0;
  double vel_speed_preprocess =
      std::max(GetContext.get_state_info()->vehicle_speed, 0.01);
  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverState_R) {
    shift_direction_index = -1.0;
    vel_speed_preprocess =
        std::min((shift_direction_index * vel_speed_preprocess), -0.01);
  }
  double ego_curvature =
      std::tan(GetContext.get_state_info()->steer_wheel_angle_degree /
               GetContext.get_param()->steer_ratio) /
      GetContext.get_param()->wheel_base;
  // 1.标定信息
  box_collision_.t_start = 0.0;
  box_collision_.t_end = 2.0;
  box_collision_.time_step = 0.1;

  box_collision_.dec_request =
      GetContext.get_param()->meb_acc_collision_thrd * shift_direction_index;
  // 2.车辆信息

  box_collision_.boxs_info_.ego_a_x =
      0.0;  // GetContext.get_state_info()->vel_acc;
  box_collision_.boxs_info_.ego_backshaft_2_fbumper =
      GetContext.get_param()->origin_2_front_bumper;
  box_collision_.boxs_info_.ego_length = GetContext.get_param()->ego_length;
  box_collision_.boxs_info_.ego_radius =
      (ego_curvature > -0.00009) ? meb_radius_ : (-1.0 * meb_radius_);
  box_collision_.boxs_info_.ego_v_x = vel_speed_preprocess;
  // shift_direction_index * GetContext.get_state_info()->vehicle_speed;
  // vel_speed_preprocess;
  box_collision_.boxs_info_.ego_width = GetContext.get_param()->ego_width;
  const auto &param = *GetContext.get_param();
  double od_box_dis_buffer = 0.0;
  for (uint8 i = 0; i < fusion_od_obs_size; ++i) {
    const iflyauto::Obstacle &obs =
        fusion_objects_info.fusion_object[i].common_info;

    double temp_buffer = 0.0;
    const bool is_reverse = GetContext.get_state_info()->shift_lever_state ==
                            iflyauto::ShiftLeverStateEnum::ShiftLeverState_R;
    const auto &speed_kmh = param.meb_odbox_dis_buffer_veh_speed_kmh;
    const auto group = GetOdObjGroup(obs.type);
    const auto &table = GetOdBufferTable(param, group, is_reverse, false);
    temp_buffer = pnc::mathlib::Interp1(speed_kmh, table,
                                        std::fabs(vel_speed_preprocess) * 3.6);
    box_collision_.time_dealy =
        GetContext.get_param()->meb_actuator_act_time +
        temp_buffer / std::max(0.5, GetContext.get_state_info()->vehicle_speed);
    // Pose2D center_pose(obs.relative_center_position.x,
    //                    obs.relative_center_position.y,
    //                    obs.relative_heading_angle);
    Eigen::Vector2d obs_pos;
    obs_pos << obs.relative_center_position.x, obs.relative_center_position.y;
    Eigen::Vector2d car_origin_pos;
    car_origin_pos << 0.0, 0.0;
    double car_point_distance = (car_origin_pos - obs_pos).norm();
    bool zone_in_flag = true;
    if (GetContext.get_state_info()->shift_lever_state ==
        iflyauto::ShiftLeverState_R) {
      if (obs.relative_center_position.x > meb_zone_x1_ + 0.5) {
        zone_in_flag = false;
      }
    } else {
      if (obs.relative_center_position.x < meb_zone_x2_ - 0.5) {
        zone_in_flag = false;
      }
    }
    if ((car_point_distance > 15.0) || (obs.type < 3) ||
        (std::fabs(obs.velocity.x) < 0.1 && std::fabs(obs.velocity.y) < 0.1) ||
        (zone_in_flag == false)) {
      continue;
    }
    // 只处理动态障碍物属性
    box_collision_.boxs_info_.obj_a_x = 0.0;  // obs.relative_acceleration.x;
    box_collision_.boxs_info_.obj_a_y = 0.0;  // obs.relative_acceleration.y;
    box_collision_.boxs_info_.obj_heading_angle = obs.relative_heading_angle;
    box_collision_.boxs_info_.obj_length = obs.shape.length;
    box_collision_.boxs_info_.obj_v_x =
        obs.relative_velocity.x + vel_speed_preprocess;
    box_collision_.boxs_info_.obj_v_y = obs.relative_velocity.y;
    box_collision_.boxs_info_.obj_width = obs.shape.width;
    box_collision_.boxs_info_.obj_x = obs.relative_center_position.x;
    box_collision_.boxs_info_.obj_y = obs.relative_center_position.y;

    od_box_collision_flag_ = box_collision_.GetCollisionResultBySimEgoDec(
        box_collision_.boxs_info_, box_collision_.t_start, box_collision_.t_end,
        box_collision_.time_step, box_collision_.time_dealy,
        box_collision_.dec_request);
    if (od_box_collision_flag_ == true) {
      od_box_id_ = obs.id;
      od_box_dis_buffer = temp_buffer;
      break;
    }
  }

  JSON_DEBUG_VALUE("od_box_dis_buffer", od_box_dis_buffer);
  JSON_DEBUG_VALUE("OdBox_index", od_box_id_);
  if ((od_box_id_ >= 0) &&
      (od_box_id_ < fusion_objects_info.fusion_object_size)) {
    JSON_DEBUG_VALUE(
        "OdBox_dis_type",
        (int)(fusion_objects_info.fusion_object[od_box_id_].common_info.type));
  } else {
    JSON_DEBUG_VALUE("OdBox_dis_type", (int)(-1));
  }
}

const MebResult adas_function::meb_core::MebCore::ArcCollisionUpdateCommon(
    const ArcSegMent &arc, const double dis_buffer,
    std::vector<Eigen::Vector2d> &obs_vec) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  if (obs_vec.size() < 1) {
    MebResult tmp_result;
    tmp_result.remain_path_distance = arc.length;
    tmp_result.remain_dist = arc.length;
    return tmp_result;
  }

  // obstacle arc segment
  const auto v_OA = arc.pA - arc.circle_center;
  const auto v_OB = arc.pB - arc.circle_center;
  const auto car_rot_angle = pnc::geometry_lib::GetAngleFromTwoVec(v_OA, v_OB);

  // obstacle rotates around the the car rotation center to form a circle
  // The minimum angle allowed for obstacle rotation
  auto min_obs_rot_limit_angle = 5.0;
  pnc::geometry_lib::Circle obs_rot_circle;
  // the cross points of obstacle circle and single car polygon line seg
  std::vector<Eigen::Vector2d> cross_points;
  // collision points between car movement along trajectory and obstacles
  Eigen::Vector2d col_pt_ego_global;
  col_pt_ego_global.setZero();
  size_t i = 0;
  size_t j = 0;
  MebResult result;
  result.relative_distance_min = 10.0;
  Eigen::Vector2d car_origin_pos;
  car_origin_pos << 0.0, 0.0;
  for (const auto &obs_pt_global : obs_vec) {
    double realtive_distancec = (car_origin_pos - obs_pt_global).norm();
    if (realtive_distancec < result.relative_distance_min) {
      result.relative_distance_min = realtive_distancec;
    }
    for (const auto &car_line_loc : car_line_local_vec_) {
      obs_rot_circle.center = arc.circle_center;
      obs_rot_circle.radius = (obs_pt_global - obs_rot_circle.center).norm();

      const auto num = pnc::geometry_lib::CalcCrossPointsOfLineSegAndCircle(
          car_line_loc, obs_rot_circle, cross_points);

      if (num == 0) {
        // if num == 0, no cross points, contiue
        continue;
      }

      const auto v_OC = obs_pt_global - arc.circle_center;
      Eigen::Vector2d D;
      if (num == 1) {
        D = cross_points.front();
      } else if (num == 2) {
        const auto v_OD1 = cross_points.front() - arc.circle_center;
        const auto v_OD2 = cross_points.back() - arc.circle_center;

        const auto obs_rot_angle1 =
            pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD1);

        const auto obs_rot_angle2 =
            pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD2);

        if (std::fabs(obs_rot_angle1) < std::fabs(obs_rot_angle2)) {
          D = cross_points.front();
        } else {
          D = cross_points.back();
        }
      }
      const auto v_OD = cross_points.front() - arc.circle_center;

      const auto obs_rot_angle =
          pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD);

      if (obs_rot_angle * car_rot_angle < 0.0 &&
          fabs(obs_rot_angle) < fabs(min_obs_rot_limit_angle)) {
        // the rotation direction of obstacles and car must be opposite
        min_obs_rot_limit_angle = obs_rot_angle;
        col_pt_ego_global = cross_points.front();
        result.nearest_point = obs_pt_global;
        i = j;
      }
    }
    j++;
  }

  result.remain_path_distance = fabs(car_rot_angle) * arc.circle_radius;
  result.remain_obstacle_dist =
      fabs(min_obs_rot_limit_angle) * arc.circle_radius;

  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_path_distance);
  result.ttc_min = result.remain_dist /
                   std::max(GetContext.get_state_info()->vehicle_speed, 0.5);

  result.acc_min = CollisionCalculateAcc(result.remain_dist - dis_buffer);
  if (result.acc_min < GetContext.get_param()->meb_acc_collision_thrd) {
    result.collision_flag = true;
  } else {
    result.collision_flag = false;
  }
  return result;
}

/*ccccccccccccccccc*/
const MebResult adas_function::meb_core::MebCore::ArcCollisionUpdateCommonOd(
    const ArcSegMent &arc, std::vector<Eigen::Vector2d> &obs_vec,
    std::vector<int> &obs_index_vec) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  if (obs_vec.size() < 1) {
    MebResult tmp_result;
    tmp_result.remain_path_distance = arc.length;
    tmp_result.remain_dist = arc.length;
    return tmp_result;
  }

  // obstacle arc segment
  const auto v_OA = arc.pA - arc.circle_center;
  const auto v_OB = arc.pB - arc.circle_center;
  const auto car_rot_angle = pnc::geometry_lib::GetAngleFromTwoVec(v_OA, v_OB);

  // obstacle rotates around the the car rotation center to form a circle
  // The minimum angle allowed for obstacle rotation
  auto min_obs_rot_limit_angle = 5.0;
  pnc::geometry_lib::Circle obs_rot_circle;
  // the cross points of obstacle circle and single car polygon line seg
  std::vector<Eigen::Vector2d> cross_points;
  // collision points between car movement along trajectory and obstacles
  Eigen::Vector2d col_pt_ego_global;
  col_pt_ego_global.setZero();
  size_t i = 0;
  size_t j = 0;
  MebResult result;
  result.relative_distance_min = 10.0;
  Eigen::Vector2d car_origin_pos;
  car_origin_pos << 0.0, 0.0;
  const auto &param = *GetContext.get_param();
  const bool is_reverse = GetContext.get_state_info()->shift_lever_state ==
                          iflyauto::ShiftLeverStateEnum::ShiftLeverState_R;
  const auto &speed_kmh = param.meb_odbox_dis_buffer_veh_speed_kmh;
  const auto &fusion_objects_info = GetContext.mutable_session()
                                        ->mutable_environmental_model()
                                        ->get_local_view()
                                        .fusion_objects_info;
  double selected_dis_buffer = 0.4;
  for (const auto &obs_pt_global : obs_vec) {
    double realtive_distancec = (car_origin_pos - obs_pt_global).norm();
    if (realtive_distancec < result.relative_distance_min) {
      result.relative_distance_min = realtive_distancec;
    }
    for (const auto &car_line_loc : car_line_local_vec_) {
      obs_rot_circle.center = arc.circle_center;
      obs_rot_circle.radius = (obs_pt_global - obs_rot_circle.center).norm();

      const auto num = pnc::geometry_lib::CalcCrossPointsOfLineSegAndCircle(
          car_line_loc, obs_rot_circle, cross_points);

      if (num == 0) {
        // if num == 0, no cross points, contiue
        continue;
      }

      const auto v_OC = obs_pt_global - arc.circle_center;
      Eigen::Vector2d D;
      if (num == 1) {
        D = cross_points.front();
      } else if (num == 2) {
        const auto v_OD1 = cross_points.front() - arc.circle_center;
        const auto v_OD2 = cross_points.back() - arc.circle_center;

        const auto obs_rot_angle1 =
            pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD1);

        const auto obs_rot_angle2 =
            pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD2);

        if (std::fabs(obs_rot_angle1) < std::fabs(obs_rot_angle2)) {
          D = cross_points.front();
        } else {
          D = cross_points.back();
        }
      }
      const auto v_OD = cross_points.front() - arc.circle_center;

      const auto obs_rot_angle =
          pnc::geometry_lib::GetAngleFromTwoVec(v_OC, v_OD);

      if (obs_rot_angle * car_rot_angle < 0.0 &&
          fabs(obs_rot_angle) < fabs(min_obs_rot_limit_angle)) {
        // the rotation direction of obstacles and car must be opposite
        min_obs_rot_limit_angle = obs_rot_angle;
        col_pt_ego_global = cross_points.front();
        result.nearest_point = obs_pt_global;
        if (j < obs_index_vec.size()) {
          const int obj_index = obs_index_vec[j];
          if (obj_index >= 0 &&
              obj_index <
                  static_cast<int>(fusion_objects_info.fusion_object_size)) {
            const auto obj_type =
                fusion_objects_info.fusion_object[obj_index].common_info.type;
            const auto group = GetOdObjGroup(obj_type);
            const auto &table =
                GetOdBufferTable(param, group, is_reverse, true);
            selected_dis_buffer = pnc::mathlib::Interp1(
                speed_kmh, table,
                std::fabs(GetContext.get_state_info()->vehicle_speed) * 3.6);
          }
        }
        i = j;
      }
    }
    j++;
  }
  JSON_DEBUG_VALUE("Arc_dis_type",
                   (int)(fusion_objects_info.fusion_object[obs_index_vec[i]]
                             .common_info.type));
  JSON_DEBUG_VALUE("Arc_dis_buffer", selected_dis_buffer);
  JSON_DEBUG_VALUE("Arc_index", obs_index_vec[i]);

  // MebResult result;

  result.remain_path_distance = fabs(car_rot_angle) * arc.circle_radius;
  // result.remain_obstacle_dist =
  //     fabs(min_obs_rot_limit_angle) * obs_rot_circle.radius; //err cal method
  result.remain_obstacle_dist =
      fabs(min_obs_rot_limit_angle) * arc.circle_radius;

  // result.collision_flag =
  //     (result.remain_obstacle_dist <= result.remain_path_distance);

  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_path_distance);
  result.ttc_min = result.remain_dist /
                   std::max(GetContext.get_state_info()->vehicle_speed, 0.5);

  result.acc_min =
      CollisionCalculateAcc(result.remain_dist - selected_dis_buffer);
  if (result.acc_min < GetContext.get_param()->meb_acc_collision_thrd) {
    result.collision_flag = true;
  } else {
    result.collision_flag = false;
  }
  return result;
}

const MebResult adas_function::meb_core::MebCore::LineCollisionUpdateCommon(
    const LineSegMent &line_seg, const double dis_buffer,
    std::vector<Eigen::Vector2d> &obs_vec) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 只需要碰撞距离,碰撞风险标志位
  if (obs_vec.size() < 1) {
    MebResult tmp_result;
    tmp_result.remain_path_distance = line_seg.length;
    tmp_result.remain_dist = line_seg.length;
    return tmp_result;
  }

  // segment
  double min_obs_move_dist = 33.3;
  pnc::geometry_lib::LineSegment obs_move_line;
  // the cross points of obstacle lin seg and single car polygon line seg
  Eigen::Vector2d cross_point;
  // collision points between car movement along trajectory and obstacles
  Eigen::Vector2d col_pt_ego_global(0.0, 0.0);

  const Eigen::Vector2d AB = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d unit_obs_move_line = -AB.normalized();
  size_t i = 0;
  size_t j = 0;

  MebResult result;
  result.relative_distance_min = 10.0;
  Eigen::Vector2d car_origin_pos;
  car_origin_pos << 0.0, 0.0;
  for (const auto &obs_pt_global : obs_vec) {
    double realtive_distancec = (car_origin_pos - obs_pt_global).norm();
    if (realtive_distancec < result.relative_distance_min) {
      result.relative_distance_min = realtive_distancec;
    }
    for (auto &car_line_loc : car_line_local_vec_) {
      obs_move_line.pA = obs_pt_global;
      // obs_move_line.pB = obs_move_line.pA - AB;
      obs_move_line.pB =
          obs_move_line.pA + min_obs_move_dist * unit_obs_move_line;
      if (GetIntersectionFromTwoLineSeg(cross_point, car_line_loc,
                                        obs_move_line)) {
        const auto dist_CP = (cross_point - obs_move_line.pA).norm();
        if (dist_CP < min_obs_move_dist) {
          col_pt_ego_global = cross_point;
          min_obs_move_dist = dist_CP;
          result.nearest_point = obs_pt_global;
          i = j;
        }
      }
    }
    j++;
  }

  result.remain_path_distance = AB.norm();
  result.remain_obstacle_dist = min_obs_move_dist;
  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_path_distance);

  // result.collision_flag =
  //     (result.remain_obstacle_dist <= result.remain_path_distance);
  result.ttc_min = result.remain_dist /
                   std::max(GetContext.get_state_info()->vehicle_speed, 0.5);

  result.acc_min = CollisionCalculateAcc(result.remain_dist - dis_buffer);
  if (result.acc_min < GetContext.get_param()->meb_acc_collision_thrd) {
    result.collision_flag = true;
  } else {
    result.collision_flag = false;
  }

  return result;
}

const MebResult adas_function::meb_core::MebCore::LineCollisionUpdateCommonOd(
    const LineSegMent &line_seg, std::vector<Eigen::Vector2d> &obs_vec,
    std::vector<int> &obs_index_vec) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  // 只需要碰撞距离,碰撞风险标志位
  if (obs_vec.size() < 1) {
    MebResult tmp_result;
    tmp_result.remain_path_distance = line_seg.length;
    tmp_result.remain_dist = line_seg.length;
    return tmp_result;
  }

  // segment
  double min_obs_move_dist = 33.3;
  pnc::geometry_lib::LineSegment obs_move_line;
  // the cross points of obstacle lin seg and single car polygon line seg
  Eigen::Vector2d cross_point;
  // collision points between car movement along trajectory and obstacles
  Eigen::Vector2d col_pt_ego_global(0.0, 0.0);

  const Eigen::Vector2d AB = line_seg.pB - line_seg.pA;
  const Eigen::Vector2d unit_obs_move_line = -AB.normalized();
  size_t i = 0;
  size_t j = 0;

  MebResult result;
  result.relative_distance_min = 10.0;
  Eigen::Vector2d car_origin_pos;
  car_origin_pos << 0.0, 0.0;
  const auto &param = *GetContext.get_param();
  const bool is_reverse = GetContext.get_state_info()->shift_lever_state ==
                          iflyauto::ShiftLeverStateEnum::ShiftLeverState_R;
  const auto &speed_kmh = param.meb_odbox_dis_buffer_veh_speed_kmh;
  const auto &fusion_objects_info = GetContext.mutable_session()
                                        ->mutable_environmental_model()
                                        ->get_local_view()
                                        .fusion_objects_info;

  double selected_dis_buffer = 0.4;

  for (const auto &obs_pt_global : obs_vec) {
    double realtive_distancec = (car_origin_pos - obs_pt_global).norm();
    if (realtive_distancec < result.relative_distance_min) {
      result.relative_distance_min = realtive_distancec;
    }
    for (auto &car_line_loc : car_line_local_vec_) {
      obs_move_line.pA = obs_pt_global;
      // obs_move_line.pB = obs_move_line.pA - AB;
      obs_move_line.pB =
          obs_move_line.pA + min_obs_move_dist * unit_obs_move_line;
      if (GetIntersectionFromTwoLineSeg(cross_point, car_line_loc,
                                        obs_move_line)) {
        const auto dist_CP = (cross_point - obs_move_line.pA).norm();
        if (dist_CP < min_obs_move_dist) {
          col_pt_ego_global = cross_point;
          min_obs_move_dist = dist_CP;
          result.nearest_point = obs_pt_global;

          if (j < obs_index_vec.size()) {
            const int obj_index = obs_index_vec[j];
            if (obj_index >= 0 &&
                obj_index <
                    static_cast<int>(fusion_objects_info.fusion_object_size)) {
              const auto obj_type =
                  fusion_objects_info.fusion_object[obj_index].common_info.type;
              const auto group = GetOdObjGroup(obj_type);
              const auto &table =
                  GetOdBufferTable(param, group, is_reverse, true);
              selected_dis_buffer = pnc::mathlib::Interp1(
                  speed_kmh, table,
                  std::fabs(GetContext.get_state_info()->vehicle_speed) * 3.6);
            }
          }
          i = j;
        }
      }
    }
    j++;
  }

  JSON_DEBUG_VALUE("Line_dis_type",
                   (int)(fusion_objects_info.fusion_object[obs_index_vec[i]]
                             .common_info.type));
  JSON_DEBUG_VALUE("Line_dis_buffer", selected_dis_buffer);
  JSON_DEBUG_VALUE("Line_index", (int)(obs_index_vec[i]));

  result.remain_path_distance = AB.norm();
  result.remain_obstacle_dist = min_obs_move_dist;
  result.remain_dist =
      std::min(result.remain_obstacle_dist, result.remain_path_distance);

  // result.collision_flag =
  //     (result.remain_obstacle_dist <= result.remain_path_distance);
  result.ttc_min = result.remain_dist /
                   std::max(GetContext.get_state_info()->vehicle_speed, 0.5);

  result.acc_min =
      CollisionCalculateAcc(result.remain_dist - selected_dis_buffer);
  if (result.acc_min < GetContext.get_param()->meb_acc_collision_thrd) {
    result.collision_flag = true;
  } else {
    result.collision_flag = false;
  }

  return result;
}

const MebResult MebCore::CollisionUpdateUss() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  MebResult tmp_result;
  bool vel_reverse_flag = false;
  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
    vel_reverse_flag = true;
  } else if (GetContext.get_state_info()->shift_lever_state ==
             iflyauto::ShiftLeverStateEnum::ShiftLeverState_N) {
    // Todo:空档下车辆溜坡判断，通过轮速传感器判断
  }

  std::vector<bool> uss_collision_result_vec = {false, false, false,
                                                false, false, false};
  int first_index = 0;
  if (vel_reverse_flag == true) {
    first_index = 6;
  }
  double min_dis = 10.0;
  double min_ttc = 10.0;
  bool collision_flag = false;
  double acc = 0.0;
  uss_acc_vec_.clear();
  uss_acc_vec_.resize(6);
  for (int i = first_index; i < (first_index + 6); i++) {
    if (uss_distance_vec_[i] < min_dis) {
      min_dis = uss_distance_vec_[i];
    }
    acc = CollisionCalculateAcc(uss_distance_vec_[i]);
    uss_acc_vec_[i - first_index] = acc;
    if (acc < GetContext.get_param()->meb_acc_collision_thrd) {
      uss_collision_result_vec[i - first_index] = true;
      tmp_result.collision_flag = true;
    } else {
      uss_collision_result_vec[i - first_index] = false;
    }
  }

  tmp_result.remain_dist = min_dis;
  tmp_result.remain_obstacle_dist = min_dis;
  tmp_result.remain_path_distance = 7.5;
  tmp_result.acc_min =
      *std::min_element(uss_acc_vec_.begin(), uss_acc_vec_.end());
  tmp_result.ttc_min =
      min_dis / std::max(GetContext.get_state_info()->vehicle_speed, 0.5);
  if (tmp_result.collision_flag == true) {
    // 校核相邻两个点是否碰撞

    if (uss_collision_result_vec[1] && uss_collision_result_vec[2]) {
      tmp_result.collision_flag = true;
    } else if (uss_collision_result_vec[2] && uss_collision_result_vec[3]) {
      tmp_result.collision_flag = true;
    } else if (uss_collision_result_vec[3] && uss_collision_result_vec[4]) {
      tmp_result.collision_flag = true;
    } else {
      if (meb_param_.dynamic_uss_collision_switch == true) {
        if (uss_collision_result_vec[1] &&
            (uss_acc_vec_[2] <
             GetContext.get_param()->meb_acc_collision_thrd_bak)) {
          tmp_result.collision_flag = true;
        } else if (uss_collision_result_vec[2] &&
                   (uss_acc_vec_[1] <
                        GetContext.get_param()->meb_acc_collision_thrd_bak ||
                    uss_acc_vec_[3] <
                        GetContext.get_param()->meb_acc_collision_thrd_bak)) {
          tmp_result.collision_flag = true;
        } else if (uss_collision_result_vec[3] &&
                   (uss_acc_vec_[2] <
                        GetContext.get_param()->meb_acc_collision_thrd_bak ||
                    uss_acc_vec_[4] <
                        GetContext.get_param()->meb_acc_collision_thrd_bak)) {
          tmp_result.collision_flag = true;
        } else if (uss_collision_result_vec[4] &&
                   (uss_acc_vec_[3] <
                    GetContext.get_param()->meb_acc_collision_thrd_bak)) {
          tmp_result.collision_flag = true;
        } else {
          tmp_result.collision_flag = false;
        }
      } else {
        tmp_result.collision_flag = false;
      }
    }
  } else {
    tmp_result.collision_flag = false;
  }

  return tmp_result;
}

const bool MebCore::IsPointInZone(Eigen::Vector2d point) {
  /*
 y1         y2
x1-|----------|----
 |          |
 |          |
 |          |
 |          |
 |          |
 |          |
x2-|----------|--
 |          |
*/
  if (point.x() > meb_zone_x1_ || point.x() < meb_zone_x2_ ||
      point.y() > meb_zone_y1_ || point.y() < meb_zone_y2_) {
    return false;
  }

  return true;
}

void adas_function::meb_core::MebCore::UpdateObstacles() {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();

  const auto rotm2d = GetContext.get_state_info()->rotm2d;
  const uint8 fusion_occ_obs_size =
      std::min(GetContext.mutable_session()
                   ->mutable_environmental_model()
                   ->get_local_view()
                   .fusion_occupancy_objects_info.fusion_object_size,
               static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
  occ_obs_vec_.clear();
  od_obs_vec_.clear();
  /*计算有效碰撞区域*/

  /*
   y1         y2
x1-|----------|----
   |          |
   |          |
   |          |
   |          |
   |          |
   |          |
x2-|----------|--
   |          |
  */
  bool vel_reverse_flag = false;
  if (GetContext.get_state_info()->shift_lever_state ==
      iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
    vel_reverse_flag = true;
  }
  double y_traj_radius_gain = 10.0;
  std::vector<double> meb_radius_vector = {5.0, 10.0, 20.0, 40.0, 60.0, 100.0};
  std::vector<double> meb_radius_vector_gain = {2.5, 2.0, 1.6, 1.2, 1.0, 1.0};
  y_traj_radius_gain = pnc::mathlib::Interp1(
      meb_radius_vector, meb_radius_vector_gain, meb_radius_);
  double x_range =
      std::max(10.0, GetContext.get_state_info()->vehicle_speed * 2.0);
  // x_range = std::min(5.0, x_range);
  if (vel_reverse_flag == false) {
    meb_zone_x1_ = GetContext.get_param()->origin_2_front_bumper + x_range;
    meb_zone_x2_ = GetContext.get_param()->origin_2_front_bumper -
                   GetContext.get_param()->wheel_base - 0.2;
    meb_zone_y1_ =
        0.6 * GetContext.get_param()->ego_width + 1.0 * y_traj_radius_gain;
    meb_zone_y2_ =
        -0.6 * GetContext.get_param()->ego_width - 1.0 * y_traj_radius_gain;
  } else {
    meb_zone_x1_ = 0.2;
    meb_zone_x2_ =
        -1.0 * GetContext.get_param()->origin_2_rear_bumper - x_range;
    meb_zone_y1_ =
        0.6 * GetContext.get_param()->ego_width + 1.0 * y_traj_radius_gain;
    meb_zone_y2_ =
        -0.6 * GetContext.get_param()->ego_width - 1.0 * y_traj_radius_gain;
  }

  const Eigen::Vector2d car_pos = GetContext.get_state_info()->current_pos_i;
  double car_point_distance = 0.0;

  for (uint8 i = 0; i < fusion_occ_obs_size; ++i) {
    // [hack]: need to retire in published version.
    const iflyauto::FusionOccupancyAdditional &fusion_occupancy_object =
        GetContext.mutable_session()
            ->mutable_environmental_model()
            ->get_local_view()
            .fusion_occupancy_objects_info.fusion_object[i]
            .additional_occupancy_info;
    const uint32 polygon_points_size =
        std::min(fusion_occupancy_object.polygon_points_size,
                 static_cast<uint32>(
                     FUSION_OCCUPANCY_OBJECTS_POLYGON_POINTS_SET_MAX_NUM));
    if (polygon_points_size < 1) {
      continue;
    }

    for (uint32 j = 0; j < polygon_points_size; ++j) {
      const Eigen::Vector2d fusion_pt(
          fusion_occupancy_object.polygon_points[j].x,
          fusion_occupancy_object.polygon_points[j].y);

      occ_obs_vec_.reserve(occ_obs_vec_.size() + 1);
      const Eigen::Vector2d fusion_loc =
          rotm2d.transpose() *
          (fusion_pt - GetContext.get_state_info()->current_pos_i);
      /*根据区域筛选点*/
      if (IsPointInZone(fusion_loc)) {
        occ_obs_vec_.emplace_back(fusion_loc);
      }
    }
  }

  // oddddddd
  const auto fusion_objects_info = GetContext.mutable_session()
                                       ->mutable_environmental_model()
                                       ->get_local_view()
                                       .fusion_objects_info;
  const uint8 fusion_od_obs_size =
      std::min(fusion_objects_info.fusion_object_size,
               static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
  od_obs_vec_.clear();
  od_obs_index_vec_.clear();
  for (uint8 i = 0; i < fusion_od_obs_size; ++i) {
    const iflyauto::Obstacle &obs =
        fusion_objects_info.fusion_object[i].common_info;

    Pose2D center_pose(obs.relative_center_position.x,
                       obs.relative_center_position.y,
                       obs.relative_heading_angle);

    Eigen::Vector2d obs_pos;
    obs_pos << obs.relative_center_position.x, obs.relative_center_position.y;
    Eigen::Vector2d car_origin_pos;
    car_origin_pos << 0.0, 0.0;
    car_point_distance = (car_origin_pos - obs_pos).norm();
    if ((car_point_distance > 15.0) || (obs.type < 3) ||
        (std::fabs(obs.velocity.x > 0.1) || std::fabs(obs.velocity.y) > 0.1)) {
      continue;
    }
    // 只保留静态属性障碍物
    double sin_theta_ = std::sin(obs.relative_heading_angle);
    double cos_theta_ = std::cos(obs.relative_heading_angle);
    double dx1 = 0.5 * obs.shape.length * cos_theta_;
    double dy1 = 0.5 * obs.shape.length * sin_theta_;
    double dx2 = 0.5 * obs.shape.width * sin_theta_;
    double dy2 = 0.5 * obs.shape.width * cos_theta_;

    // 获取前左角点坐标
    Eigen::Vector2d point0;
    point0.x() = obs.relative_center_position.x + dx1 - dx2;
    point0.y() = obs.relative_center_position.y + dy1 + dy2;
    if (IsPointInZone(point0)) {
      od_obs_vec_.emplace_back(point0);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
    // 获取前右角点坐标
    Eigen::Vector2d point1;
    point1.x() = obs.relative_center_position.x + dx1 + dx2;
    point1.y() = obs.relative_center_position.y + dy1 - dy2;
    if (IsPointInZone(point1)) {
      od_obs_vec_.emplace_back(point1);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
    // 获取后左角点坐标
    Eigen::Vector2d point2;
    point2.x() = obs.relative_center_position.x - dx1 - dx2;
    point2.y() = obs.relative_center_position.y - dy1 + dy2;
    if (IsPointInZone(point2)) {
      od_obs_vec_.emplace_back(point2);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
    // 获取后右角点坐标
    Eigen::Vector2d point3;
    point3.x() = obs.relative_center_position.x - dx1 + dx2;
    point3.y() = obs.relative_center_position.y - dy1 - dy2;
    if (IsPointInZone(point3)) {
      od_obs_vec_.emplace_back(point3);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
    Eigen::Vector2d point4;
    point4.x() = (point0.x() + point1.x()) * 0.5;
    point4.y() = (point0.y() + point1.y()) * 0.5;
    if (IsPointInZone(point4)) {
      od_obs_vec_.emplace_back(point4);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
    point4.x() = (point3.x() + point1.x()) * 0.5;
    point4.y() = (point3.y() + point1.y()) * 0.5;
    if (IsPointInZone(point4)) {
      od_obs_vec_.emplace_back(point4);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
    point4.x() = (point0.x() + point2.x()) * 0.5;
    point4.y() = (point0.y() + point2.y()) * 0.5;
    if (IsPointInZone(point4)) {
      od_obs_vec_.emplace_back(point4);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
    point4.x() = (point2.x() + point3.x()) * 0.5;
    point4.y() = (point2.y() + point3.y()) * 0.5;
    if (IsPointInZone(point4)) {
      od_obs_vec_.emplace_back(point4);
      od_obs_index_vec_.emplace_back(static_cast<int>(i));
    }
  }
  // usssssssssssss
  uss_distance_vec_.clear();
  uss_distance_vec_.resize(0);
  uss_obs_vec_.clear();
  uss_obs_vec_.resize(0);
  if (GetContext.get_param()->meb_uss_obs_switch) {
    UpdateUssDistanceFunction();
    const auto uss_obs_data = GetContext.mutable_session()
                                  ->mutable_environmental_model()
                                  ->get_local_view()
                                  .uss_percept_info.out_line_dataori[0];
    for (int i = 0; i < uss_obs_data.obj_pt_cnt; ++i) {
      const Eigen::Vector2d uss_pt(uss_obs_data.obj_pt_local[i].x,
                                   uss_obs_data.obj_pt_local[i].y);
      /*根据区域筛选点*/
      if (IsPointInZone(uss_pt)) {
        uss_obs_vec_.reserve(uss_obs_vec_.size() + 1);
        uss_obs_vec_.emplace_back(uss_pt);
      }
    }
  }
  return;
}

void adas_function::meb_core::MebCore::RunOnce(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;
  // if (meb_state_machine_init_flag_ == false) {
  //   Init();
  // }
  meb_main_switch_ = UpdateMebMainSwitch();
  meb_enable_code_ = UpdateMebEnableCode();
  meb_disable_code_ = UpdateMebDisableCode();
  meb_fault_code_ = UpdateMebFaultCode();
  meb_kickdown_code_ = UpdateMebKickdownCode();
  meb_intervention_flag_ = UpdateIntervention();
  meb_inner_state_ = MebStateMachine();
  SetMebOutputInfo();
  Log();
}

void adas_function::meb_core::MebCore::Log(void) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  auto vehicle_service_output_info_ptr = &GetContext.mutable_session()
                                              ->mutable_environmental_model()
                                              ->get_local_view()
                                              .vehicle_service_output_info;

  // const auto rotm2d = GetContext.get_state_info()->rotm2d;
  const uint8 fusion_obs_size =
      std::min(GetContext.mutable_session()
                   ->mutable_environmental_model()
                   ->get_local_view()
                   .fusion_occupancy_objects_info.fusion_object_size,
               static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
  const auto fusion_objects_info = GetContext.mutable_session()
                                       ->mutable_environmental_model()
                                       ->get_local_view()
                                       .fusion_objects_info;
  const uint8 fusion_od_obs_size =
      std::min(fusion_objects_info.fusion_object_size,
               static_cast<uint8>(FUSION_OCCUPANCY_OBJECTS_MAX_NUM));
  // log
  JSON_DEBUG_VALUE("meb_inner_state", (uint32)meb_inner_state_);
  JSON_DEBUG_VALUE("meb_state", (uint32)meb_state_);
  JSON_DEBUG_VALUE("meb_main_switch", meb_main_switch_);
  JSON_DEBUG_VALUE("meb_enable_code", meb_enable_code_);
  JSON_DEBUG_VALUE("meb_disable_code", meb_disable_code_);
  JSON_DEBUG_VALUE("meb_fault_code", meb_fault_code_);
  JSON_DEBUG_VALUE("meb_intervention_flag", meb_intervention_flag_);
  JSON_DEBUG_VALUE("meb_od_obs_collision_flag", od_meb_result_.collision_flag);
  JSON_DEBUG_VALUE("meb_occ_obs_collision_flag",
                   occ_meb_result_.collision_flag);
  JSON_DEBUG_VALUE("meb_kickdown_code", meb_kickdown_code_);
  JSON_DEBUG_VALUE(
      "meb_all_obs_point_num",
      od_obs_vec_.size() + occ_obs_vec_.size() + uss_obs_vec_.size());
  JSON_DEBUG_VALUE("meb_obs_distance", std::min(od_meb_result_.remain_dist,
                                                occ_meb_result_.remain_dist));
  JSON_DEBUG_VALUE("meb_od_obs_distance", od_meb_result_.remain_dist);
  JSON_DEBUG_VALUE("meb_occ_obs_distance", occ_meb_result_.remain_dist);
  JSON_DEBUG_VALUE("meb_od_path_distance", od_meb_result_.remain_path_distance);
  JSON_DEBUG_VALUE("meb_occ_path_distance",
                   occ_meb_result_.remain_path_distance);

  JSON_DEBUG_VALUE("meb_radius", meb_radius_);
  JSON_DEBUG_VALUE("meb_fusion_occ_obs_size", occ_obs_vec_.size());
  JSON_DEBUG_VALUE("meb_fusion_od_obs_size", od_obs_vec_.size());
  JSON_DEBUG_VALUE("meb_fusion_uss_obs_size", uss_obs_vec_.size());
  JSON_DEBUG_VALUE("meb_relative_distance_min",
                   std::min(od_meb_result_.relative_distance_min,
                            occ_meb_result_.relative_distance_min));
  JSON_DEBUG_VALUE("meb_uss_obs_distance", uss_meb_result_.remain_dist);
  JSON_DEBUG_VALUE("meb_uss_collision_flag", uss_meb_result_.collision_flag);
  JSON_DEBUG_VALUE("meb_od_acc_min", od_meb_result_.acc_min);
  JSON_DEBUG_VALUE("meb_occ_acc_min", occ_meb_result_.acc_min);
  JSON_DEBUG_VALUE("meb_uss_acc_min", uss_meb_result_.acc_min);
  JSON_DEBUG_VALUE("meb_od_ttc_min", od_meb_result_.ttc_min);
  JSON_DEBUG_VALUE("meb_occ_ttc_min", occ_meb_result_.ttc_min);
  JSON_DEBUG_VALUE("meb_uss_ttc_min", uss_meb_result_.ttc_min);
  JSON_DEBUG_VALUE("meb_od_box_collision_flag", od_box_collision_flag_);
  JSON_DEBUG_VALUE("meb_od_box_id", od_box_id_);

  bool switch_for_sim = true;
  std::vector<double> meb_od_obs_x_vector;
  std::vector<double> meb_od_obs_y_vector;
  std::vector<double> meb_occ_obs_x_vector;
  std::vector<double> meb_occ_obs_y_vector;
  std::vector<double> meb_uss_obs_x_vector;
  std::vector<double> meb_uss_obs_y_vector;
  std::vector<double> meb_point_x_vector;
  std::vector<double> meb_point_y_vector;
  meb_od_obs_x_vector.clear();
  meb_od_obs_y_vector.clear();
  meb_occ_obs_x_vector.clear();
  meb_occ_obs_y_vector.clear();
  meb_uss_obs_x_vector.clear();
  meb_uss_obs_y_vector.clear();
  meb_point_x_vector.clear();
  meb_point_y_vector.clear();
  meb_point_x_vector.resize(3);
  meb_point_y_vector.resize(3);
  if (switch_for_sim == true) {
    if (od_obs_vec_.size() + occ_obs_vec_.size() + uss_obs_vec_.size() > 0) {
      meb_od_obs_x_vector.resize(od_obs_vec_.size());
      meb_od_obs_y_vector.resize(od_obs_vec_.size());
      for (int i = 0; i < od_obs_vec_.size(); i++) {
        meb_od_obs_x_vector[i] = od_obs_vec_[i].x();
        meb_od_obs_y_vector[i] = od_obs_vec_[i].y();
      }
      meb_occ_obs_x_vector.resize(occ_obs_vec_.size());
      meb_occ_obs_y_vector.resize(occ_obs_vec_.size());
      for (int i = 0; i < occ_obs_vec_.size(); i++) {
        meb_occ_obs_x_vector[i] = occ_obs_vec_[i].x();
        meb_occ_obs_y_vector[i] = occ_obs_vec_[i].y();
      }
      meb_occ_obs_x_vector.resize(occ_obs_vec_.size());
      meb_occ_obs_y_vector.resize(occ_obs_vec_.size());
      for (int i = 0; i < occ_obs_vec_.size(); i++) {
        meb_occ_obs_x_vector[i] = occ_obs_vec_[i].x();
        meb_occ_obs_y_vector[i] = occ_obs_vec_[i].y();
      }
      meb_uss_obs_x_vector.resize(uss_obs_vec_.size());
      meb_uss_obs_y_vector.resize(uss_obs_vec_.size());
      for (int i = 0; i < uss_obs_vec_.size(); i++) {
        meb_uss_obs_x_vector[i] = uss_obs_vec_[i].x();
        meb_uss_obs_y_vector[i] = uss_obs_vec_[i].y();
      }
    } else {
      meb_od_obs_x_vector.resize(1);
      meb_od_obs_y_vector.resize(1);
      meb_od_obs_x_vector[0] = 0.0;
      meb_od_obs_y_vector[0] = 0.0;

      meb_occ_obs_x_vector.resize(1);
      meb_occ_obs_y_vector.resize(1);
      meb_occ_obs_x_vector[0] = 0.0;
      meb_occ_obs_y_vector[0] = 0.0;

      meb_uss_obs_x_vector.resize(1);
      meb_uss_obs_y_vector.resize(1);
      meb_uss_obs_x_vector[0] = 0.0;
      meb_uss_obs_y_vector[0] = 0.0;
    }

    JSON_DEBUG_VECTOR("meb_od_obs_x_vector", meb_od_obs_x_vector, 2);
    JSON_DEBUG_VECTOR("meb_od_obs_y_vector", meb_od_obs_y_vector, 2);
    JSON_DEBUG_VECTOR("meb_occ_obs_x_vector", meb_occ_obs_x_vector, 2);
    JSON_DEBUG_VECTOR("meb_occ_obs_y_vector", meb_occ_obs_y_vector, 2);
    JSON_DEBUG_VECTOR("meb_uss_obs_x_vector", meb_uss_obs_x_vector, 2);
    JSON_DEBUG_VECTOR("meb_uss_obs_y_vector", meb_uss_obs_y_vector, 2);
    // 保存轨迹点
    bool vel_reverse_flag = false;
    if (GetContext.get_state_info()->shift_lever_state ==
        iflyauto::ShiftLeverStateEnum::ShiftLeverState_R) {
      vel_reverse_flag = true;
    } else if (GetContext.get_state_info()->shift_lever_state ==
               iflyauto::ShiftLeverStateEnum::ShiftLeverState_N) {
      // Todo:空档下车辆溜坡判断，通过轮速传感器判断
    }
    std::vector<double> meb_traj_x_vector;
    std::vector<double> meb_traj_y_vector;
    std::vector<double> meb_traj_dphi_vector;
    meb_traj_x_vector.resize(10);
    meb_traj_y_vector.resize(10);
    meb_traj_dphi_vector.resize(10);
    predict_point_ << 0.0, 0.0;
    if (meb_radius_ > 60.0) {
      double reverse_index = 1.0;
      if (vel_reverse_flag == true) {
        reverse_index = -1.0;
      }
      for (int i = 0; i < meb_traj_y_vector.size(); i++) {
        meb_traj_x_vector[i] = 0.0 + reverse_index * i * 0.5;
        meb_traj_y_vector[i] = 0.0;
        meb_traj_dphi_vector[i] = 0.0;
      }
    } else {
      // 按圆弧处理
      double reverse_index = 1.0;
      if (vel_reverse_flag == true) {
        reverse_index = -1.0;
      }

      for (int i = 0; i < meb_traj_y_vector.size(); i++) {
        double length_gap = (double)i * 0.5;
        double delat_gap = length_gap / meb_radius_;
        meb_traj_x_vector[i] =
            0.0 + reverse_index * std::sin(delat_gap) * meb_radius_;
        if (vehicle_service_output_info_ptr->steering_wheel_angle > 0.0) {
          // 方向盘向左打
          meb_traj_y_vector[i] =
              1.0 * (1.0 - std::cos(delat_gap)) * meb_radius_;
        } else {
          // 方向盘向右打
          meb_traj_y_vector[i] =
              -1.0 * (1.0 - std::cos(delat_gap)) * meb_radius_;
        }
      }

      // dphiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
      double s = 0.0;
      std::vector<double> dx_vec_;
      std::vector<double> dy_vec_;
      std::vector<double> s_vec_;
      dx_vec_.clear();
      dy_vec_.clear();
      s_vec_.clear();
      s_vec_.resize(10);
      dx_vec_.resize(10);
      dy_vec_.resize(10);
      for (size_t i = 0; i < 10; ++i) {
        dx_vec_[i] = meb_traj_x_vector[i];
        dy_vec_[i] = meb_traj_y_vector[i];
        if (i == 0) {
          s_vec_[i] = 0.0;
        } else {
          const double ds = std::hypot(dx_vec_[i] - dx_vec_[i - 1],
                                       dy_vec_[i] - dy_vec_[i - 1]);
          s += std::max(ds, 1e-3);
          s_vec_[i] = s;
        }
      }

      // longitudinal reference
      pnc::mathlib::spline dx_s_spline_;
      pnc::mathlib::spline dy_s_spline_;
      dx_s_spline_.set_points(s_vec_, dx_vec_);
      dy_s_spline_.set_points(s_vec_, dy_vec_);
      double s_ref = 0.0;
      for (size_t i = 0; i < 10; ++i) {
        double dx_s_derv_1st = dx_s_spline_.deriv(1, s_ref);
        double dy_s_derv_1st = dy_s_spline_.deriv(1, s_ref);

        if (vel_reverse_flag) {
          dx_s_derv_1st *= -1.0;
          dy_s_derv_1st *= -1.0;
        }

        double dphi_ref = std::atan2(dy_s_derv_1st, dx_s_derv_1st);
        if (dphi_ref > 3.1416 * 0.5) {
          dphi_ref -= 3.1416;
        } else if (dphi_ref < -3.1416 * 0.5) {
          dphi_ref += 3.1416;
        }
        meb_traj_dphi_vector[i] = dphi_ref;
        s_ref += 1.0;
      }

      // end dohiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii
    }
    JSON_DEBUG_VECTOR("meb_traj_x_vector", meb_traj_x_vector, 2);
    JSON_DEBUG_VECTOR("meb_traj_y_vector", meb_traj_y_vector, 2);
    JSON_DEBUG_VECTOR("meb_traj_dphi_vector", meb_traj_dphi_vector, 2);
  }
  meb_point_x_vector[0] = predict_point_.x();
  meb_point_x_vector[1] = od_meb_result_.nearest_point.x();
  meb_point_x_vector[2] = occ_meb_result_.nearest_point.x();
  meb_point_y_vector[0] = predict_point_.y();
  meb_point_y_vector[1] = od_meb_result_.nearest_point.y();
  meb_point_y_vector[2] = occ_meb_result_.nearest_point.y();
  JSON_DEBUG_VECTOR("meb_point_x_vector", meb_point_x_vector, 2);
  JSON_DEBUG_VECTOR("meb_point_y_vector", meb_point_y_vector, 2);
  JSON_DEBUG_VECTOR("uss_distance_vec", uss_distance_vec_, 2);
  JSON_DEBUG_VECTOR("uss_acc_vec_", uss_acc_vec_, 2);

  JSON_DEBUG_VALUE(
      "meb_output_state",
      (int)GetContext.mutable_output_info()->meb_output_info_.meb_state);
  JSON_DEBUG_VALUE(
      "meb_output_requset_status",
      GetContext.mutable_output_info()->meb_output_info_.meb_request_status);
  JSON_DEBUG_VALUE(
      "meb_output_requset_value",
      GetContext.mutable_output_info()->meb_output_info_.meb_request_value);
}

}  // namespace meb_core
}  // namespace adas_function