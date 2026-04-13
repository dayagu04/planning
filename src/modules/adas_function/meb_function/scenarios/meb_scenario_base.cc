#include "meb_scenario_base.h"

using namespace planning;
namespace adas_function {

const std::vector<double> MebScenarioBase::GetOdBufferTable(
    const adas_function::context::Parameters &param, const OdObjGroup group,
    const bool is_reverse, const bool is_static, bool park_mode) {
  // 泊车模式参数
  if (park_mode) {
    if (is_reverse) {
      if (is_static) {
        switch (group) {
          case OdObjGroup::kPeople:
            return param.park_meb_odbox_dis_buffer_r_static_people;
          case OdObjGroup::kCar:
            return param.park_meb_odbox_dis_buffer_r_static_car;
          case OdObjGroup::kMotor:
            return param.park_meb_odbox_dis_buffer_r_static_motor;
          default:
            return param.park_meb_odbox_dis_buffer_r_static_default;
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
          return param.park_meb_odbox_dis_buffer_d_static_people;
        case OdObjGroup::kCar:
          return param.park_meb_odbox_dis_buffer_d_static_car;
        case OdObjGroup::kMotor:
          return param.park_meb_odbox_dis_buffer_d_static_motor;
        default:
          return param.park_meb_odbox_dis_buffer_d_static_default;
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

  // 行车模式
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

void MebScenarioBase::CollisionCalculate(double stop_distance_buffer_reduction,
                                         bool is_obs_straight) {
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  const auto &param = *GetContext.get_param();
  auto &meb_pre = adas_function::MebPreprocess::GetInstance();

  float32 radius = meb_pre.GetInstance().GetMebInput().ego_radius;
  float32 meb_acc_collision_thrd = param.meb_acc_collision_thrd;
  float32 ego_backshaft_2_fbumper = param.origin_2_front_bumper;
  float32 ego_length = param.ego_length;
  float32 ego_width = param.ego_width;
  float32 ego_acc = GetContext.get_state_info()->vel_acc;

  double shift_direction_index =
      meb_pre.GetInstance().GetMebInput().shift_direction_index;
  double vel_speed_preprocess =
      meb_pre.GetInstance().GetMebInput().signed_ego_vel_mps;

  //修正本车的加速度,只考虑本车减速工况,不考虑本车加速工况
  if (vel_speed_preprocess > 0.0) {
    // 本车处于前进状态
    if (ego_acc > 0.0) {
      ego_acc = 0.0;
    }
  } else {
    // 本车处于倒车状态
    if (ego_acc < 0.0) {
      ego_acc = 0.0;
    }
  }

  // 1.标定信息
  box_collision_.t_start = 0.0;
  box_collision_.t_end = 2.0;
  box_collision_.time_step = 0.1;
  box_collision_.dec_request = meb_acc_collision_thrd * shift_direction_index;

  // 2.车辆信息
  box_collision_.boxs_info_.ego_a_x = ego_acc;
  box_collision_.boxs_info_.ego_backshaft_2_fbumper = ego_backshaft_2_fbumper;
  box_collision_.boxs_info_.ego_length = ego_length;
  box_collision_.boxs_info_.ego_radius = radius;
  box_collision_.boxs_info_.ego_v_x = vel_speed_preprocess;
  box_collision_.boxs_info_.ego_width = ego_width;

  for (auto &obs : interest_obj_info_.interest_obj_vec_) {
    const bool is_reverse = GetContext.get_state_info()->shift_lever_state ==
                            iflyauto::ShiftLeverStateEnum::ShiftLeverState_R;
    const auto &ego_speed_kph_table = param.meb_odbox_dis_buffer_veh_speed_kmh;
    double obs_v = sqrt((obs.rel_vx + vel_speed_preprocess) *
                            (obs.rel_vx + vel_speed_preprocess) +
                        obs.rel_vy * obs.rel_vy);
    bool is_static;
    if (obs_v < 1.0) {
      is_static = true;
    } else {
      is_static = false;
    }
    const auto &stop_distance_buffer_table =
        GetOdBufferTable(param, obs.type_for_meb, is_reverse, is_static,
                         meb_pre.GetMebInput().park_mode);
    double temp_buffer =
        pnc::mathlib::Interp1(ego_speed_kph_table, stop_distance_buffer_table,
                              std::fabs(vel_speed_preprocess) * 3.6);
    obs.stop_distance_buffer = temp_buffer - stop_distance_buffer_reduction;
    // 防止出现异常
    if (obs.stop_distance_buffer < 0.1) {
      obs.stop_distance_buffer = 0.1;
    }
    box_collision_.time_dealy =
        GetContext.get_param()->meb_actuator_act_time +
        obs.stop_distance_buffer /
            std::max(0.5, GetContext.get_state_info()->vehicle_speed);

    box_collision_.boxs_info_.obj_a_x = 0.0;  // obs.relative_acceleration.x;
    box_collision_.boxs_info_.obj_a_y = 0.0;  // obs.relative_acceleration.y;
    box_collision_.boxs_info_.obj_heading_angle = obs.rel_heading_angle;
    box_collision_.boxs_info_.obj_length = obs.length;

    if ((obs.abs_vx * vel_speed_preprocess < 0.0) && (is_obs_straight)) {
      // 当你直行，且障碍物靠近车辆的速度，则认为障碍物是静止的。
      box_collision_.boxs_info_.obj_v_x = 0.0;
    } else {
      box_collision_.boxs_info_.obj_v_x =
          obs.rel_vx + vel_speed_preprocess;  // 需要考虑自车前进及后退
    }

    box_collision_.boxs_info_.obj_v_y = obs.rel_vy;
    box_collision_.boxs_info_.obj_width = obs.width;
    box_collision_.boxs_info_.obj_x = obs.rel_x;
    box_collision_.boxs_info_.obj_y = obs.rel_y;

    obs.is_collision = box_collision_.GetCollisionResultBySimEgoDec(
        box_collision_.boxs_info_, box_collision_.t_start, box_collision_.t_end,
        box_collision_.time_step, box_collision_.time_dealy,
        box_collision_.dec_request);
    if (obs.is_collision) {
      collision_obj_info_.valid_num++;
      collision_obj_info_.interest_obj_vec_.emplace_back(obs);
    }
  }
  collision_obj_info_.interest_obj_vec_.shrink_to_fit();
}
}  // namespace adas_function
