#include "planning_scheduler.h"

#include <common/config/basic_type.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <memory>

#include "adas_function/adaptive_cruise_control.h"
#include "adas_function/mrc_condition.h"
#include "adas_function/start_stop_enable.h"
#include "apa_function/util/apa_utils.h"
#include "basic_types.pb.h"
#include "common/config_context.h"
#include "config/basic_type.h"
#include "config/vehicle_param.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "func_state_machine_c.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "math/math_utils.h"
#include "planning_context.h"
#include "planning_debug_info.pb.h"
#include "planning_hmi_c.h"
#include "planning_plan_c.h"
#include "scene_type_config.pb.h"
#include "struct_container.hpp"
#include "utils/file.h"
#include "utils/lateral_utils.h"
#include "vehicle_config_context.h"
#include "vehicle_status.pb.h"
#include "virtual_lane.h"

namespace planning {

PlanningScheduler::PlanningScheduler(
    const LocalView* const local_view,
    const common::EngineConfiguration* const engine_config_ptr)
    : local_view_(local_view) {
  Init(engine_config_ptr);
}

PlanningScheduler::~PlanningScheduler() {}

void PlanningScheduler::Init(
    const common::EngineConfiguration* const engine_config_ptr) {
  session_.Init();
  environmental_model_manager_.Init(&session_);
  EnvironmentalModel* environmental_model =
      session_.mutable_environmental_model();
  environmental_model->feed_local_view(local_view_);
  // TODO: 车辆配置文件从文件读取

  VehicleConfigurationContext::Instance()->set_vehicle_param(
      engine_config_ptr->vehicle_cfg_dir);
  ILOG_DEBUG
      << "load vehicle param success! car type: "
      << VehicleConfigurationContext::Instance()->get_vehicle_param().car_type
      << ", the car wheel base is: "
      << VehicleConfigurationContext::Instance()
             ->get_vehicle_param()
             .wheel_base;

  // TODO：配置文件改成和场景/功能有关，不能使用默认场景
  planning::common::SceneType scene_type = session_.get_scene_type();
  auto config_builder =
      session_.environmental_model().config_builder(scene_type);
  config_ = config_builder->cast<GeneralPlanningConfig>();

  // TODO：移到Scc_Function
  InitSccFunction();

  hpp_function_ = std::make_unique<HppFunction>(&session_);
  noa_function_ = std::make_unique<NoaFunction>(&session_);
  scc_function_ = std::make_unique<SccFunction>(&session_);
  apa_function_ = std::make_unique<ApaFunction>(&session_);
  rads_function_ = std::make_unique<RadsFunction>(&session_);
  adas_function_ = std::make_unique<AdasFunction>(&session_);
  nsa_function_ = std::make_unique<NsaFunction>(&session_);
}

void PlanningScheduler::SyncParameters(planning::common::SceneType scene_type) {
  // std::string path;
  // auto engine_config =
  //     common::ConfigurationContext::Instance()->engine_config();
  // switch (scene_type) {
  //   case planning::common::SceneType::RADS:
  //     path = engine_config.module_cfg_dir +
  //     "/general_planner_module_rads.json"; break;
  //   case planning::common::SceneType::HIGHWAY:
  //     path =
  //         engine_config.module_cfg_dir +
  //         "/general_planner_module_highway.json";
  //     break;
  //   case planning::common::SceneType::PARKING_APA:
  //     path =
  //         engine_config.module_cfg_dir +
  //         "/general_planner_module_parking.json";
  //     break;
  //   case planning::common::SceneType::HPP:
  //     path = engine_config.module_cfg_dir +
  //     "/general_planner_module_hpp.json"; break;
  //   default:
  //     path =
  //         engine_config.module_cfg_dir +
  //         "/general_planner_module_highway.json";
  // }

  // std::string config_file = common::util::ReadFile(path);
  // auto config = mjson::Reader(config_file);

  auto config_builder =
      session_.environmental_model().config_builder(scene_type);
  config_ = config_builder->cast<GeneralPlanningConfig>();
  environmental_model_manager_.SetConfig(scene_type);
  // TODO: need to update rads param when switch to rads scene
  //  all parameters can be changed here
  GENERAL_PLANNING_CONTEXT.MutablePram().planner_type = config_.planner_type;
  // JSON_READ_VALUE(GENERAL_PLANNING_CONTEXT.MutablePram().planner_type, int,
  //                 "planner_type");
}

planning::common::SceneType PlanningScheduler::DetermineSceneType(
    const iflyauto::FuncStateMachine& func_state_machine) {
  auto scene_type = planning::common::SceneType::HIGHWAY;
  ILOG_INFO << "wahaha current state = " << func_state_machine.current_state;

  if (IsUndefinedScene(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::HIGHWAY;
  } else if (IsSwitchApaState(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::PARKING_APA;
  } else if (IsValidHppState(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::HPP;
  } else if (IsValidRadsState(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::RADS;
  } else if (IsValidNsaState(func_state_machine.current_state)) {
    scene_type = planning::common::SceneType::NSA;
  } else {
    scene_type = planning::common::SceneType::HIGHWAY;
  }

  session_.set_scene_type(scene_type);

  auto frame_info =
      DebugInfoManager::GetInstance().GetDebugInfoPb()->mutable_frame_info();
  frame_info->set_scene_type(common::SceneType_Name(scene_type));

  return scene_type;
}

bool PlanningScheduler::RunOnce(
    iflyauto::PlanningOutput* const planning_output,
    iflyauto::PlanningHMIOutputInfoStr* const planning_hmi_info) {
  ILOG_INFO << "PlanningScheduler::RunOnce";
  auto& planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  const double start_timestamp = IflyTime::Now_ms();
  bool planning_success = false;
  session_.mutable_planning_context()->Clear();
  session_.mutable_planning_context()->feed_planning_hmi_info(
      planning_hmi_info);

  const auto& state_machine = local_view_->function_state_machine_info;
  auto function_type = DetermineSceneType(state_machine);
  planning_result.scene_type = function_type;
  planning_result.timestamp = start_timestamp;
  planning_output->successful_slot_info_list_size = 0;
  planning_output->planning_status.apa_planning_status = iflyauto::APA_NONE;
  planning_output->planning_status.hpp_planning_status = iflyauto::HPP_UNKNOWN;
  planning_output->msg_meta.start_time =
      planning_result.timestamp * 1000.0;  // 临时使用该字段
  // reset
  if (function_type == common::PARKING_APA || function_type == common::HPP) {
    const auto& state_machine = local_view_->function_state_machine_info;
    if (GENERAL_PLANNING_CONTEXT.GetStatemachine().apa_reset_flag &&
        state_machine.current_state !=
            iflyauto::FunctionalState_PARK_GUIDANCE) {
      apa_function_->Reset();
      ILOG_INFO << "reset parking";

      ResetGLogFile();
    }
  }

  // adas_function step
  bool adas_function_sucess = false;
  adas_function_sucess = adas_function_->Plan();
  if (!adas_function_sucess) {
    ILOG_ERROR << "adas runonce failed !!!!";
    // return false;  //
    // TODO:这里有问题。会导致不运行FillPlanningTrajectory和FillPlanningHmiInfo。任何情况下都需要给输出赋值。
  }

  is_hpp_slot_searching_ = IsHppSlotSearchingByDistance();
  if (function_type == common::PARKING_APA || is_hpp_slot_searching_) {
    planning_success = ExcuteParkingFunction(
        function_type, start_timestamp, planning_output, planning_hmi_info);
  }

  if (function_type == common::HIGHWAY || function_type == common::HPP ||
      function_type == common::RADS || function_type == common::NSA) {
    planning_success = ExcuteNavigationFunction(
        function_type, start_timestamp, planning_output, planning_hmi_info);
    // can not active lcc/noa function if planning failed
    if (!planning_success) {
      planning_hmi_info->ad_info.is_avaliable = false;
    }
  }

  int64_t frame_duration = IflyTime::Now_ms() - start_timestamp;
  ILOG_INFO << "The time cost of RunOnce is: " << frame_duration;

  TimeBenchmark::Instance().SetTime(TimeBenchmarkType::TB_PLANNING_TOTAL,
                                    frame_duration);

  TimeBenchmark::Instance().DebugString();

  RecordTimeBenchmarkInfo();

  return planning_success;
}

uint64_t PlanningScheduler::FaultCode() { return session_.get_fault_code(); }

void PlanningScheduler::SetFaultCode(uint64_t faultcode) {
  return session_.set_fault_code(faultcode);
}

bool PlanningScheduler::FaultCanRecover() {
  using namespace framework;
  const auto& fault_counter_vec = session_.fault_counter_info();
  uint64_t fault_code = FaultCode();
  bool can_recover = false;
  switch (fault_code) {
    case 39000:
      if (fault_counter_vec[static_cast<int>(FaultType::PERCEPTION_TIME_OUT)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39001:
      if (fault_counter_vec[static_cast<int>(FaultType::LOCALIZATION_TIME_OUT)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39002:
      if (fault_counter_vec[static_cast<int>(FaultType::PREDICTION_TIME_OUT)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39003:
      if (fault_counter_vec[static_cast<int>(
                                FaultType::VEHICLE_SERVICE_TIME_OUT)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39004:
      if (fault_counter_vec[static_cast<int>(FaultType::FSM_TIME_OUT)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39006:
      if (fault_counter_vec[static_cast<int>(FaultType::TRAJ_LENGTH_RANGE)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39007:
      if (fault_counter_vec[static_cast<int>(FaultType::TRAJ_CURVATURE_RANGE)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39008:
      if (fault_counter_vec[static_cast<int>(
                                FaultType::TRAJ_LON_POS_CONSISTENCY)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39009:
      if (fault_counter_vec[static_cast<int>(
                                FaultType::TRAJ_LON_VEL_CONSISTENCY)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39010:
      if (fault_counter_vec[static_cast<int>(
                                FaultType::TRAJ_LON_ACC_CONSISTENCY)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39011:
      if (fault_counter_vec[static_cast<int>(
                                FaultType::TRAJ_LON_DEC_CONSISTENCY)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;
    case 39012:
      if (fault_counter_vec[static_cast<int>(
                                FaultType::TRAJ_LAT_POS_CONSISTENCY)]
              .fault_recovery_counter >= 3) {
        can_recover = true;
      }
      break;

    default:
      break;
  }
  return can_recover;
}

void PlanningScheduler::FillPlanningTrajectory(
    double start_time, iflyauto::PlanningOutput* const planning_output) {
  // 获取LDP&&ELK功能干预状态
  auto& GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  bool lkas_intervention_flag;
  if ((GetContext.get_output_info()
           ->ldp_output_info_.ldp_left_intervention_flag_ == true) ||
      (GetContext.get_output_info()
           ->ldp_output_info_.ldp_right_intervention_flag_ == true) ||
      (GetContext.get_output_info()
           ->elk_output_info_.elk_left_intervention_flag_ == true) ||
      (GetContext.get_output_info()
           ->elk_output_info_.elk_right_intervention_flag_ == true)) {
    lkas_intervention_flag = true;
  } else {
    lkas_intervention_flag = false;
  }

  // 获取计算结果
  const auto& lateral_output =
      session_.planning_context().lateral_behavior_planner_output();
  const auto& vision_only_longitudinal_outputs =
      session_.planning_context().vision_longitudinal_behavior_planner_output();
  const auto& planning_context = session_.planning_context();
  const auto& planning_result = planning_context.planning_result();
  const auto& ego_state =
      session_.environmental_model().get_ego_state_manager();
  const auto& function_info = session_.environmental_model().function_info();
  const bool active = session_.environmental_model().GetVehicleDbwStatus();
  auto virtual_lane_manager =
      session_.environmental_model().get_virtual_lane_manager();
  const auto& speed_limit_decider_output =
      session_.planning_context().speed_limit_decider_output();
  const auto lane_borrow_decider_output =
      session_.planning_context().lane_borrow_decider_output();
  // 更新输出
  iflyauto::strcpy_array(planning_output->meta.plan_strategy_name,
                         "Real Time Planning");

  // 2.Trajectory
  auto trajectory = &planning_output->trajectory;
  trajectory->trajectory_points_size = 0;
  // set trajectory false when dbw is false
  if (lkas_intervention_flag) {
    trajectory->available = true;
  } else {
    trajectory->available = active;
  }
  JSON_DEBUG_VALUE("traj_available", trajectory->available)

  // 根据定位有效性决定实时、长时
  auto location_valid = session_.environmental_model().location_valid();
  const auto &function_state_machine_info_ptr =
      session_.environmental_model()
          .get_local_view()
          .function_state_machine_info;
  if (location_valid) {
    trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;
    auto lkas_trajectory = GetContext.get_lka_trajectory_info();
    if (lkas_intervention_flag &&
        function_state_machine_info_ptr.current_state !=
            iflyauto::FunctionalState_ACC_ACTIVATE) {
      for (size_t i = 0; i < PLANNING_TRAJ_POINTS_MAX_NUM; i++) {
        auto path_point = &trajectory->trajectory_points[i];
        path_point->x = lkas_trajectory->trajectory_points[i].x;
        path_point->y = lkas_trajectory->trajectory_points[i].y;
        path_point->heading_yaw =
            lkas_trajectory->trajectory_points[i].heading_yaw;
        path_point->curvature = lkas_trajectory->trajectory_points[i].curvature;
        path_point->t = lkas_trajectory->trajectory_points[i].t;
        path_point->v = lkas_trajectory->trajectory_points[i].v;
        path_point->a = lkas_trajectory->trajectory_points[i].a;
        path_point->distance = lkas_trajectory->trajectory_points[i].distance;
        path_point->jerk = lkas_trajectory->trajectory_points[i].jerk;
        ++(trajectory->trajectory_points_size);
      }
    } else if (
        lkas_intervention_flag &&
        function_state_machine_info_ptr.current_state ==
            iflyauto::
                FunctionalState_ACC_ACTIVATE) {
      // 以下新增acc和ldp共同开启的工况

      //======将lkas目标轨迹线转到自车坐标系下,构建x_vec、y_vec、s_vec======//
      // 存储lkas的目标轨迹线散点(自车坐标系下) x坐标值
      std::vector<double> lkas_traj_dx_vec_;
      // 存储lkas的目标轨迹线散点(自车坐标系下) y坐标值
      std::vector<double> lkas_traj_dy_vec_;
      // 存储lkas的目标轨迹线散点(自车坐标系下) 起始点s值为0
      std::vector<double> lkas_traj_s_vec_;
      uint32 lkas_traj_enu_points_num = lkas_trajectory->trajectory_points_size;
      lkas_traj_dx_vec_.resize(lkas_traj_enu_points_num, 0);
      lkas_traj_dx_vec_.clear();
      lkas_traj_dx_vec_.reserve(lkas_traj_enu_points_num);
      lkas_traj_dy_vec_.resize(lkas_traj_enu_points_num, 0);
      lkas_traj_dy_vec_.clear();
      lkas_traj_dy_vec_.reserve(lkas_traj_enu_points_num);
      lkas_traj_s_vec_.resize(lkas_traj_enu_points_num, 0);
      lkas_traj_s_vec_.clear();
      lkas_traj_s_vec_.reserve(lkas_traj_enu_points_num);
      double s = 0.0;
      const auto rotm2d = GetContext.get_state_info()->rotm2d;
      for (uint32 i = 0; i < lkas_traj_enu_points_num; i++) {
        Eigen::Vector2d enu_pos_i(lkas_trajectory->trajectory_points[i].x,
                                  lkas_trajectory->trajectory_points[i].y);
        auto transformed_point =
            rotm2d.transpose() *
            (enu_pos_i - GetContext.get_state_info()->current_pos_i);
        lkas_traj_dx_vec_.emplace_back(transformed_point.x());
        lkas_traj_dy_vec_.emplace_back(transformed_point.y());

        if (i == 0) {
          lkas_traj_s_vec_.emplace_back(0.0);
        } else {
          const double ds =
              std::hypot(lkas_traj_dx_vec_[i] - lkas_traj_dx_vec_[i - 1],
                         lkas_traj_dy_vec_[i] - lkas_traj_dy_vec_[i - 1]);
          s += std::max(ds, 1e-3);
          lkas_traj_s_vec_.emplace_back(s);
        }
      }
      //======将lkas目标轨迹线转到自车坐标系下,构建dx_s_spline_dy_s_spline_======//
      pnc::mathlib::spline lkas_traj_dx_s_spline_;
      pnc::mathlib::spline lkas_traj_dy_s_spline_;
      lkas_traj_dx_s_spline_.set_points(lkas_traj_s_vec_, lkas_traj_dx_vec_);
      lkas_traj_dy_s_spline_.set_points(lkas_traj_s_vec_, lkas_traj_dy_vec_);
      //======根据ACC原始轨迹的v-t曲线,重新生成目标轨迹======//
      double s_proj = 0.0;
      Eigen::Vector2d current_pos = Eigen::Vector2d::Zero();
      adas_function::CalProjectionPointForLong(lkas_traj_dx_s_spline_,
                                               lkas_traj_dy_s_spline_, 0, s,
                                               current_pos, s_proj);
      double s_ref = s_proj;
      auto& car2local = GetContext.get_session()
                            ->environmental_model()
                            .get_ego_state_manager()
                            ->get_car2enu();
      for (size_t i = 0; i < planning_result.traj_points.size(); i++) {
        double plan_traj_dt = 0.025;

        if (i > 0) {
          s_ref +=
              std::max(1.0, planning_result.traj_points[i].v) * plan_traj_dt;
        }
        Eigen::Vector3d car_point, local_point;
        // 根据s查找位置
        car_point.x() = lkas_traj_dx_s_spline_(s_ref);
        car_point.y() = lkas_traj_dy_s_spline_(s_ref);
        car_point.z() = 0.0;
        //转到local坐标系下
        local_point = car2local * car_point;

        auto path_point = &trajectory->trajectory_points[i];
        path_point->x = local_point.x();  // 设定轨迹
        path_point->y = local_point.y();  // 设定轨迹
        path_point->heading_yaw = planning_result.traj_points[i].heading_angle;
        path_point->curvature = planning_result.traj_points[i].curvature;
        path_point->t = planning_result.traj_points[i].t;
        path_point->v = planning_result.traj_points[i].v;
        path_point->a = planning_result.traj_points[i].a;
        path_point->distance = planning_result.traj_points[i].s;
        path_point->jerk = planning_result.traj_points[i].jerk;
        ++(trajectory->trajectory_points_size);
      }
    }

    else {
      for (size_t i = 0; i < planning_result.traj_points.size(); i++) {
        auto path_point = &trajectory->trajectory_points[i];
        path_point->x = planning_result.traj_points[i].x;
        path_point->y = planning_result.traj_points[i].y;
        path_point->heading_yaw = planning_result.traj_points[i].heading_angle;
        path_point->curvature = planning_result.traj_points[i].curvature;
        path_point->t = planning_result.traj_points[i].t;
        path_point->v = planning_result.traj_points[i].v;
        path_point->a = planning_result.traj_points[i].a;
        path_point->distance = planning_result.traj_points[i].s;
        path_point->jerk = planning_result.traj_points[i].jerk;
        ++(trajectory->trajectory_points_size);
      }
    }

    // 设置参考线为default
    auto target_ref = &trajectory->target_reference;
    // add polynomial
    const auto& d_polynomial = lateral_output.d_poly;
    for (size_t i = 0; i < d_polynomial.size(); i++) {
      target_ref->polynomial[i] = d_polynomial[i];
    }
    target_ref->target_velocity =
        vision_only_longitudinal_outputs.velocity_target;
    auto acceleration_range_limit = &target_ref->acceleration_range_limit;
    acceleration_range_limit->min_a = -4.0;
    acceleration_range_limit->max_a = 4.0;

    target_ref->lateral_maneuver_gear = iflyauto::LATERAL_MANEUVER_GEAR_NORMAL;
  } else {
    // set vision_only_longitudinal_outputs if hdmpa valid is false
    trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TARGET_REFERENCE;
    // trajectory->mutable_trajectory_points()->Clear();
    // trajectory->mutable_target_reference()->Clear();
    // 设置轨迹为default
    auto path_point = &trajectory->trajectory_points[0];
    path_point->x = 0.0;
    path_point->y = 0.0;
    path_point->heading_yaw = 0.0;
    path_point->curvature = 0.0;
    path_point->t = 0.0;
    path_point->v = 0.0;
    path_point->a = 0.0;
    path_point->distance = 0.0;
    path_point->jerk = 0.0;
    ++(trajectory->trajectory_points_size);

    auto target_ref = &trajectory->target_reference;
    // add polynomial
    // clip the polynomial C_3
    const double lat_offset_rate = config_.d_poly_lat_offset_rate;
    const double max_lat_offset = config_.d_poly_max_lat_offset;
    const auto& d_polynomial = lateral_output.d_poly;
    double lat_offset_bound = max_lat_offset;
    static double limited_polynomial_3 = 0.0;

    if (d_polynomial.size() == 4) {
      if (lateral_output.lc_status == "left_lane_change" ||
          lateral_output.lc_status == "right_lane_change") {
        lat_offset_bound = ComputeBoundOfReferenceIntercept();

        if (std::fabs(d_polynomial[3]) > lat_offset_bound) {
          limited_polynomial_3 += planning_math::Clamp(
              d_polynomial[3], -lat_offset_rate, lat_offset_rate);
        } else {
          limited_polynomial_3 = planning_math::Clamp(
              d_polynomial[3], -lat_offset_bound, lat_offset_bound);
        }
      } else if ((lateral_output.lc_status == "left_lane_change_back" ||
                  lateral_output.lc_status == "right_lane_change_back") &&
                 std::fabs(d_polynomial[3]) >
                     config_.lc_back_consider_smooth_dpoly_thr) {
        limited_polynomial_3 =
            planning_math::Clamp(d_polynomial[3], -config_.lc_back_smooth_thr,
                                 config_.lc_back_smooth_thr);
      } else if (lateral_output.lc_status == "left_lane_change_wait" ||
                 lateral_output.lc_status == "right_lane_change_wait") {
        limited_polynomial_3 = 0.0;
      } else {
        limited_polynomial_3 = planning_math::Clamp(
            d_polynomial[3], -lat_offset_bound, lat_offset_bound);
      }
      limited_polynomial_3 = planning_math::Clamp(
          limited_polynomial_3, -lat_offset_bound, lat_offset_bound);
    }

    ILOG_DEBUG << "smooth dpoly enable_none_smooth: "
               << config_.enable_none_smooth
               << "   config_.none_consider_slope_thr:   "
               << config_.none_consider_slope_thr;

    ILOG_DEBUG << "limited_polynomial_3: " << limited_polynomial_3;
    std::vector<double> polynomial_limited(4);

    const auto& current_lane = session_.environmental_model()
                                   .get_virtual_lane_manager()
                                   ->get_current_lane();
    if ((lkas_intervention_flag == true) && (current_lane != nullptr)) {
      polynomial_limited[0] = current_lane->get_center_line()[3];
      polynomial_limited[1] = current_lane->get_center_line()[2];
      polynomial_limited[2] = current_lane->get_center_line()[1];
      polynomial_limited[3] = current_lane->get_center_line()[0];
    } else {
      // todo(ldh): temporally hack, need to be optimized
      polynomial_limited[0] = d_polynomial[0];
      polynomial_limited[1] = d_polynomial[1];
      polynomial_limited[2] = d_polynomial[2];
      polynomial_limited[3] = limited_polynomial_3;
    }
    ILOG_DEBUG << "limited_polynomial C0:" << polynomial_limited[0]
               << " C1:" << polynomial_limited[1]
               << " C2:" << polynomial_limited[2]
               << " C3:" << polynomial_limited[3];

    for (size_t i = 0; i < polynomial_limited.size(); i++) {
      target_ref->polynomial[i] = polynomial_limited[i];
    }

    target_ref->target_velocity =
        vision_only_longitudinal_outputs.velocity_target;

    auto acceleration_range_limit = &(target_ref->acceleration_range_limit);
    acceleration_range_limit->min_a =
        vision_only_longitudinal_outputs.a_target_min;
    acceleration_range_limit->max_a =
        vision_only_longitudinal_outputs.a_target_max;
    target_ref->lateral_maneuver_gear = iflyauto::LATERAL_MANEUVER_GEAR_NORMAL;
  }
  // 3.Turn signal
  auto turn_signal = &(planning_output->turn_signal_command);
  turn_signal->available = true;
  if (planning_result.turn_signal == NO_CHANGE) {
    turn_signal->turn_signal_value = iflyauto::TURN_SIGNAL_TYPE_NONE;
  } else if (planning_result.turn_signal == LEFT_CHANGE) {
    turn_signal->turn_signal_value = iflyauto::TURN_SIGNAL_TYPE_LEFT;
  } else {
    turn_signal->turn_signal_value = iflyauto::TURN_SIGNAL_TYPE_RIGHT;
  }

  if (session_.environmental_model().is_mrc_mode()) {
    turn_signal->turn_signal_value = iflyauto::TURN_SIGNAL_TYPE_EMERGENCY_FLASH;
  }

  // WB start:--------临时hack以下信号--------
  // 4.Light signal
  auto light_signal = &(planning_output->light_signal_command);
  light_signal->available = true;
  if (GetContext.get_output_info()->ihc_output_info_.ihc_request_ == true) {
    light_signal->light_signal_value = iflyauto::LIGHT_SIGNAL_TYPE_HIGH_BEAM;
  } else {
    light_signal->light_signal_value = iflyauto::LIGHT_SIGNAL_TYPE_NONE;
  }

  // 5.Horn signal
  auto horn_signal_command = &(planning_output->horn_signal_command);
  horn_signal_command->available = true;
  horn_signal_command->horn_signal_value = iflyauto::HORN_SIGNAL_TYPE_NONE;

  // 6.Gear signal
  auto gear_command = &(planning_output->gear_command);
  gear_command->available = true;
  // 需要获取目标挡位值
  gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_DRIVE;
  const auto& state_machine = local_view_->function_state_machine_info;
  const auto rads_scene_is_completed = session_.planning_context()
                                           .start_stop_decider_output()
                                           .rads_scene_is_completed();
  if (session_.is_rads_scene()) {
    if (state_machine.current_state == iflyauto::FunctionalState_RADS_TRACING &&
        !rads_scene_is_completed) {
      gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_REVERSE;
    } else if (state_machine.current_state ==
                   iflyauto::FunctionalState_RADS_TRACING &&
               rads_scene_is_completed) {
      gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_PARKING;
    } else {
      gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_NONE;
    }
  }

  // 7.Open loop steering command
  auto open_loop_steering_command =
      &(planning_output->open_loop_steering_command);
  open_loop_steering_command->available = true;
  open_loop_steering_command->jerk_factor = 70.0;  // hack
  open_loop_steering_command->need_steering_wheel_stationary = false;
  open_loop_steering_command->steering_wheel_rad_limit = 0.1;

  // 8.Planning status
  auto planning_status = &(planning_output->planning_status);
  planning_status->standstill = false;
  if (function_info.function_mode() == common::DrivingFunctionInfo::ACC ||
      function_info.function_mode() == common::DrivingFunctionInfo::SCC) {
    planning_status->standstill = std::fabs(ego_state->ego_v()) < 0.1;
  }
  // 启停状态机
  planning_status->ready_to_go = planning_context.start_stop_result().state() ==
                                 common::StartStopInfo::START;
  planning_status->apa_planning_status = iflyauto::APA_NONE;
  // WB end:--------临时hack以上信号--------
  const bool planning_success = planning_context.planning_success();
  const auto &running_mode = state_machine.running_mode;
  const auto scene_type = session_.get_scene_type();
  if(running_mode == iflyauto::RunningMode::RUNNING_MODE_MEMORY_PARKING) {
    const bool hpp_cruise_routing_completed =
        planning_context.hpp_cruise_routing_completed();
    const bool target_slot_allowed_to_park =
        planning_context.target_slot_allowed_to_park();
    const bool timeout_for_target_slot_allowed_to_park =
        planning_context.timeout_for_target_slot_allowed_to_park();
    // TODO(taolu10): define HPP_PARKING_COMPLETED status
    if (hpp_cruise_routing_completed && (target_slot_allowed_to_park || timeout_for_target_slot_allowed_to_park)) {
      if (target_slot_allowed_to_park) {
        planning_status->hpp_planning_status = iflyauto::HPP_ROUTING_COMPLETED;
      } else if (timeout_for_target_slot_allowed_to_park) {
        planning_status->hpp_planning_status = iflyauto::HPP_ROUTING_PLANNING_FAILED;
      }
    } else if (planning_success) {
      planning_status->hpp_planning_status = iflyauto::HPP_RUNNING;
    } else {
      if (scene_type == common::SceneType::HPP) {
        planning_status->hpp_planning_status = iflyauto::HPP_ROUTING_PLANNING_FAILED;
      } else {  // HPP_PARKING
        planning_status->hpp_planning_status = iflyauto::HPP_PARKING_PLANNING_FAILED;
      }
    }
  } else if (running_mode == iflyauto::RunningMode::RUNNING_MODE_REVERSE_FOLLOW_TRACE) {
    const bool rads_planning_completed = planning_context.rads_planning_completed();
    if (rads_planning_completed) {
      planning_status->rads_planning_status = iflyauto::RADS_COMPLETED;
    } else if (planning_success) {
      planning_status->rads_planning_status = iflyauto::RADS_RUNNING;
    } else {
      planning_status->rads_planning_status = iflyauto::RADS_RUNNING_FAILED;
    }
  }

  if (session_.is_nsa_scene()) {
    const double nsa_ego_stop_vel_thred = 0.1;
    const auto nsa_is_completed = session_.planning_context().nsa_planning_completed();
    if (((state_machine.current_state == iflyauto::FunctionalState_NRA_GUIDANCE &&
         nsa_is_completed) || state_machine.current_state == iflyauto::FunctionalState_NRA_COMPLETED ) &&
        ego_state->ego_v() < nsa_ego_stop_vel_thred) {
      gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_PARKING;
    }
    if (nsa_is_completed) {
      planning_status->nsa_planning_status = iflyauto::NSA_COMPLETED;
    } else if (planning_success) {
      planning_status->nsa_planning_status = iflyauto::NSA_RUNNING;
    } else {
      planning_status->nsa_planning_status = iflyauto::NSA_RUNNING_FAILED;
    }
  }
  JSON_DEBUG_VALUE(
    "gear_command",
    static_cast<int>(gear_command->gear_command_value));
  // planning request
  // 绕行接管
  if (lane_borrow_decider_output.takeover_prompt) {
    planning_output->planning_request.take_over_req_level =
        iflyauto::REQUEST_LEVEL_MILD;
    planning_output->planning_request.request_reason =
        iflyauto::REQUEST_REASON_BORROW_FAILED;
  } else {
    planning_output->planning_request.take_over_req_level =
        iflyauto::RequestLevel::REQUEST_LEVEL_NO_REQ;
    planning_output->planning_request.request_reason =
        iflyauto::RequestReason::REQUEST_REASON_NO_REASON;
  }

  if (speed_limit_decider_output.is_function_fading_away() &&
      config_.left_right_turn_func_fading_away_switch) {
    planning_output->planning_request.take_over_req_level =
        iflyauto::RequestLevel::REQUEST_LEVEL_MILD;
    planning_output->planning_request.request_reason =
        speed_limit_decider_output.request_reason();
  } else if (speed_limit_decider_output.function_inhibited_near_roundabout()) {
    planning_output->planning_request.take_over_req_level =
        iflyauto::RequestLevel::REQUEST_LEVEL_MILD;
    planning_output->planning_request.request_reason =
        iflyauto::RequestReason::REQUEST_REASON_ON_ROUNDABOUT;
  } else {
    planning_output->planning_request.take_over_req_level =
        iflyauto::RequestLevel::REQUEST_LEVEL_NO_REQ;
    planning_output->planning_request.request_reason =
        iflyauto::RequestReason::REQUEST_REASON_NO_REASON;
  }
  JSON_DEBUG_VALUE(
      "take_over_request",
      static_cast<int>(planning_output->planning_request.take_over_req_level));
  JSON_DEBUG_VALUE(
      "request_reason",
      static_cast<int>(planning_output->planning_request.request_reason));
}

void PlanningScheduler::GenerateStopTrajectory(
    double start_time, iflyauto::PlanningOutput* const planning_output) {
  // 更新输出
  // planning_output->msg_header.stamp = IflyTime::Now_ms();

  auto trajectory = &(planning_output->trajectory);
  // Hack: 长时规划
  trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;
  trajectory->trajectory_points_size = 0;
  double t = 0.0;
  for (size_t i = 0; i < 21; i++) {
    t = 0.1 * i;
    auto path_point =
        &(trajectory->trajectory_points[trajectory->trajectory_points_size++]);
    path_point->x = 0.0;
    path_point->y = 0.0;
    path_point->heading_yaw = 0.0;
    path_point->curvature = 0.0;
    path_point->t = t;
    path_point->v = 0.0;
    path_point->a = 0.0;
    path_point->distance = 0.0;
    path_point->jerk = 0.0;  // TBD
  }
}

void PlanningScheduler::FillPlanningHmiInfo(
    double start_timestamp,
    iflyauto::PlanningHMIOutputInfoStr* const planning_hmi_info) {
  const auto& lateral_output =
      session_.planning_context().lateral_behavior_planner_output();
  const auto& lane_change_decider_output =
      session_.planning_context().lane_change_decider_output();
  const auto& lat_offset_decider_output =
      session_.planning_context().lateral_offset_decider_output();
  const auto &agent_longitudinal_decider_output =
      session_.planning_context().agent_longitudinal_decider_output();

  planning_hmi_info->msg_header.stamp = IflyTime::Now_us();
  // HMI for alc
  auto alc_output_pb = &(planning_hmi_info->alc_output_info);

  iflyauto::strcpy_array(alc_output_pb->lc_request,
                         lateral_output.lc_request.c_str());

  iflyauto::strcpy_array(alc_output_pb->lc_status,
                         lateral_output.lc_status.c_str());

  iflyauto::strcpy_array(alc_output_pb->lc_invalid_reason,
                         lane_change_decider_output.lc_invalid_reason.c_str());

  iflyauto::strcpy_array(
      alc_output_pb->lc_back_reason,
      lane_change_decider_output.lc_back_invalid_reason.c_str());

  // HMI for CIPV
  // TBD: 后续需要丰富障碍物的信息，后车、侧方车辆等
  const auto& cipv_info =
      session_.planning_context().planning_hmi_info().cipv_info;
  planning_hmi_info->cipv_info.has_cipv = cipv_info.has_cipv;
  planning_hmi_info->cipv_info.cipv_id = cipv_info.cipv_id;

  // HMI for ad_info
  const auto& ad_info = session_.planning_context().planning_hmi_info().ad_info;
  // available
  planning_hmi_info->ad_info.is_avaliable = ad_info.is_avaliable;
  planning_hmi_info->ad_info.cruise_speed = ad_info.cruise_speed;
  planning_hmi_info->ad_info.lane_change_direction =
      ad_info.lane_change_direction;
  planning_hmi_info->ad_info.lane_change_status = ad_info.lane_change_status;
  planning_hmi_info->ad_info.status_update_reason =
      ad_info.status_update_reason;
  planning_hmi_info->ad_info.obstacle_info[0] = ad_info.obstacle_info[0];
  planning_hmi_info->ad_info.obstacle_info_size = ad_info.obstacle_info_size;
  planning_hmi_info->ad_info.lane_change_reason = ad_info.lane_change_reason;
  planning_hmi_info->ad_info.is_curva = ad_info.is_curva;
  planning_hmi_info->ad_info.intersection_pass_sts =
      ad_info.intersection_pass_sts;
  planning_hmi_info->ad_info.intersection_state = ad_info.intersection_state;

  planning_hmi_info->ad_info.distance_to_ramp = ad_info.distance_to_ramp;
  planning_hmi_info->ad_info.distance_to_split = ad_info.distance_to_split;
  planning_hmi_info->ad_info.distance_to_merge = ad_info.distance_to_merge;
  planning_hmi_info->ad_info.split_select_direction = ad_info.split_select_direction;
  planning_hmi_info->ad_info.distance_to_toll_station =
      ad_info.distance_to_toll_station;
  planning_hmi_info->ad_info.noa_exit_warning_level_distance =
      ad_info.noa_exit_warning_level_distance;
  // planning_hmi_info->ad_info.distance_to_tunnel = ;
  // planning_hmi_info->ad_info.is_within_hdmap = ;
  planning_hmi_info->ad_info.ramp_direction = ad_info.ramp_direction;
  planning_hmi_info->ad_info.dis_to_reference_line =
      ad_info.dis_to_reference_line;
  planning_hmi_info->ad_info.angle_to_roaddirection =
      ad_info.angle_to_roaddirection;
  planning_hmi_info->ad_info.is_in_sdmaproad = ad_info.is_in_sdmaproad;
  planning_hmi_info->ad_info.road_type = ad_info.road_type;
  planning_hmi_info->ad_info.ramp_pass_sts = ad_info.ramp_pass_sts;
  planning_hmi_info->ad_info.landing_point = ad_info.landing_point;
  // lane borrow
  planning_hmi_info->ad_info.start_nudging = ad_info.start_nudging;
  planning_hmi_info->ad_info.borrow_lane_type = ad_info.borrow_lane_type;
  planning_hmi_info->ad_info.borrow_direction = ad_info.borrow_direction;


  planning_hmi_info->ad_info.reference_line_msg =
      session_.planning_context()
          .planning_hmi_info()
          .ad_info.reference_line_msg;
  planning_hmi_info->ad_info.timestamp = local_view_->road_info.isp_timestamp;
  planning_hmi_info->ad_info.cone_warning_info.cone_warning = ad_info.cone_warning_info.cone_warning;
  planning_hmi_info->ad_info.construction_info.construction_state = ad_info.construction_info.construction_state;

  const auto& cutin_ttc_info =
      agent_longitudinal_decider_output.closest_cutin_ttc_info;
  planning_hmi_info->ad_info.cutin_track_id = cutin_ttc_info.agent_id;
  planning_hmi_info->ad_info.cutin_ttc = static_cast<float>(cutin_ttc_info.ttc);

  // HMI for hpp
  const bool is_reached_target_slot = session_.environmental_model()
                                          .get_parking_slot_manager()
                                          ->IsReachedTargetSlot();
  const auto& ego_state_manager =
      session_.environmental_model().get_ego_state_manager();
  const auto& route_info_output =
      session_.environmental_model().get_route_info()->get_route_info_output();
  const auto& planning_context = session_.planning_context();
  auto hpp_info = &(session_.mutable_planning_context()
                        ->mutable_planning_hmi_info()
                        ->hpp_info);
  hpp_info->is_avaliable = route_info_output.hpp_route_info_output.is_on_hpp_lane;
  hpp_info->distance_to_parking_space =
      is_reached_target_slot ? 0.0 : route_info_output.hpp_route_info_output.distance_to_target_slot;
  hpp_info->is_on_hpp_lane = route_info_output.hpp_route_info_output.is_on_hpp_lane;
  // hpp_info->is_on_hpp_lane = true;  // hack
  hpp_info->is_reached_hpp_trace_start =
      route_info_output.hpp_route_info_output.is_reached_hpp_start_point;
  hpp_info->accumulated_driving_distance =
      route_info_output.hpp_route_info_output.sum_distance_driving;

  hpp_info->is_approaching_intersection = false;
  hpp_info->is_approaching_turn = false;
  hpp_info->is_target_parking_space_occupied = false;
  hpp_info->is_new_parking_space_found = false;
  hpp_info->hpp_state_switch = iflyauto::HPPStateSwitch::HPP_NONE;
  auto reference_path_manager =
      session_.environmental_model().get_reference_path_manager();
  auto current_reference_path =
      reference_path_manager->get_reference_path_by_current_lane();
  const double kCheckTurnDistance = 15.0;
  const double kEgoIsOnTurnDistance1 = -3.0;
  const double kEgoIsOnTurnDistance2 = 5.0;
  auto& frenet_ego_state = current_reference_path->get_frenet_ego_state();
  auto& points = current_reference_path->get_points();
  double ego_s = frenet_ego_state.s();
  if (current_reference_path != nullptr) {
    for (auto& point : points) {
      double distance = point.path_point.s() - ego_s;
      if (distance > kEgoIsOnTurnDistance1 &&
          distance < kEgoIsOnTurnDistance2 &&
          point.path_point.kappa() > 0.08) {  // ego is on the curve
        break;
      }
      if (distance > kEgoIsOnTurnDistance2 && distance <= kCheckTurnDistance) {
        if (point.path_point.kappa() >
            0.1) {  // 关注实际曲率的连续性，考虑多点还是单点
          // hpp_info->set_is_approaching_intersection(true);
          hpp_info->is_approaching_turn = true;
          break;
        }
      } else if (distance > kCheckTurnDistance) {
        break;
      }
    }
  }
  // if (route_info_output.distance_to_target_slot < 10.0) {
  //   hpp_info->distance_to_parking_space =
  //       std::min(std::fabs(points.back().path_point.s() - ego_s),
  //                route_info_output.distance_to_target_slot);
  // }
  // hpp状态切park_in状态
  if (session_.is_hpp_scene()) {
    const auto& parking_switch_info = session_.planning_context()
                                          .parking_switch_decider_output()
                                          .parking_switch_info;
    if (parking_switch_info.is_target_slot_allowed_to_park) {
      hpp_info->hpp_state_switch =
          iflyauto::HPPStateSwitch::HPP_CRUISING_TO_PARKING;
    } else if (parking_switch_info.is_target_slot_occupied) {
      hpp_info->is_target_parking_space_occupied = true;
    } else if (parking_switch_info.is_selected_slot_allowed_to_park) {
      hpp_info->hpp_state_switch =
          iflyauto::HPPStateSwitch::HPP_CRUISING_TO_PARKING;
    }

    const bool timeout_for_target_slot_allowed_to_park = planning_context.timeout_for_target_slot_allowed_to_park();
    const bool hpp_cruise_routing_completed = planning_context.hpp_cruise_routing_completed();
    if(hpp_cruise_routing_completed && timeout_for_target_slot_allowed_to_park) {
      hpp_info->hpp_planning_failed_reason = iflyauto::HPPPlanningFailedReason::
          HPP_PLANNING_FAILED_REASON_TARGET_PARKING_SPACE_OCCUPIED;
    }
    // todo: is_new_parking_space_found is unused.
    if (parking_switch_info.has_parking_slot_in_hpp_searching) {
      hpp_info->is_new_parking_space_found = true;
    }
  }

  return;
}

void PlanningScheduler::FillAdasPlanningHmiInfo(
    double start_timestamp,
    iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info) {
  planning_hmi_info->msg_header.stamp = IflyTime::Now_us();

  /*new adas*/
  auto &GetContext = adas_function::context::AdasFunctionContext::GetInstance();
  planning_hmi_info->ldw_output_info.ldw_state =
      GetContext.get_output_info()->ldw_output_info_.ldw_state_;
  planning_hmi_info->ldw_output_info.ldw_left_warning =
      GetContext.get_output_info()->ldw_output_info_.ldw_left_warning_;
  planning_hmi_info->ldw_output_info.ldw_right_warning =
      GetContext.get_output_info()->ldw_output_info_.ldw_right_warning_;
  // HMI for ldp
  planning_hmi_info->ldp_output_info.ldp_state =
      GetContext.get_output_info()->ldp_output_info_.ldp_state_;
  planning_hmi_info->ldp_output_info.ldp_left_intervention_flag =
      GetContext.get_output_info()
          ->ldp_output_info_.ldp_left_intervention_flag_;
  planning_hmi_info->ldp_output_info.ldp_right_intervention_flag =
      GetContext.get_output_info()
          ->ldp_output_info_.ldp_right_intervention_flag_;
  planning_hmi_info->ldp_output_info.ldp_warning_audio_flag =
      GetContext.get_output_info()->ldp_output_info_.ldp_warning_audio_flag_;
  planning_hmi_info->ldp_output_info.ldp_driver_handsoff_warning =
      GetContext.get_output_info()
          ->ldp_output_info_.ldp_driver_handsoff_warning_;
  // HMI for elk
  planning_hmi_info->elk_output_info.elk_state =
      GetContext.get_output_info()->elk_output_info_.elk_state_;
  planning_hmi_info->elk_output_info.elk_left_intervention_flag =
      GetContext.get_output_info()
          ->elk_output_info_.elk_left_intervention_flag_;
  planning_hmi_info->elk_output_info.elk_right_intervention_flag =
      GetContext.get_output_info()
          ->elk_output_info_.elk_right_intervention_flag_;

  planning_hmi_info->elk_output_info.elk_risk_obj.obj_valid =
      GetContext.get_output_info()->elk_output_info_.elk_risk_obj_.obj_valid;
  planning_hmi_info->elk_output_info.elk_risk_obj.id =
      GetContext.get_output_info()->elk_output_info_.elk_risk_obj_.id;

  // HMI for ihc

  planning_hmi_info->ihc_output_info.ihc_state =
      GetContext.get_output_info()->ihc_output_info_.ihc_state_;
  planning_hmi_info->ihc_output_info.ihc_request =
      GetContext.get_output_info()->ihc_output_info_.ihc_request_;
  planning_hmi_info->ihc_output_info.ihc_request_status =
      GetContext.get_output_info()->ihc_output_info_.ihc_request_status_;
  // HMI for tsr
  planning_hmi_info->tsr_output_info.tsr_state =
      GetContext.get_output_info()->tsr_output_info_.tsr_state_;
  planning_hmi_info->tsr_output_info.tsr_warning =
      GetContext.get_output_info()->tsr_output_info_.tsr_warning_;
  planning_hmi_info->tsr_output_info.tsr_speed_unlimit_warning =
      GetContext.get_output_info()->tsr_output_info_.isli_display_type_;
  planning_hmi_info->tsr_output_info.tsr_speed_limit =
      GetContext.get_output_info()->tsr_output_info_.tsr_speed_limit_;
  planning_hmi_info->tsr_output_info.tsr_supp_sign_type =
      GetContext.get_output_info()->tsr_output_info_.supp_sign_type;

  // HMI for meb
  planning_hmi_info->meb_output_info.meb_state =
      GetContext.get_output_info()->meb_output_info_.meb_state;
  planning_hmi_info->meb_output_info.meb_request_status =
      GetContext.get_output_info()->meb_output_info_.meb_request_status;
  planning_hmi_info->meb_output_info.meb_request_value =
      GetContext.get_output_info()->meb_output_info_.meb_request_value;
  planning_hmi_info->meb_output_info.meb_request_direction =
      GetContext.get_output_info()->meb_output_info_.meb_request_direction;
  // HMI for amap
  planning_hmi_info->amap_output_info.amap_state =
      GetContext.get_output_info()->amap_output_info_.amap_state;
  planning_hmi_info->amap_output_info.amap_request_flag =
      GetContext.get_output_info()->amap_output_info_.amap_request_flag;
  planning_hmi_info->amap_output_info.amap_trq_limit_max =
      GetContext.get_output_info()->amap_output_info_.amap_trq_limit_max;

  JSON_DEBUG_VALUE("planning_hmi_ldw_state",
                   (int)planning_hmi_info->ldw_output_info.ldw_state);
  JSON_DEBUG_VALUE("planning_hmi_ldp_state",
                   (int)planning_hmi_info->ldp_output_info.ldp_state);
  JSON_DEBUG_VALUE("planning_hmi_elk_state",
                   (int)planning_hmi_info->elk_output_info.elk_state);

  return;
}

void PlanningScheduler::FillPlanningRequest(
    iflyauto::RequestLevel request,
    iflyauto::PlanningOutput* const planning_output) {
  planning_output->planning_request.take_over_req_level = request;
  planning_output->planning_request.request_reason =
      iflyauto::REQUEST_REASON_NO_REASON;
}

void PlanningScheduler::ClearParkingInfo(
    iflyauto::PlanningOutput* const planning_output,
    iflyauto::PlanningHMIOutputInfoStr* const planning_hmi_info) {
  session_.mutable_planning_context()
      ->mutable_planning_output()
      .planning_status.apa_planning_status = iflyauto::APA_NONE;

  session_.mutable_planning_context()
      ->mutable_planning_output()
      .successful_slot_info_list_size = 0;

  planning_output->planning_status.apa_planning_status = iflyauto::APA_NONE;

  memset(&planning_hmi_info->apa_info, 0, sizeof(planning_hmi_info->apa_info));
}

bool PlanningScheduler::IsUndefinedScene(
    const iflyauto::FunctionalState &current_state) {
  return current_state == iflyauto::FunctionalState_MANUAL_DRIVING ||
         current_state == iflyauto::FunctionalState_SYSTEM_ERROR ||
         current_state == iflyauto::FunctionalState_MRC;
}

bool PlanningScheduler::IsValidHppState(
    const iflyauto::FunctionalState& current_state) {
  return current_state >= iflyauto::FunctionalState_HPP_STANDBY &&
         current_state <= iflyauto::FunctionalState_HPP_ERROR;
}

bool PlanningScheduler::IsValidRadsState(
    const iflyauto::FunctionalState& current_state) {
  return current_state == iflyauto::FunctionalState_RADS_STANDBY ||
         current_state == iflyauto::FunctionalState_RADS_PRE_ACTIVE ||
         current_state == iflyauto::FunctionalState_RADS_TRACING ||
         current_state == iflyauto::FunctionalState_RADS_SUSPEND ||
         current_state == iflyauto::FunctionalState_RADS_COMPLETE;
}

bool PlanningScheduler::IsValidNsaState(
    const iflyauto::FunctionalState &current_state) {
  return current_state >= iflyauto::FunctionalState_NRA_PASSIVE &&
         current_state <= iflyauto::FunctionalState_NRA_ERROR;
}

void PlanningScheduler::InitSccFunction() {
  // TODO：配置文件改成和场景/功能有关，不能使用默认场景
  planning::common::SceneType scene_type = session_.get_scene_type();
  auto config_builder =
      session_.environmental_model().config_builder(scene_type);

  // SCC Function
  auto adaptive_cruise_control =
      std::make_shared<AdaptiveCruiseControl>(config_builder, &session_);
  session_.mutable_planning_context()->set_adaptive_cruise_control_function(
      adaptive_cruise_control);

  auto start_stop =
      std::make_shared<StartStopEnable>(config_builder, &session_);
  session_.mutable_planning_context()->set_start_stop_enable(start_stop);

  auto mrc_condition =
      std::make_shared<MrcCondition>(config_builder, &session_);
  session_.mutable_planning_context()->set_mrc_condition(mrc_condition);
}

void PlanningScheduler::interpolate_with_last_trajectory_points() {
  const auto& last_planning_result =
      session_.planning_context().last_planning_result();

  // interpolate traj points
  // todo @xbliu config

  auto& planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  auto start_time =
      (planning_result.timestamp - last_planning_result.timestamp) / 1000.0;
  assert(start_time >= 0);
  planning_result.traj_points.clear();
  auto backup_num_points = 201;
  auto delta_time = 0.025;
  auto& last_traj_points = last_planning_result.traj_points;
  assert(last_traj_points.size() >= 2);

  size_t idx = 0;
  for (int j = 0; j < backup_num_points; ++j) {
    TrajectoryPoint traj_pt;
    auto t = j * delta_time;
    auto interpolate_t = t + start_time;
    for (; idx < last_traj_points.size() - 1; idx++) {
      if (last_traj_points[idx].t <= interpolate_t &&
          interpolate_t <= last_traj_points[idx + 1].t) {
        break;
      }
    }

    auto pre_idx = std::min(idx, last_traj_points.size() - 2);
    auto& pre_pt = last_traj_points[pre_idx];
    auto& next_pt = last_traj_points[pre_idx + 1];

    traj_pt.t = t;
    traj_pt.x = planning_math::Interpolate(pre_pt.t, pre_pt.x, next_pt.t,
                                           next_pt.x, interpolate_t);
    traj_pt.y = planning_math::Interpolate(pre_pt.t, pre_pt.y, next_pt.t,
                                           next_pt.y, interpolate_t);
    traj_pt.heading_angle = planning_math::InterpolateAngle(
        pre_pt.t, pre_pt.heading_angle, next_pt.t, next_pt.heading_angle,
        interpolate_t);
    traj_pt.curvature =
        planning_math::Interpolate(pre_pt.t, pre_pt.curvature, next_pt.t,
                                   next_pt.curvature, interpolate_t);
    traj_pt.v = planning_math::Interpolate(pre_pt.t, pre_pt.v, next_pt.t,
                                           next_pt.v, interpolate_t);
    traj_pt.a = planning_math::Interpolate(pre_pt.t, pre_pt.a, next_pt.t,
                                           next_pt.a, interpolate_t);
    traj_pt.s = planning_math::Interpolate(pre_pt.t, pre_pt.s, next_pt.t,
                                           next_pt.s, interpolate_t);
    traj_pt.l = planning_math::Interpolate(pre_pt.t, pre_pt.l, next_pt.t,
                                           next_pt.l, interpolate_t);
    traj_pt.frenet_valid = pre_pt.frenet_valid && next_pt.frenet_valid;
    planning_result.traj_points.emplace_back(traj_pt);
  }
  planning_result.raw_traj_points.clear();
}

bool PlanningScheduler::UpdateFailedPlanningResult() {
  const auto& coarse_planning_info = session_.planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto& last_planning_result =
      session_.planning_context().last_planning_result();
  auto& planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  if (last_planning_result.scene_type != planning_result.scene_type) {
    return false;
  }

  if (last_planning_result.target_lane_id ==
          coarse_planning_info.target_lane_id &&
      last_planning_result.use_backup_cnt <= config_.failure_counter_thrshld) {
    auto delta_time =
        (planning_result.timestamp - last_planning_result.timestamp) / 1000.0;
    if (0.0 < delta_time && delta_time < 1.0) {
      interpolate_with_last_trajectory_points();
      planning_result.use_backup_cnt = last_planning_result.use_backup_cnt + 1;
      return true;
    }
  }

  return false;
}

bool PlanningScheduler::UpdateSuccessfulPlanningResult() {
  const auto& coarse_planning_info = session_.planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  auto& planning_result =
      session_.mutable_planning_context()->mutable_planning_result();
  planning_result.use_backup_cnt = 0;
  planning_result.target_lane_id = coarse_planning_info.target_lane_id;
  session_.mutable_planning_context()->mutable_last_planning_result() =
      planning_result;

  return true;
}

double PlanningScheduler::ComputeBoundOfReferenceIntercept() {
  int origin_lane_virtual_id = session_.planning_context()
                                   .lane_change_decider_output()
                                   .origin_lane_virtual_id;
  int target_lane_virtual_id = session_.planning_context()
                                   .lane_change_decider_output()
                                   .target_lane_virtual_id;
  auto target_reference =
      session_.environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane_virtual_id, false);
  auto origin_reference =
      session_.environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_virtual_id, false);
  const double max_lat_offset = config_.d_poly_max_lat_offset;
  const double min_lat_offset = config_.d_poly_min_lat_offset;
  double intercept_cart_point = 0.0;
  double intercept_presee_cart_point = 0.0;
  double presee_dist = 25;
  double reference_intercept_bound = max_lat_offset;
  ILOG_DEBUG << "";

  if (origin_reference != nullptr && target_reference != nullptr) {
    Point2D frenet_of_cart_point_in_target;
    Point2D cart_point_in_target;
    Point2D presee_cart_point_in_target;
    Point2D frenet_of_presee_cart_point_in_target;

    if (target_reference->get_frenet_coord()->SLToXY(
            Point2D(target_reference->get_frenet_ego_state().s(), 0),
            cart_point_in_target)) {
      if (origin_reference->get_frenet_coord()->XYToSL(
              cart_point_in_target, frenet_of_cart_point_in_target)) {
        intercept_cart_point = frenet_of_cart_point_in_target.y;
      }
    }

    double s_end =
        std::min(target_reference->get_frenet_coord()->Length(),
                 target_reference->get_frenet_ego_state().s() + presee_dist);
    if (target_reference->get_frenet_coord()->SLToXY(
            Point2D(s_end, 0), presee_cart_point_in_target)) {
      if (origin_reference->get_frenet_coord()->XYToSL(
              presee_cart_point_in_target,
              frenet_of_presee_cart_point_in_target)) {
        intercept_presee_cart_point = frenet_of_presee_cart_point_in_target.y;
      }
    }
  }
  ILOG_DEBUG << "intercept_presee_cart_point: " << intercept_presee_cart_point
             << "intercept_cart_point: " << intercept_cart_point;

  if (std::fabs(intercept_presee_cart_point) >
      std::fabs(intercept_cart_point)) {
    reference_intercept_bound =
        max_lat_offset -
        std::fabs(intercept_presee_cart_point - intercept_cart_point);
    reference_intercept_bound = planning_math::Clamp(
        reference_intercept_bound, min_lat_offset, max_lat_offset);
    ILOG_DEBUG << "lat_offset_bound: " << reference_intercept_bound;
  }

  return reference_intercept_bound;
}

bool PlanningScheduler::IsHppSlotSearchingByDistance() {
  // check state
  const auto& state_machine = local_view_->function_state_machine_info;
  if (!IsHppSlotSearchingStage(state_machine.current_state)) {
    return false;
  }

  // check dist
  if (state_machine.current_state ==
      iflyauto::FunctionalState_HPP_CRUISE_ROUTING) {
    double dist = session_.environmental_model()
                      .get_route_info()
                      ->get_route_info_output()
                      .hpp_route_info_output.distance_to_target_dest;
    const double kdistance_thresh = 20.0;
    if (dist > kdistance_thresh) {
      return false;
    }
  }

  //  check speed
  const auto& ego_state =
      session_.environmental_model().get_ego_state_manager();
  const double kspeed_thresh = 5.0;
  if (ego_state->ego_v() > kspeed_thresh) {
    return false;
  }

  return true;
}

const bool PlanningScheduler::ExcuteParkingFunction(
    const common::SceneType function_type, const double start_timestamp,
    iflyauto::PlanningOutput *const planning_output,
    iflyauto::PlanningHMIOutputInfoStr *const planning_hmi_info) {
  // 泊车规划部分
  bool planning_success = apa_function_->Plan();

  iflyauto::MsgHeader msg_header = planning_output->msg_header;
  iflyauto::MsgMeta msg_meta = planning_output->msg_meta;
  iflyauto::PlanMeta meta = planning_output->meta;
  *planning_output = session_.planning_context().planning_output();
  planning_output->msg_header = msg_header;
  planning_output->msg_meta = msg_meta;
  planning_output->meta = meta;
  FillAdasPlanningHmiInfo(start_timestamp, planning_hmi_info);
  session_.mutable_planning_context()->mutable_planning_success() = planning_success;
  return planning_success;
}

const bool PlanningScheduler::ExcuteNavigationFunction(
    const common::SceneType function_type, const double start_timestamp,
    iflyauto::PlanningOutput* const planning_output,
    iflyauto::PlanningHMIOutputInfoStr* const planning_hmi_info) {
  // 行车规划部分
  // TODO(xjli32): 功能切换时，reset
  if(is_hpp_slot_searching_ == false) {
    ClearParkingInfo(planning_output, planning_hmi_info);
  }

  // sync parameters only if scene_type or dbw_status changes
  const bool dbw_status = session_.environmental_model().GetVehicleDbwStatus();
  if ((function_type != GENERAL_PLANNING_CONTEXT.GetStatemachine().scene_type ||
       (dbw_status != GENERAL_PLANNING_CONTEXT.GetStatemachine().dbw_status))) {
    session_.mutable_planning_context()->ResetTaskOutput();
    SyncParameters(function_type);
  }
  GENERAL_PLANNING_CONTEXT.MutableStatemachine().dbw_status = dbw_status;
  GENERAL_PLANNING_CONTEXT.MutableStatemachine().scene_type = function_type;

#ifdef PlanTimeBenchmark
  double start_time, end_time;
  start_time = IflyTime::Now_ms();
#endif
  // update environment model
  if (!environmental_model_manager_.Run()) {
    session_.mutable_planning_context()->Clear();
    return false;
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("EnvironmentalModelManagerCost", end_time - start_time);
  start_time = IflyTime::Now_ms();
#endif

  bool planning_success;
  if (function_type == planning::common::SceneType::HIGHWAY) {
    planning_success = scc_function_->Plan();
  } else if (function_type == planning::common::SceneType::HPP) {
    planning_success = hpp_function_->Plan();
  } else if (function_type == planning::common::SceneType::RADS) {
    planning_success = rads_function_->Plan();
  }  else if (function_type == planning::common::SceneType::NSA) {
    planning_success = nsa_function_->Plan();
  } else {
    planning_success = scc_function_->Plan();
  }

#ifdef PlanTimeBenchmark
  end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("TaskFunctionCost", end_time - start_time);
#endif

  JSON_DEBUG_VALUE("current planning_success", planning_success);
  session_.mutable_planning_context()->mutable_last_planning_success() =
      planning_success;
  if (!planning_success) {
    ILOG_ERROR << "Planning failed!!! ";
    if (!UpdateFailedPlanningResult()) {
      ILOG_ERROR << "RunOnce failed !!!";
      FillPlanningRequest(iflyauto::REQUEST_LEVEL_MIDDLE, planning_output);
      return false;
    }
  } else {
    CheckTrajectory();
    FillPlanningRequest(iflyauto::REQUEST_LEVEL_NO_REQ, planning_output);
    UpdateSuccessfulPlanningResult();
  }

  ILOG_INFO << "The RunOnce is successed !!!!:";
  JSON_DEBUG_VALUE("planning_fault_code", FaultCode());
  // 存在问题
  // session_.mutable_planning_context()->mutable_last_planning_success() =
  // planning_success;
  session_.mutable_planning_context()->mutable_planning_success() = true;

  const auto end_timestamp = IflyTime::Now_ms();
  const double time_consumption = end_timestamp - start_timestamp;
  ILOG_INFO << "general planning: planning time cost: " << time_consumption;
  JSON_DEBUG_VALUE("planning_time_cost", time_consumption);
  FillPlanningTrajectory(start_timestamp, planning_output);
  FillPlanningHmiInfo(start_timestamp, planning_hmi_info);
  FillAdasPlanningHmiInfo(start_timestamp, planning_hmi_info);
  // can not active lcc/noa function if current planning failed
  if (!planning_success) {
    planning_hmi_info->ad_info.is_avaliable = false;
  }
  return true;
}

void PlanningScheduler::CheckTrajectory() {
  using namespace framework;
  const auto& traj_points =
      session_.planning_context().planning_result().traj_points;
  const auto& motion_planner_output =
      session_.planning_context().motion_planner_output();
  motion_planner_output.x_s_spline;
  const auto ego_state_mgr =
      session_.environmental_model().get_ego_state_manager();
  const auto& ego_pose = ego_state_mgr->ego_pose();
  Eigen::Vector2d cur_pos(ego_pose.x, ego_pose.y);
  Eigen::Vector2d init_point(ego_state_mgr->planning_init_point().x,
                             ego_state_mgr->planning_init_point().y);

  auto* fault_counter_info_ptr = session_.mutable_fault_counter_info();
  for (FaultType type = FaultType::TRAJ_LENGTH_RANGE;
       type != static_cast<FaultType>(
                   static_cast<int>(FaultType::TRAJ_ROLL_CONSISTENCY) + 1);
       type = static_cast<FaultType>(static_cast<int>(type) + 1)) {
    int i = static_cast<int>(type);
    switch (type) {
      case FaultType::TRAJ_LENGTH_RANGE: {
        const double traj_length = traj_points.back().s - traj_points.front().s;
        if (traj_length > 250) {
          (*fault_counter_info_ptr)[i].fault_recovery_counter = 0;
          (*fault_counter_info_ptr)[i].fault_trigger_counter++;
          if ((*fault_counter_info_ptr)[i].fault_trigger_counter >= 5 &&
              FaultCode() < 39000) {
            SetFaultCode(39006);
          }
        } else {
          (*fault_counter_info_ptr)[i].fault_trigger_counter = 0;
          if (FaultCode() == 39006) {
            (*fault_counter_info_ptr)[i].fault_recovery_counter++;
          }
        }
        break;
      }
      case FaultType::TRAJ_LENGTH_CONTINUITY:

        break;

      case FaultType::TRAJ_CURVATURE_RANGE: {
        if (std::any_of(traj_points.begin(), traj_points.end(),
                        [](const TrajectoryPoint& traj_point) {
                          return std::fabs(traj_point.curvature) > 0.35;
                        })) {
          (*fault_counter_info_ptr)[i].fault_recovery_counter = 0;
          (*fault_counter_info_ptr)[i].fault_trigger_counter++;
          if ((*fault_counter_info_ptr)[i].fault_trigger_counter >= 5 &&
              FaultCode() < 39000) {
            SetFaultCode(39007);
          }
        } else {
          (*fault_counter_info_ptr)[i].fault_trigger_counter = 0;
          if (FaultCode() == 39007) {
            (*fault_counter_info_ptr)[i].fault_recovery_counter++;
          }
        }
        break;
      }

      case FaultType::TRAJ_CURVATURE_CONTINUITY:
        break;

      case FaultType::TRAJ_LON_POS_CONSISTENCY: {
        pnc::spline::Projection projection_spline;
        projection_spline.CalProjectionPoint(
            motion_planner_output.x_s_spline, motion_planner_output.y_s_spline,
            motion_planner_output.s_lat_vec.front(),
            motion_planner_output.s_lat_vec.back(), cur_pos);
        const auto& proj_point = projection_spline.GetOutput().point_proj;
        const auto lon_err = std::hypot(init_point.x() - proj_point.x(),
                                        init_point.y() - proj_point.y());
        if (fabs(lon_err) > 3.0) {
          (*fault_counter_info_ptr)[i].fault_recovery_counter = 0;
          (*fault_counter_info_ptr)[i].fault_trigger_counter++;
          if ((*fault_counter_info_ptr)[i].fault_trigger_counter >= 5 &&
              FaultCode() < 39000) {
            SetFaultCode(39008);
          }
        } else {
          (*fault_counter_info_ptr)[i].fault_trigger_counter = 0;
          if (FaultCode() == 39008) {
            (*fault_counter_info_ptr)[i].fault_recovery_counter++;
          }
        }
        break;
      }

      case FaultType::TRAJ_LON_VEL_CONSISTENCY: {
        const auto lon_vel_err =
            ego_state_mgr->planning_init_point().v - ego_state_mgr->ego_v();
        if (fabs(lon_vel_err) > (5.0 / 3.6)) {
          (*fault_counter_info_ptr)[i].fault_recovery_counter = 0;
          (*fault_counter_info_ptr)[i].fault_trigger_counter++;
          if ((*fault_counter_info_ptr)[i].fault_trigger_counter >= 5 &&
              FaultCode() < 39000) {
            SetFaultCode(39009);
          }
        } else {
          (*fault_counter_info_ptr)[i].fault_trigger_counter = 0;
          if (FaultCode() == 39009) {
            (*fault_counter_info_ptr)[i].fault_recovery_counter++;
          }
        }
        break;
      }

      case FaultType::TRAJ_LON_ACC_CONSISTENCY: {
        bool is_acc = ego_state_mgr->planning_init_point().a > 0.1;
        const auto lon_acc_err =
            ego_state_mgr->planning_init_point().a - ego_state_mgr->ego_acc();
        if (is_acc && fabs(lon_acc_err) > 3.0) {
          (*fault_counter_info_ptr)[i].fault_recovery_counter = 0;
          (*fault_counter_info_ptr)[i].fault_trigger_counter++;
          if ((*fault_counter_info_ptr)[i].fault_trigger_counter >= 5 &&
              FaultCode() < 39000) {
            SetFaultCode(39010);
          }
        } else {
          (*fault_counter_info_ptr)[i].fault_trigger_counter = 0;
          if (FaultCode() == 39010) {
            (*fault_counter_info_ptr)[i].fault_recovery_counter++;
          }
        }
        break;
      }

      case FaultType::TRAJ_LON_DEC_CONSISTENCY: {
        bool is_acc = ego_state_mgr->planning_init_point().a > 0.1;
        const auto lon_acc_err =
            ego_state_mgr->planning_init_point().a - ego_state_mgr->ego_acc();
        if ((!is_acc) && fabs(lon_acc_err) > 4.0) {
          (*fault_counter_info_ptr)[i].fault_recovery_counter = 0;
          (*fault_counter_info_ptr)[i].fault_trigger_counter++;
          if ((*fault_counter_info_ptr)[i].fault_trigger_counter >= 5 &&
              FaultCode() < 39000) {
            SetFaultCode(39011);
          }
        } else {
          (*fault_counter_info_ptr)[i].fault_trigger_counter = 0;
          if (FaultCode() == 39011) {
            (*fault_counter_info_ptr)[i].fault_recovery_counter++;
          }
        }
        break;
      }

      case FaultType::TRAJ_LAT_POS_CONSISTENCY: {
        pnc::spline::Projection projection_spline;
        projection_spline.CalProjectionPoint(
            motion_planner_output.x_s_spline, motion_planner_output.y_s_spline,
            motion_planner_output.s_lat_vec.front(),
            motion_planner_output.s_lat_vec.back(), cur_pos);
        const auto& lat_err = projection_spline.GetOutput().dist_proj;
        if (fabs(lat_err) > 0.8) {
          (*fault_counter_info_ptr)[i].fault_recovery_counter = 0;
          (*fault_counter_info_ptr)[i].fault_trigger_counter++;
          if ((*fault_counter_info_ptr)[i].fault_trigger_counter >= 5 &&
              FaultCode() < 39000) {
            SetFaultCode(39012);
          }
        } else {
          (*fault_counter_info_ptr)[i].fault_trigger_counter = 0;
          if (FaultCode() == 39012) {
            (*fault_counter_info_ptr)[i].fault_recovery_counter++;
          }
        }
        break;
      }
      case FaultType::TRAJ_LAT_ACC_CONSISTENCY: {
        // double cos_theta = std::cos(ego_pose.theta);
        // double sin_theta = std::sin(ego_pose.theta);

        // // 计算车身坐标系下的速度分量
        // v_body.vx = v_world.vx * cos_theta + v_world.vy * sin_theta;
        break;
      }

      case FaultType::TRAJ_YAW_CONSISTENCY: {
        break;
      }

      default:
        break;
    }
  }
}

void PlanningScheduler::RecordTimeBenchmarkInfo() {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto* time_benchmark = debug->mutable_time_benchmark();
  const auto& benchmark = TimeBenchmark::Instance();

  time_benchmark->set_planning_time(
      benchmark.times[TimeBenchmarkType::TB_PLANNING_TOTAL].time_ms_);

  // set apa_planning_module_time
  time_benchmark->mutable_apa_planning_time()->set_apa_planning_time(
      benchmark.times[TimeBenchmarkType::TB_APA_TOTAL_TIME].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_slot_manager_time(
      benchmark.times[TimeBenchmarkType::TB_APA_SLOT_MANAGER_TIME].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_jlt_optimizer_time(
      benchmark.times[TimeBenchmarkType::TB_APA_JLT_TIME].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_speed_qp_time(
      benchmark.times[TimeBenchmarkType::TB_APA_QP_TIME].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_dp_optimizer_time(
      benchmark.times[TimeBenchmarkType::TB_APA_DP_TIME].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_apa_stop_decider_time(
      benchmark.times[TimeBenchmarkType::TB_APA_STOP_DECIDER].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_speed_limit_decider_time(
      benchmark.times[TimeBenchmarkType::TB_APA_SPEED_LIMIT_DECIDER].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_osqp_time(
      benchmark.times[TimeBenchmarkType::TB_OSQP].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_path_plan_time(
      benchmark.times[TimeBenchmarkType::TB_APA_PATH_PLAN_TIME].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_astar_time(
      benchmark.times[TimeBenchmarkType::TB_APA_ASTAR].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_speed_plan_time(
      benchmark.times[TimeBenchmarkType::TB_APA_SPEED_PLAN_TIME].time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_find_target_pose_time(
      benchmark.times[TimeBenchmarkType::TB_APA_FIND_TARGET_POSE_TIME]
          .time_ms_);
  time_benchmark->mutable_apa_planning_time()->set_gen_obs_time(
      benchmark.times[TimeBenchmarkType::TB_APA_GEN_OBS_TIME].time_ms_);

  TimeBenchmark::Instance().Clear();
}

}  // namespace planning