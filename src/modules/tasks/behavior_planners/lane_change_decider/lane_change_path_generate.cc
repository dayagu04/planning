#include "lane_change_path_generate.h"
#include <cmath>
#include <vector>
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "library/lc_idm_lib/include/basic_intelligent_driver_model.h"
#include "planning_context.h"
#include "session.h"
#include "utils/kd_path.h"
#include "vehicle_model_simulation.h"

static const double kOneSix = 1.0 / 6.0;
static const double kOneThree = 1.0 / 3.0;

namespace planning {

LaneChangePathGenerateManager::LaneChangePathGenerateManager(
    std::shared_ptr<ReferencePath> ref_path, framework::Session* session)
    : ref_path_(ref_path), session_(session) {
  // lc_decider_config_ = config_builder->cast<LaneChangeDeciderConfig>();
}

bool LaneChangePathGenerateManager::GenerateLCPath(const double lat_offset) {
  const double dt = 0.2;
  const double max_simulate_time = 5.0;
  const double end_poise_lat_err = 0.2;
  const double end_poise_theta_err = 0.12;
  const double default_front_dis = 150;
  const double default_front_v = 33.3;

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  lc_path_result_.reset();
  // lat param 、state
  const auto& ego_state = ref_path_->get_frenet_ego_state();
  const double wheel_base = vehicle_param.wheel_base;
  const double ld = ego_state.velocity() * 3.0;
  BasicPurePursuitModel::ModelParam pp_model_param(ld, wheel_base);

  const auto &planning_init_point = ego_state.planning_init_point();

  BasicPurePursuitModel::ModelState pp_model_state(
      planning_init_point.x, planning_init_point.y,
      planning_init_point.heading_angle, ego_state.velocity());

  // lon model param 、state
  BasicIntelligentDriverModel::ModelParam idm_model_param;
  idm_model_param.kDesiredVelocity =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();

  BasicIntelligentDriverModel::ModelState idm_model_state(
      0.0, planning_init_point.lon_init_state.v(), default_front_dis,
      default_front_v);

  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();

  const int front_agent_node_id =
      lane_change_decider_output.lc_gap_info.front_node_id;

  bool is_exist_front_agent = front_agent_node_id != planning_data::kInvalidId;
  bool is_get_front_agent_predicton_infos_succeed = false;
  LaneChangePathGenerateManager::AgentPredictionTrajectoryPoints
      front_agent_prediction_infos;

  if (is_exist_front_agent) {
    const auto front_agent_node =
        session_->environmental_model().get_dynamic_world()->GetNode(
            front_agent_node_id);
    is_get_front_agent_predicton_infos_succeed =
        CalculateFrontAgentPredictionInfo(&front_agent_prediction_infos,
                                          front_agent_node);
  }

  if (ref_path_ == nullptr) {
    return false;
  }

  const auto& ref_frenet_coor = ref_path_->get_frenet_coord();
  Point2D cart_point(pp_model_state.x, pp_model_state.y);
  Point2D frenet_point;

  if (!ref_frenet_coor->XYToSL(cart_point, frenet_point)) {
    return false;
  }

  int index = 0;
  bool iter_terminate = false;

  std::vector<double> ego_sim_s;

  pp_model_.ProcessReferencePath(ref_path_);

  while (!iter_terminate) {
    index++;

    //lat simulation

    pp_model_.set_model_state(pp_model_state);

    pp_model_.set_model_param(pp_model_param);

    pp_model_.CalculateDesiredDelta(lat_offset);

    pp_model_.Reset();

    State_Sim temp_state;
    temp_state.x = pp_model_state.x;
    temp_state.y = pp_model_state.y;
    temp_state.theta = pp_model_state.theta;
    temp_state.delta = pp_model_.get_delta();

    // lon simulation
    double desire_acc;
    idm_model_.GetAccDesiredAcceleration(idm_model_param, idm_model_state, &desire_acc);
    temp_state.v = idm_model_state.vel + desire_acc * dt;

    //update ego next state using vehicle simulation model
    pnc::steerModel::VehicleSimulation vehicle_simulate;
    vehicle_simulate.set_dt_resolution(0.2);
    pnc::steerModel::VehicleParameter vehicle_param;
    pnc::steerModel::VehicleState vehicle_state{temp_state.x, temp_state.y, temp_state.theta};
    pnc::steerModel::VehicleControl vehicle_control{temp_state.v, temp_state.delta};

    vehicle_simulate.Init(vehicle_state);
    vehicle_simulate.Update(vehicle_control, vehicle_param);
    const auto new_state = vehicle_simulate.GetState();

    // update pp_model state continue iter
    pp_model_state.x = new_state.x_;
    pp_model_state.y = new_state.y_;
    pp_model_state.theta = new_state.phi_;
    pp_model_state.vel = temp_state.v;

    // update idm_model state continue iter
    if (is_exist_front_agent &&
        is_get_front_agent_predicton_infos_succeed) {
      idm_model_state.s = 0;
      idm_model_state.s_front =
          front_agent_prediction_infos.s_t_spline(dt * index);
      idm_model_state.vel = temp_state.v;
      idm_model_state.vel_front =
          front_agent_prediction_infos.v_t_spline(dt * index);
    } else {
      idm_model_state.s = 0;
      idm_model_state.s_front = default_front_dis;
      idm_model_state.vel = temp_state.v;
      idm_model_state.vel_front = default_front_v;
    }

    lc_path_result_.x.push_back(new_state.x_);
    lc_path_result_.y.push_back(new_state.y_);
    lc_path_result_.theta.push_back(new_state.phi_);
    lc_path_result_.t.push_back(dt * index);
    lc_path_result_.v.push_back(temp_state.v);

    cart_point.x = new_state.x_;
    cart_point.y = new_state.y_;

    if (!ref_frenet_coor->XYToSL(cart_point, frenet_point)) {
      return false;
    }
    ego_sim_s.push_back(frenet_point.x - ref_path_->get_frenet_ego_state().s());

    ReferencePathPoint reference_path_point;
    if (!ref_path_->get_reference_point_by_lon(frenet_point.x,
                                               reference_path_point)) {
      return false;
    }

    double theta_err = std::abs(new_state.phi_ -
                                reference_path_point.path_point.theta());
    double lat_err = std::abs(frenet_point.y - lat_offset);
    bool is_poise_near_lane =
        lat_err < end_poise_lat_err && theta_err < end_poise_theta_err;

    bool is_over_lane = false;
    const auto& lc_dir = lane_change_decider_output.lc_request;
    if (lc_dir == LEFT_CHANGE) {
      is_over_lane = frenet_point.y > planning_math::KD_EPSILON;
    } else if (lc_dir == RIGHT_CHANGE) {
      is_over_lane = frenet_point.y < planning_math::KD_EPSILON;
    }

    iter_terminate =
          is_poise_near_lane ||
          index * dt > max_simulate_time ||
          is_over_lane;
  }

  if (lc_path_result_.t.size() < 3) {
    return false;
  }

  lc_path_result_.x_t_spline.set_points(lc_path_result_.t, lc_path_result_.x);
  lc_path_result_.y_t_spline.set_points(lc_path_result_.t, lc_path_result_.y);
  lc_path_result_.theta_t_spline.set_points(lc_path_result_.t,
                                            lc_path_result_.theta);

  JSON_DEBUG_VECTOR("lat_path_x", lc_path_result_.x, 2);
  JSON_DEBUG_VECTOR("lat_path_y", lc_path_result_.y, 2);
  JSON_DEBUG_VECTOR("lat_path_v", lc_path_result_.v, 2);
  JSON_DEBUG_VECTOR("lat_path_t", lc_path_result_.t, 2);
  JSON_DEBUG_VECTOR("ego_sim_s", ego_sim_s, 2);

  return true;
}

bool LaneChangePathGenerateManager::GenerateEgoFutureTrajectory(const double lat_offset,
                                  const planning_data::DynamicAgentNode* front_agent_node) {
  // 如果有头车，先处理预测轨迹
  front_node_future_trajectory_.clear();
  bool is_exist_front_agent = front_agent_node != nullptr;
  if(is_exist_front_agent){
    const auto &front_agent_prediction_trajectory =
      front_agent_node->node_trajectories_used_by_st_graph()[0];
    const auto& ref_frenet_coord = ref_path_->get_frenet_coord();
    if (!ref_frenet_coord) {
      return false;
    }
    const auto vehicle_param =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
    for (const auto agent_point: front_agent_prediction_trajectory) {
      TrajectoryPoint point;
      point.x = agent_point.x();
      point.y = agent_point.y();
      point.heading_angle = agent_point.theta();
      point.v = agent_point.vel();
      Point2D cart_point(point.x, point.y);
      Point2D frenet_point;
      if (!ref_frenet_coord->XYToSL(cart_point, frenet_point)) {
        continue;
      }
      point.s = frenet_point.x - front_agent_node->node_length() * 0.5; // back edge
      front_node_future_trajectory_.push_back(point);
    }
  }
  // 清空 trajectory
  ego_future_trajectory_.clear();
  lc_path_result_.reset();
  // 递推条件
  const double dt = 0.2;
  const double max_simulate_time = 5.0;
  const double end_poise_lat_err = 0.4;
  const double end_poise_theta_err = 0.12;

  // 公共信息
  const auto &car_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& ego_state = ref_path_->get_frenet_ego_state();
  const auto &planning_init_point = ego_state.planning_init_point();
  const auto& ref_frenet_coor = ref_path_->get_frenet_coord();
  const auto &lane_change_decider_output =
    session_->planning_context().lane_change_decider_output();

  // pp model default
  const double wheel_base = car_param.wheel_base;
  const double ld = ego_state.velocity() * 2.0 + 1.0;
  BasicPurePursuitModel::ModelParam pp_model_param(ld, wheel_base);
  BasicPurePursuitModel::ModelState pp_model_state(
    planning_init_point.x, planning_init_point.y,
    planning_init_point.heading_angle, ego_state.velocity());
  pp_model_.ProcessReferencePath(ref_path_);

  // idm model default
  const double default_front_dis = 150;
  const double default_front_v = 33.3;
  BasicIntelligentDriverModel::ModelParam idm_model_param;
  idm_model_param.kDesiredVelocity =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();
  BasicIntelligentDriverModel::ModelState idm_model_state(
      0.0, ego_state.velocity(), default_front_dis,
      default_front_v);

  if (is_exist_front_agent) {
      idm_model_state.s = 0;
      idm_model_state.s_front = front_node_future_trajectory_[0].s -
      (50.0 + car_param.front_edge_to_rear_axle);//ego_traj_point.s
      // idm_model_state.vel = temp_state.v;
      idm_model_state.vel_front =
          front_node_future_trajectory_[0].v;
    }
  // const auto &lane_change_decider_output =
  //     session_->planning_context().lane_change_decider_output();

  // const int front_agent_node_id =
  //     lane_change_decider_output.lc_gap_info.front_node_id;

  // bool is_exist_front_agent = front_agent_node_id != planning_data::kInvalidId;
  // bool is_get_front_agent_predicton_infos_succeed = false;
  // LaneChangePathGenerateManager::AgentPredictionTrajectoryPoints
  //     front_agent_prediction_infos;

  // if (is_exist_front_agent) {
  //   const auto front_agent_node =
  //       session_->environmental_model().get_dynamic_world()->GetNode(
  //           front_agent_node_id);
  //   is_get_front_agent_predicton_infos_succeed =
  //       CalculateFrontAgentPredictionInfo(&front_agent_prediction_infos,
  //                                         front_agent_node);
  // }

  // Point2D cart_point(pp_model_state.x, pp_model_state.y);
  // Point2D frenet_point;
  // if (ref_path_ == nullptr) {
  //   return false;
  // }
  // if (!ref_frenet_coor->XYToSL(cart_point, frenet_point)) {
  //   return false;
  // }

  // 自车轨迹起点
  TrajectoryPoint init_point;
  init_point.x = planning_init_point.x;
  init_point.y = planning_init_point.y;
  init_point.v = ego_state.velocity();
  init_point.a = planning_init_point.a;
  init_point.heading_angle = planning_init_point.heading_angle;
  init_point.s = planning_init_point.frenet_state.s;
  init_point.l = planning_init_point.frenet_state.r;
  init_point.t = 0;
  ego_future_trajectory_.push_back(init_point);
  // 可视化同步
  lc_path_result_.x.push_back(init_point.x );
  lc_path_result_.y.push_back(init_point.y);
  lc_path_result_.theta.push_back(init_point.heading_angle);
  lc_path_result_.t.push_back(0);
  lc_path_result_.v.push_back(init_point.v);
  std::vector<double> ego_sim_s;

  int index = 0;
  bool iter_terminate = false;
  bool is_close = false;
  // 递推自车轨迹
  while (!iter_terminate) {
    index++; //index = 1 -> ...
    //lat simulation
    // double ld_v = ComputeLd(pp_model_state.vel, is_close);
    double ld_v = pp_model_state.vel * 1.2 + 1;
    if(is_close){
      ld_v = pp_model_state.vel * 0.3 + 0.5;
    }
    BasicPurePursuitModel::ModelParam pp_model_param(ld_v, wheel_base);
    pp_model_.set_model_state(pp_model_state);
    pp_model_.set_model_param(pp_model_param);
    pp_model_.CalculateDesiredDelta(lat_offset);
    pp_model_.Reset();
    State_Sim temp_state;
    temp_state.x = pp_model_state.x;
    temp_state.y = pp_model_state.y;
    temp_state.theta = pp_model_state.theta;
    temp_state.delta = pp_model_.get_delta();
    temp_state.delta = pnc::mathlib::Clamp(temp_state.delta, -0.5, 0.5);

    // lon simulation
    double desire_acc;
    idm_model_.GetAccDesiredAcceleration(idm_model_param, idm_model_state, &desire_acc);
    desire_acc = pnc::mathlib::Clamp(desire_acc, -1.5, 1.5);
    temp_state.v = idm_model_state.vel + desire_acc * dt;

    //update ego next state using vehicle simulation model
    pnc::steerModel::VehicleSimulation vehicle_simulate;
    pnc::steerModel::VehicleParameter vehicle_param;
    pnc::steerModel::VehicleState vehicle_state{temp_state.x, temp_state.y, temp_state.theta};
    pnc::steerModel::VehicleControl vehicle_control{temp_state.v, temp_state.delta};

    vehicle_simulate.Init(vehicle_state);
    vehicle_simulate.Update(vehicle_control, vehicle_param);
    const auto new_state = vehicle_simulate.GetState();

    // update pp_model state continue iter
    pp_model_state.x = new_state.x_;
    pp_model_state.y = new_state.y_;
    pp_model_state.theta = new_state.phi_;
    pp_model_state.vel = temp_state.v;

    lc_path_result_.x.push_back(new_state.x_);
    lc_path_result_.y.push_back(new_state.y_);
    lc_path_result_.theta.push_back(new_state.phi_);
    lc_path_result_.t.push_back(dt * index);
    lc_path_result_.v.push_back(temp_state.v);

    Point2D cart_new_xy(new_state.x_, new_state.y_);
    Point2D sl_point;
    if (!ref_frenet_coor->XYToSL(cart_new_xy, sl_point)) {
      return false;
    }
    TrajectoryPoint ego_traj_point;
    ego_traj_point.s = sl_point.x;
    ego_traj_point.l = sl_point.y;
    ego_traj_point.t = index * dt;
    ego_traj_point.v = temp_state.v;
    ego_traj_point.x = new_state.x_;
    ego_traj_point.y = new_state.y_;
    ego_traj_point.heading_angle = new_state.phi_;
    ego_traj_point.a = desire_acc;
    ego_future_trajectory_.emplace_back(ego_traj_point);

    ego_sim_s.push_back(sl_point.x - ref_path_->get_frenet_ego_state().s());

 // update idm_model state continue iter
    if (is_exist_front_agent) {
      idm_model_state.s = 0;
      idm_model_state.s_front = front_node_future_trajectory_[index].s -
      (ego_traj_point.s + car_param.front_edge_to_rear_axle);
      idm_model_state.vel = temp_state.v;
      idm_model_state.vel_front =
          front_node_future_trajectory_[index].v;
    } else {
      idm_model_state.s = 0;
      idm_model_state.s_front = default_front_dis;
      idm_model_state.vel = temp_state.v;
      idm_model_state.vel_front = default_front_v;
    }


    ReferencePathPoint reference_path_point;
    if (!ref_path_->get_reference_point_by_lon(sl_point.x,
                                               reference_path_point)) {
      return false;
    }

    double theta_err = std::abs(new_state.phi_ -
                                reference_path_point.path_point.theta());
    double lat_err = std::abs(sl_point.y - lat_offset);
    bool is_near_lane =
        lat_err < end_poise_lat_err && theta_err < end_poise_theta_err;

    bool is_over_lane = false;
    const auto& lc_dir = lane_change_decider_output.lc_request;
    if (lc_dir == LEFT_CHANGE) {
      is_over_lane = sl_point.y > planning_math::KD_EPSILON;
    } else if (lc_dir == RIGHT_CHANGE) {
      is_over_lane = sl_point.y < planning_math::KD_EPSILON;
    }
    is_close = is_near_lane;
    iter_terminate =
          index * dt > max_simulate_time;
  }

  if (ego_future_trajectory_.size() < 3) {
    return false;
  }

  // lc_path_result_.x_t_spline.set_points(lc_path_result_.t, lc_path_result_.x);
  // lc_path_result_.y_t_spline.set_points(lc_path_result_.t, lc_path_result_.y);
  // lc_path_result_.theta_t_spline.set_points(lc_path_result_.t,
  //                                           lc_path_result_.theta);

  // JSON_DEBUG_VECTOR("lat_path_x", lc_path_result_.x, 2);
  // JSON_DEBUG_VECTOR("lat_path_y", lc_path_result_.y, 2);
  // JSON_DEBUG_VECTOR("lat_path_v", lc_path_result_.v, 2);
  // JSON_DEBUG_VECTOR("lat_path_t", lc_path_result_.t, 2);
  // JSON_DEBUG_VECTOR("ego_sim_s", ego_sim_s, 2);

  return true;
}
LaneChangePathGenerateManager::State_Sim LaneChangePathGenerateManager::UpdateDynamicsOneStep( State_Sim state ,
                                                     double dt) {
  const double &theta = state.theta;
  const double &delta = state.delta;
  const double &omega = state.omega;

  const double &v = state.v;
  const double &k = 0.33;

  const double dt2 = dt * dt;
  const double dtv = dt * v;

  const double theta_tmp = theta + (delta * dt * k * v) * 0.5;
  const double theta_tmp2 =
      (k * omega * v * dt2) * 0.5 + delta * k * v * dt + theta;
  const double theta_tmp3 =
      (k * omega * v * dt2) * 0.25 + (delta * k * v * dt) * 0.5 + theta;

  State_Sim state1 = state;
  state1.x = state.x + (dtv * cos(theta)) * kOneSix +
             (dtv * cos(theta_tmp)) * kOneThree +
             (dtv * cos(theta_tmp2)) * kOneSix +
             (dtv * cos(theta_tmp3)) * kOneThree;

  state1.y = state.y + (dtv * sin(theta)) * kOneSix +
             (dtv * sin(theta_tmp)) * kOneThree +
             (dtv * sin(theta_tmp2)) * kOneSix +
             (dtv * sin(theta_tmp3)) * kOneThree;

  state1.theta = theta_tmp2;

  state1.delta = delta + dt * omega;

  return state1;
}

bool LaneChangePathGenerateManager::CalculateFrontAgentPredictionInfo(
    AgentPredictionTrajectoryPoints* agent_prediction_traj_points,
    const planning_data::DynamicAgentNode* front_agent) {

  const auto &front_agent_prediction_trajectory =
      front_agent->node_trajectories_used_by_st_graph()[0];

  const auto& ref_frenet_coord = ref_path_->get_frenet_coord();
  if (!ref_frenet_coord) {
    return false;
  }

  const auto vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  for (const auto agent_point: front_agent_prediction_trajectory) {
    Point2D cart_point(agent_point.x(), agent_point.y());
    Point2D frenet_point;
    if (!ref_frenet_coord->XYToSL(cart_point, frenet_point)) {
      break;
    }

    double s_front_agent =
        frenet_point.x - ref_path_->get_frenet_ego_state().s() -
        front_agent->node_length() / 2 - vehicle_param.front_edge_to_rear_axle;

    agent_prediction_traj_points->x_vec.push_back(agent_point.x());
    agent_prediction_traj_points->y_vec.push_back(agent_point.y());
    agent_prediction_traj_points->v_vec.push_back(agent_point.vel());
    agent_prediction_traj_points->t_vec.push_back(agent_point.absolute_time());
    agent_prediction_traj_points->s_vec.push_back(s_front_agent);
  }

  if (agent_prediction_traj_points->t_vec.size() < 3) {
    return false;
  }

  agent_prediction_traj_points->s_t_spline.set_points(
      agent_prediction_traj_points->t_vec, agent_prediction_traj_points->s_vec);
  agent_prediction_traj_points->v_t_spline.set_points(
      agent_prediction_traj_points->t_vec, agent_prediction_traj_points->v_vec);

  return true;
}
double LaneChangePathGenerateManager::ComputeLd(double v, bool is_close) {
  const double L0 = 2.0;     // base_ld
  const double kv = 2.0;     // s
  const double Lmin = 3.0;   // m
  const double Lmax = 40.0;  // m

  double Ld = L0 + kv * v;
  if (Ld < Lmin) Ld = Lmin;
  if (Ld > Lmax) Ld = Lmax;

  if (is_close) {
    const double gamma = 0.6;  // 0.6~0.8
    Ld = std::max(Lmin, gamma * Ld);
  }
  return Ld;
}
}