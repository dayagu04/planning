#include "src/modules/tasks/behavior_planners/spatio_temporal_planner/spatio_temporal_grid_map_adapter.h"
#include "src/modules/context/planning_context.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <utility>
#include <vector>
#include "behavior_planners/lateral_offset_decider/lateral_offset_decider_utils.h"
#include "behavior_planners/spatio_temporal_planner/slt_point.h"
#include "define/geometry.h"
#include "config/basic_type.h"
#include "math/aabox2d.h"
#include "math/box2d.h"
#include "spatio_temporal_union_dp_input.pb.h"
#include "src/common/vec2d.h"


namespace planning {

namespace {
constexpr double kAgentPredTrajTimeInterval = 0.2;
constexpr int kDefaultEgoBoxVertices = 4;
constexpr int kDefaultAgentVertices = 8;

constexpr double kLastPlanLengthThr = 2.0;
constexpr double kBoundaryCrossEgoBehindThr = 5.0;
constexpr double kBoundaryCrossEgoFrontThr = 10.0;
constexpr double kCrossLaneCostDefault = 0.2;
constexpr double kInitPosCostStandardThr = 3.6;
constexpr double kInitPosCostWeight = 1.0;
constexpr double kCumuLateralDistanceCostWeight = 1.5;
constexpr double kCrossLaneCostWeight = 1.0;
constexpr double kLaneChangeExecutionWeightRatio = 4.0;
constexpr int32_t kLaneCenterMinPointsThr = 3;
constexpr double kLaneLineSegmentLength = 5.0;
constexpr double kConsiderLaneLineLength = 50.0;
constexpr double kDefaultRoadRadius = 750.0;
constexpr int32_t kDefaultPointNums = 33;
constexpr int32_t kLeastDefaultPointNums = 3;
constexpr double kConsiderLaneStraightFrontEgo = 30.0;
constexpr double kDefaultFrontConsiderLength = 150.0;
constexpr double kDefaultRearConsiderLength = 12.0;
constexpr double kDefaultlatConsiderLength = 6.0;
constexpr double kDefaultVirtualAgentPassConsiderLength = 2.0;
constexpr double kDefaultAgentLateralConsiderDis = 1.2;
constexpr double kDefaultConsiderDynamicObstacleTajsTime = 2.6;
constexpr double kDefaultConsiderLaneWidth = 1.8;
constexpr double kDefaultLaneWidthBuffer = 0.2;
constexpr double kTrajectoryPointsInterval = 0.2;
constexpr double kHalfCoefficient = 0.5;
constexpr double kDefaultLaneWidth = 3.75;
constexpr double kDefaultPlanningDuration = 5.0;
constexpr double kPlanningDuration = 6.0;
constexpr double kMinEgoFrontConsiderDistance = 20.0;
constexpr double kMaxEgoAcceleration = 2.0;
constexpr double kMaxEgoLateralVelocity = 0.3;
constexpr double kLongitReservedBuffer  = 0.5;

}  // namespace

std::string SLTGridMapAdapter::Name() { return std::string("spatio_temporal_grid_map_adapter"); }

SLTGridMapAdapter::SLTGridMapAdapter(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
  : session_(session) {
  config_ = config_builder->cast<SpatioTemporalGridMap>();
  // * SscMap config
  SscMap::Config map_cfg;
  map_cfg.map_size[0] = config_.map_size_x;
  map_cfg.map_size[1] = config_.map_size_y;
  map_cfg.map_size[2] = config_.map_size_z;
  map_cfg.map_resolution[0] = config_.map_resl_x;
  map_cfg.map_resolution[1] = config_.map_resl_y;
  map_cfg.map_resolution[2] = config_.map_resl_z;
  map_cfg.s_back_len = config_.s_back_len;
  p_ssc_map_ = new SscMap(map_cfg);
}

void SLTGridMapAdapter::RunOnce() {
  auto time_start = IflyTime::Now_ms();
  if (session_ == nullptr) {
    return;
  }

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& virtual_lane_mgr =
      // session_->mutable_planning_context()->virtual_lane_manager();
      session_->environmental_model().get_virtual_lane_manager();
  const auto& reference_path_mgr =
      session_->environmental_model().get_reference_path_manager();
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();

  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  const auto& obstacles_id_behind_ego =
      session_->mutable_planning_context()->mutable_lateral_obstacle_decider_output().obstacles_id_behind_ego;

  reference_path_ = reference_path_mgr->get_reference_path_by_lane(
      current_lane->get_virtual_id(), false);
  // planning_init_point_ =
  //     ego_state_manager->planning_init_point();

  current_lane_coord_ = reference_path_->get_frenet_coord();
  ego_frenet_state_ = current_lane->get_reference_path()->get_frenet_ego_state();
  Point2D ego_cart_point(ego_state_manager->ego_pose().x, ego_state_manager->ego_pose().y);
  Point2D ego_frenet_point;
  if (!current_lane_coord_->XYToSL(ego_cart_point,
                                    ego_frenet_point)) {
    LOG_DEBUG("SLTGridMapAdapter::RunOnce() ego on reference path failed!");
  }

  if (reference_path_ != nullptr) {
    origin_lane_width_ =
        virtual_lane_mgr
            ->get_lane_with_virtual_id(
                current_lane->get_virtual_id())
            ->width_by_s(ego_frenet_point.x);  // near s
    use_query_lane_width_ = true;
  } else {
    origin_lane_width_ = kDefaultLaneWidth;
    use_query_lane_width_ = false;
  }


  // 计算自车初始状态initial_state_
  initial_state_.time_stamp = 0.0;
  initial_state_.vec_position.set_x(ego_state_manager->ego_pose().x);
  initial_state_.vec_position.set_y(ego_state_manager->ego_pose().y);
  initial_state_.angle = ego_state_manager->heading_angle();
  initial_state_.velocity = ego_state_manager->ego_v();
  initial_state_.acceleration = ego_state_manager->ego_acc();

  const auto& current_agents = agent_manager->GetAllCurrentAgents();
  consider_surround_agents_.clear();
  // static_obstacles_.clear();
  // dynamic_obstacles_.clear();
  virtual_agents_.clear();
  double min_consider_distance = 10.0;
  double ve_max =
      std::min(ego_state_manager->ego_v() + kMaxEgoAcceleration * kDefaultPlanningDuration, ego_state_manager->ego_v_cruise());
  double acc_t = (ego_state_manager->ego_v_cruise() - ego_state_manager->ego_v()) / kMaxEgoAcceleration;
  if (acc_t > 0.0) {
    if (acc_t > kDefaultPlanningDuration) {
      ego_front_consider_obstacle_distance_ =
          kHalfCoefficient * (ego_state_manager->ego_v() + ve_max) * kDefaultPlanningDuration;
    } else {
      ego_front_consider_obstacle_distance_ = kHalfCoefficient * (ego_state_manager->ego_v() + ve_max) * acc_t +
          ego_state_manager->ego_v_cruise() * (kDefaultPlanningDuration - acc_t);
    }
  } else {
    ego_front_consider_obstacle_distance_ = ego_state_manager->ego_v() * kDefaultPlanningDuration;
  }

  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  const auto& lead_agent = agent_manager->GetAgent(cipv_info.cipv_id());
  if (lead_agent != nullptr) {
    Point2D lead_point(lead_agent->x(), lead_agent->y());
    Point2D lead_frenet_point(ego_front_consider_obstacle_distance_, 0.0);
    if (!current_lane_coord_->XYToSL(lead_point,
                                  lead_frenet_point)) {
      LOG_DEBUG("lead_agent on reference path failed!");
    }
    if (lead_agent->is_static()) {
      ego_front_consider_obstacle_distance_ = 
          std::min(ego_front_consider_obstacle_distance_, lead_frenet_point.x - ego_frenet_state_.s());
    }
  }
  ego_front_consider_obstacle_distance_ = std::max(ego_front_consider_obstacle_distance_, min_consider_distance);

  for (const auto& agent : current_agents) {
    if (agent != nullptr) {
      //添加虚拟障碍物
      if (agent->type() == agent::AgentType::VIRTUAL && agent->is_tfl_virtual_obs()) {
        virtual_agents_.emplace_back(agent);
      }

      Point2D agent_point(agent->x(), agent->y());
      Point2D agent_frenet_point;
      if (!current_lane_coord_->XYToSL(agent_point,
                                        agent_frenet_point)) {
        LOG_DEBUG("agent on reference path failed!");
        continue;
      }
      // if (agent_frenet_point.x > lead_frenet_point.x &&
      //     std::fabs(agent_frenet_point.y) < kDefaultConsiderLaneWidth) {
      //   continue;
      // }

      if (agent_frenet_point.x - ego_frenet_point.x > ego_front_consider_obstacle_distance_ ||
          std::fabs(agent_frenet_point.y) > kDefaultlatConsiderLength ||
          agent_frenet_point.x < ego_frenet_point.x - kDefaultRearConsiderLength) {
        continue;
      }

      //过滤自车正后方障碍物
      if (agent_frenet_point.x < ego_frenet_point.x) {
        auto iter =
            std::find(obstacles_id_behind_ego.begin(), obstacles_id_behind_ego.end(), agent->agent_id());
        if (iter != obstacles_id_behind_ego.end()) {
          continue;
        }
      }

      consider_surround_agents_.emplace_back(agent);


      // 添加静止、动态障碍物
      // if (agent->is_static()) {
      //   static_obstacles_.emplace_back(agent);
      // } else {
      //   dynamic_obstacles_.emplace_back(agent);
      // }
    }
  }

  // auto t_prepare = IflyTime::Now_ms();
  StateTransformForInputData();

  // auto t_stf = IflyTime::Now_ms();;
  // LOG_DEBUG("StateTransformForInputData:%f\n", t_stf - t_prepare);

  // time_origin_ = initial_state_.time_stamp;
  // p_ssc_map_->ResetSscMap(
  //     ego_frenet_state_, time_origin_);

  // for (int i = 0; i < surround_forward_trajs_state_.size(); ++i) {
  //   p_ssc_map_->ConstructSscMap(surround_forward_trajs_state_[i]);
  // }

  auto time_end = IflyTime::Now_ms();
  LOG_DEBUG("SLTGridMapAdapter::RunOnce() cost:%f\n", time_end - time_start);
  return;
}

void SLTGridMapAdapter::StateTransformForInputData() {
  std::unordered_map<int, std::vector<State>> agents_global_state_vec;
  std::unordered_map<int, std::vector<planning_math::Vec2d>> agents_frenet_point_vec;
  const int num_v = kDefaultEgoBoxVertices;
  const int num_agent = kDefaultAgentVertices;
  // double target_l = kHalfCoefficient * origin_lane_width_ + kDefaultLaneWidthBuffer;
  auto spatio_temporal_union_plan_input =
      DebugInfoManager::GetInstance().GetDebugInfoPb()->mutable_spatio_temporal_union_plan_input();
  auto agent_time_corners = spatio_temporal_union_plan_input->mutable_agent_time_corners();
  agent_time_corners->Clear();
  // ~ Stage I. Package states and points

  // * Ego vehicle state and vertices
  // include global and frenet point
  std::vector<planning_math::Vec2d> vertices;
  std::vector<planning_math::Vec2d> ego_frenet_vec;
  GetVehicleVertices( initial_state_, &vertices);
  AgentVerticesTransform(vertices, &ego_frenet_vec);

  // * Surrounding vehicle trajs
  for (size_t i = 0; i < consider_surround_agents_.size(); ++i) {
    const auto agent_iter = consider_surround_agents_[i];
    if (agent_iter->trajectories().empty()) {
      continue;
    }
    const trajectory::Trajectory agent_trajectory =
        agent_iter->trajectories()[0];
    std::vector<State> agent_trajs_state;
    std::vector<planning_math::Vec2d> agent_frenet_point_vec;
    for (size_t k = 0; k < agent_trajectory.size(); k++) {
      // states
      State traj_state;
      traj_state.time_stamp = k * kAgentPredTrajTimeInterval;
      double trajectory_point_x =
          agent_trajectory.Evaluate(traj_state.time_stamp).x();
      double trajectory_point_y =
          agent_trajectory.Evaluate(traj_state.time_stamp).y();
      traj_state.vec_position.set_x(trajectory_point_x);
      traj_state.vec_position.set_y(trajectory_point_y);
      traj_state.angle =
          agent_trajectory.Evaluate(traj_state.time_stamp).theta();
      traj_state.velocity =
          agent_trajectory.Evaluate(traj_state.time_stamp).vel();
      traj_state.acceleration =
          agent_trajectory.Evaluate(traj_state.time_stamp).acc();
      agent_trajs_state.push_back(traj_state);

      // vertices
      std::vector<planning_math::Vec2d> v_global_vec;
      std::vector<planning_math::Vec2d> v_frenet_vec;
      GetAgentVertices(*agent_iter, traj_state, &v_global_vec);

      AgentVerticesTransform(v_global_vec, &v_frenet_vec);

      agent_frenet_point_vec.insert(agent_frenet_point_vec.end(), v_frenet_vec.begin(),
                              v_frenet_vec.end());
    }
    agents_global_state_vec.insert(std::make_pair(agent_iter->agent_id(), agent_trajs_state));
    agents_frenet_point_vec.insert(std::make_pair(agent_iter->agent_id(), agent_frenet_point_vec));
  }

  // ~ Stage II. Retrieve states and points
  // * Ego spatio temporal inFo
  ego_vehicle_initial_state_.agent_id = 0;
  // ego_vehicle_initial_state_.frenet_vertices.clear();
  std::vector<SLTPoint> ego_vertices;
  // std::vector<Vec2d> ego_corners;
  std::array<planning_math::Vec2d, 8> ego_corners;
  for (int i = 0; i < num_v; ++i) {
    SLTPoint fs_point;
    Vec2d ego_corner;
    fs_point.set_t(initial_state_.time_stamp);
    fs_point.set_s(ego_frenet_vec[i].x());
    fs_point.set_l(ego_frenet_vec[i].y());
    ego_vertices.emplace_back(fs_point);
    ego_corner.set_x(ego_frenet_vec[i].x());
    ego_corner.set_y(ego_frenet_vec[i].y());
    ego_corners[i] = ego_corner;
  }
  // ego_vehicle_initial_state_.frenet_vertices.emplace_back(ego_vertices);
  AABox2d ego_box(ego_corners, num_v);

  // * Surrounding trajs spatio temporal inFo
  double min_target_l = ego_box.min_y() - kMaxEgoLateralVelocity * kDefaultConsiderDynamicObstacleTajsTime;
  double max_target_l = ego_box.max_y() + kMaxEgoLateralVelocity * kDefaultConsiderDynamicObstacleTajsTime;
  surround_forward_trajs_state_.clear();
  for (size_t k = 0; k < consider_surround_agents_.size(); ++k) {
    // std::vector<AgentFrenetSpatioTemporalInFo> sur_trajs;
    const auto& agent_iter = consider_surround_agents_[k];
    int agent_id = agent_iter->agent_id();
    AgentFrenetSpatioTemporalInFo agent_state;
    agent_state.agent_id = agent_id;
    agent_state.agent_type = (int)agent_iter->type();
    auto state_iter = agents_global_state_vec.find(agent_id);
    auto fs_iter = agents_frenet_point_vec.find(agent_id);

    if (state_iter == agents_global_state_vec.end() ||
        fs_iter == agents_frenet_point_vec.end()) {
      continue;
    }
    int offset = 0;
    std::vector<Vec2d> max_box_corners;
    double max_x = std::numeric_limits<double>::lowest();
    double min_x = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();

// #ifdef X86
    auto agent_time_corner = agent_time_corners->Add();
    agent_time_corner->set_agent_id(agent_id);
    agent_time_corner->set_agent_type((int)agent_iter->type());
    const auto& fs_point = fs_iter->second;
    int index = 0;
    for (int i = 0; i < state_iter->second.size(); ++i) {
      const auto& state = state_iter->second[i];

      auto time_and_corners = agent_time_corner->mutable_time_and_corners()->Add();
      time_and_corners->set_time(state.time_stamp);
      std::vector<SLTPoint> agent_state_vertices;
      // std::vector<Vec2d> box_corners;
      std::array<planning_math::Vec2d, 8> box_corners;
      for (int j = 0; j < num_agent; ++j) {
        auto agent_corner = time_and_corners->mutable_agent_corner()->Add();
        SLTPoint agent_fs_point;
        Vec2d corner;
        agent_fs_point.set_t(state.time_stamp);
        agent_fs_point.set_s(fs_point[offset * num_agent + j].x());
        agent_fs_point.set_l(fs_point[offset * num_agent + j].y());
        agent_state_vertices.emplace_back(agent_fs_point);
        corner.set_x(fs_point[offset * num_agent + j].x());
        corner.set_y(fs_point[offset * num_agent + j].y());
        box_corners[j] = corner;
        agent_corner->set_x(corner.x());
        agent_corner->set_y(corner.y());
      }
      AABox2d agent_box(box_corners, num_agent);
      offset++;
      agent_state.agent_box_set[i] = agent_box;
      if ((agent_box.max_y() < min_target_l) || (agent_box.min_y() > max_target_l) ||
          (agent_box.min_x() > ego_front_consider_obstacle_distance_ + ego_frenet_state_.s()) ||
          (agent_box.max_x() < ego_box.min_x() - kLongitReservedBuffer)) {
        time_and_corners->set_enable_use(false);
        index++;
        continue;
      } else {
        agent_state.agent_boxs_set.insert(std::make_pair(index, agent_box));
        time_and_corners->set_enable_use(true);
        index++;
      }
      // agent_state.frenet_vertices.emplace_back(agent_state_vertices);
      if (state.time_stamp <= kDefaultConsiderDynamicObstacleTajsTime) {
        if (agent_box.max_x() > max_x) {
          max_x = agent_box.max_x();
        }
        if (agent_box.max_y() > max_y) {
          max_y = agent_box.max_y();
        }
        if (agent_box.min_x() < min_x) {
          min_x = agent_box.min_x();
        }
        if (agent_box.min_y() < min_y) {
          min_y = agent_box.min_y();
        }
      }
    }
// #else
//     const auto& fs_point = fs_iter->second;
//     for (int i = 0; i < state_iter->second.size(); ++i) {
//       const auto& state = state_iter->second[i];
//       std::vector<SLTPoint> agent_state_vertices;
//       std::array<planning_math::Vec2d, 8> box_corners;
//       for (int j = 0; j < num_agent; ++j) {
//         SLTPoint agent_fs_point;
//         Vec2d corner;
//         agent_fs_point.set_t(state.time_stamp);
//         agent_fs_point.set_s(fs_point[offset * num_agent + j].x());
//         agent_fs_point.set_l(fs_point[offset * num_agent + j].y());
//         agent_state_vertices.emplace_back(agent_fs_point);
//         corner.set_x(fs_point[offset * num_agent + j].x());
//         corner.set_y(fs_point[offset * num_agent + j].y());
//         box_corners[j] = corner;
//       }

//       // agent_state.frenet_vertices.emplace_back(agent_state_vertices);
//       AABox2d agent_box(box_corners, num_agent);
//       agent_state.agent_box_set[i] = agent_box;
//       offset++;
//       if (state.time_stamp <= kDefaultConsiderDynamicObstacleTajsTime) {
//         if (agent_box.max_x() > max_x) {
//           max_x = agent_box.max_x();
//         }
//         if (agent_box.max_y() > max_y) {
//           max_y = agent_box.max_y();
//         }
//         if (agent_box.min_x() < min_x) {
//           min_y = agent_box.min_y();
//         }
//         if (agent_box.min_y() < min_y) {
//           min_y = agent_box.min_y();
//         }
//       }
//     }
// #endif
    if (agent_state.agent_boxs_set.empty()) {
      continue;
    }
    max_box_corners.emplace_back(Vec2d(min_x, max_y));
    max_box_corners.emplace_back(Vec2d(max_x, max_y));
    max_box_corners.emplace_back(Vec2d(max_x, min_y));
    max_box_corners.emplace_back(Vec2d(min_x, min_y));

    for (int i = 0; i < max_box_corners.size(); ++i) {
      const auto& corner = max_box_corners[i];
      auto max_box_corner = agent_time_corner->mutable_max_box_corners()->Add();
      max_box_corner->set_x(corner.x());
      max_box_corner->set_y(corner.y());
    }

    AABox2d max_AABBbox(max_box_corners);
    agent_state.max_agent_box = max_AABBbox;
    surround_forward_trajs_state_.emplace_back(agent_state);
  }

  // * Virtual agengt spatio temporal info
  virtual_agents_st_info_.clear();
  if (!virtual_agents_.empty()) {
    for(size_t i = 0; i < virtual_agents_.size(); ++i) {
      VirtualAgentSpatioTemporalInFo virtual_agent_st_info;
      const auto agent = virtual_agents_[i];
      Point2D agent_point(agent->x(), agent->y());
      Point2D frenet_point;
      if (!current_lane_coord_->XYToSL(agent_point,
                                       frenet_point)) {
        // LOG_DEBUG(
        //     "StateTransformForInputData::virtual agent in current lane failed!");
      }
      virtual_agent_st_info.agent_id = agent->agent_id();
      virtual_agent_st_info.frenet_slt_info.set_s(frenet_point.x);
      virtual_agent_st_info.frenet_slt_info.set_l(frenet_point.y);
      virtual_agent_st_info.frenet_slt_info.set_t(0.0);
      virtual_agents_st_info_.emplace_back(virtual_agent_st_info);
    }
  }

  return;
}

void SLTGridMapAdapter::GetVehicleVertices(
    const State &state,
    std::vector<planning_math::Vec2d> *vertices) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double angle = state.angle;

  double cos_theta = cos(angle);
  double sin_theta = sin(angle);

  double c_x = state.vec_position.x() + vehicle_param.rear_axle_to_center * cos_theta;
  double c_y = state.vec_position.y() + vehicle_param.rear_axle_to_center * sin_theta;

  double d_wx = vehicle_param.width * 0.5 * sin_theta;
  double d_wy = vehicle_param.width * 0.5 * cos_theta;
  double d_lx = vehicle_param.length * 0.5 * cos_theta;
  double d_ly = vehicle_param.length * 0.5 * sin_theta;

  // Counterclockwise from left-front vertex
  vertices->emplace_back(Vec2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
    // vertices->emplace_back(Vec2d(c_x - d_wx, c_y + d_wy));
  vertices->emplace_back(Vec2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
    // vertices->emplace_back(Vec2d(c_x - d_lx, c_y - d_ly));
  vertices->emplace_back(Vec2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
    // vertices->emplace_back(Vec2d(c_x + d_wx, c_y - d_wy));
  vertices->emplace_back(Vec2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
    // vertices->emplace_back(Vec2d(c_x + d_lx, c_y + d_ly));

  return;
}

void SLTGridMapAdapter::GetAgentVertices(const agent::Agent &agent,
                                         const State &state,
                                         std::vector<planning_math::Vec2d> *vertices) {
  double angle = state.angle;

  double cos_theta = cos(angle);
  double sin_theta = sin(angle);

  double c_x = state.vec_position.x();
  double c_y = state.vec_position.y();

  double d_wx = agent.width() * 0.5 * sin_theta;
  double d_wy = agent.width() * 0.5 * cos_theta;
  double d_lx = agent.length() * 0.5 * cos_theta;
  double d_ly = agent.length() * 0.5 * sin_theta;

  // Counterclockwise from left-front vertex
  vertices->emplace_back(Vec2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
  vertices->emplace_back(Vec2d(c_x - d_wx, c_y + d_wy));
  vertices->emplace_back(Vec2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
  vertices->emplace_back(Vec2d(c_x - d_lx, c_y - d_ly));
  vertices->emplace_back(Vec2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
  vertices->emplace_back(Vec2d(c_x + d_wx, c_y - d_wy));
  vertices->emplace_back(Vec2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
  vertices->emplace_back(Vec2d(c_x + d_lx, c_y + d_ly));

  return;
}

void SLTGridMapAdapter::AgentVerticesTransform(
    const std::vector<planning_math::Vec2d>& global_point_vec,
    std::vector<planning_math::Vec2d>* fs_point_vec) {
  int point_num = global_point_vec.size();
  planning_math::Vec2d fs_point;
  Point2D frenet_point;
  Point2D agent_point;

  for (int i = 0; i < point_num; ++i) {
    agent_point.x = global_point_vec[i].x();
    agent_point.y = global_point_vec[i].y();
    if (!current_lane_coord_->XYToSL(agent_point,
                                     frenet_point)) {
      // LOG_DEBUG(
      //     "AgentVerticesTransform::Agent Pose in current lane failed!");
      frenet_point.x = 200.0;
      frenet_point.y = 10.0;
    }

    fs_point.set_x(frenet_point.x);
    fs_point.set_y(frenet_point.y);
    fs_point_vec->emplace_back(fs_point);
  }

  return;
}

void SLTGridMapAdapter::EgoVerticesTransform(
    const std::array<planning_math::Vec2d, 4> &global_point_vec,
    std::array<planning_math::Vec2d, 4> &fs_point_vec) {
  int point_num = global_point_vec.size();
  planning_math::Vec2d fs_point;
  Point2D frenet_point;
  Point2D ego_point;

  for (int i = 0; i < point_num; ++i) {
    ego_point.x = global_point_vec[i].x();
    ego_point.y = global_point_vec[i].y();
    if (!current_lane_coord_->XYToSL(ego_point,
                                     frenet_point)) {
      // LOG_DEBUG(
      //     "EgoVerticesTransform::Ego Pose in current lane failed!");
      frenet_point.x = 200.0;
      frenet_point.y = 10.0;
    }

    fs_point_vec[i] = Vec2d(frenet_point.x, frenet_point.y);
  }

  return;
}

}