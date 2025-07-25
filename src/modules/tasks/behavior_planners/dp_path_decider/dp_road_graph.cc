#include "dp_road_graph.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <ostream>
#include <utility>
#include <vector>
#include "basic_types.pb.h"
#include "config/message_type.h"
#include "define/geometry.h"
#include "dp_road_graph.pb.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "ifly_time.h"
#include "log.h"
#include "math/curve1d/quintic_polynomial_curve1d.h"
#include "math_lib.h"
#include "task_interface/lane_borrow_decider_output.h"
#include "modules/context/planning_context.h"
namespace {
constexpr double kMaxLateralRange = 5.0;
constexpr double kMaxLongitRange = 70.0;
constexpr double kMinLongitRange = 25.0;
constexpr double kMaxNudgingSpeed = 4.2;  // 15 kph
};                                        // namespace

namespace planning {
bool DPRoadGraph::Execute() {
  // double time_stamp_start = IflyTime::Now_ms();
  // ProcessEnvInfos();
  // SetSampleParams();
  // SetDPCostParams();
  // double start_time = IflyTime::Now_ms();
  // SampleLanes();
  // DPSearchPath();

  // double time_stamp_end = IflyTime::Now_ms();
  // dp_cost_time_ = time_stamp_end - start_time;
  // LogDebugInfo();
  // DPPathClear();

  return true;
}
bool DPRoadGraph::ProcessEnvInfos(const LaneBorrowDeciderOutput* lane_borrow_output) {
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& agents = dynamic_world->agent_manager()->GetAllCurrentAgents();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ptr_ = virtual_lane_manager->get_current_lane();
  current_reference_path_ptr_ = current_lane_ptr_->get_reference_path();
  const auto& current_frenet_coord =
      current_reference_path_ptr_->get_frenet_coord();
  left_lane_ptr_ = virtual_lane_manager->get_left_lane();
  right_lane_ptr_ = virtual_lane_manager->get_right_lane();
  ego_frenet_state_ = session_->environmental_model()
                          .get_reference_path_manager()
                          ->get_reference_path_by_current_lane()
                          ->get_frenet_ego_state();
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  ego_s_ = ego_frenet_state_.s();
  ego_l_ = ego_frenet_state_.l();
  ego_v_ = ego_frenet_state_.velocity();
  ego_cartes_state.x =
      session_->environmental_model().get_ego_state_manager()->ego_carte().x;
  ego_cartes_state.y =
      session_->environmental_model().get_ego_state_manager()->ego_carte().y;
  v_cruise_ =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();
  vehicle_width_ =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  vehicle_length_ =
      VehicleConfigurationContext::Instance()->get_vehicle_param().length;
  const auto& planning_init_point =
      current_reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  init_sl_point_.l = planning_init_point.frenet_state.r;
  init_sl_point_.s = planning_init_point.frenet_state.s;

  // env obs filtering
  obstacles_info_.clear();
  static_obstacles_box_.clear();
  flatted_dynamic_obstacles_box_.clear();

  dp_cost_time_ = 0.0;
  iter_num_ = 0;
  path_cost_time_ = 0;
  safety_cost_time_ = 0;
  stitch_cost_time_ = 0;

  // env lanes
  if (current_lane_ptr_ == nullptr) {
    return false;
  }
  if (agents.empty()) {
    return true;
  }
  const auto& obstacles = current_reference_path_ptr_->get_obstacles();
  const auto& agent_mgr = session_->environmental_model().get_agent_manager();
  double concerned_left_l = 0.;
  double concerned_right_l = 0.;
  BorrowDirection dir = lane_borrow_output->borrow_direction;
  if (current_lane_ptr_ != nullptr) {
    concerned_left_l =
        current_lane_ptr_->width() * 0.5;
    concerned_right_l =
        -current_lane_ptr_->width() * 0.5;
  }
  if (left_lane_ptr_ != nullptr && dir == LEFT_BORROW) {
    concerned_left_l += left_lane_ptr_->width();
  }
  if (right_lane_ptr_ != nullptr && dir == RIGHT_BORROW) {
    concerned_right_l -= right_lane_ptr_->width();
  }
  const auto& lat_obstacle_decision = session_->planning_context()
                                        .lateral_obstacle_decider_output()
                                        .lat_obstacle_decision;
  for (const auto& obstacle : obstacles) {
    const auto& id = obstacle->obstacle()->id();
    const auto& agent = agent_mgr->GetAgent(id);
    //  continue
    if (agent == nullptr) {
      continue;
    }
    if (agent->agent_decision().agent_decision_type() ==
        agent::AgentDecisionType::IGNORE) {  // ignore 忽略
      continue;
    }
    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {  // 非视觉忽略
      continue;
    }
    //   const auto lat_obs_iter = lat_obstacle_decision.find(id);
    // if (lat_obs_iter != lat_obstacle_decision.end() &&
    //     lat_obs_iter->second != LatObstacleDecisionType::IGNORE) {
    //   continue;
    // }
    // static and dynamic sl current time
    std::vector<planning_math::Vec2d> obs_corners;
    const auto& obs_box = agent->box();
    obs_corners = obs_box.GetAllCorners();
    std::vector<double> agent_sl_boundary(4);
    agent_sl_boundary.at(0) = std::numeric_limits<double>::lowest();
    agent_sl_boundary.at(1) = std::numeric_limits<double>::max();
    agent_sl_boundary.at(2) = std::numeric_limits<double>::lowest();
    agent_sl_boundary.at(3) = std::numeric_limits<double>::max();
    for (size_t i = 0; i < obs_corners.size(); ++i) {
      double project_s = 0.0, project_l = 0.0;
      current_frenet_coord->XYToSL(
          obs_corners[i].x(), obs_corners[i].y(), &project_s,
          &project_l);  // 这是投影在路径上的 障碍物角点
      agent_sl_boundary.at(3) = std::fmin(agent_sl_boundary.at(3), project_l);// l right
      agent_sl_boundary.at(2) = std::fmax(agent_sl_boundary.at(2), project_l);// l left
      agent_sl_boundary.at(1) = std::fmin(agent_sl_boundary.at(1), project_s);// s start
      agent_sl_boundary.at(0) = std::fmax(agent_sl_boundary.at(0), project_s);// s end
    }
    // lateral filter
    if(agent_sl_boundary.at(3) > concerned_left_l || agent_sl_boundary.at(2) < concerned_right_l){
      continue;
    }
    if (agent->is_static()) {
      // sl meaasge
      if (std::fabs(0.5 * (agent_sl_boundary[3] + agent_sl_boundary[2]) -
                    ego_l_) >
          kMaxLateralRange) {  // tmp: should according to boundary
        continue;
      }
      // ahead 70m after 15m static filtered
      if (agent_sl_boundary[0] > ego_s_ + kMaxLongitRange ||
          agent_sl_boundary[1] < kMinLongitRange) {  // tmp: should kvalue
        continue;
      }
      StaticObstacleInfo static_obs_info{
          agent->agent_id(), agent_sl_boundary.at(1), agent_sl_boundary.at(0),
          agent_sl_boundary.at(3), agent_sl_boundary.at(2)};
      obstacles_info_.emplace_back(
          std::move(static_obs_info));  // just log info
      static_obstacles_box_.emplace_back(obs_box);
    } else if (obstacle->frenet_velocity_s() <
               kMaxNudgingSpeed) {  // slow dynamic filter
                                    // filtered backward dynamic obs
      if (!(agent_sl_boundary[3] > ego_frenet_boundary_.l_end ||
            agent_sl_boundary[2] < ego_frenet_boundary_.l_start) &&
          agent_sl_boundary[0] < ego_frenet_boundary_.s_start) {
        continue;
      }

      if (agent_sl_boundary[0] + vehicle_length_ <
              ego_frenet_boundary_.s_start &&
          obstacle->frenet_velocity_s() < ego_v_) {
        continue;
      }

      std::vector<planning_math::Vec2d> predict_corners;  // corner points
      const auto& trajectories = agent->trajectories();
      if (trajectories[0].empty()) {  // one trajectory 26 points 0.2s internal
        continue;
      }
      for (size_t i = 0; i < trajectories[0].size(); i = i + 3) {
        // for (size_t i = 0; i <=18;  i=i+3) {
        trajectory::TrajectoryPoint point =
            trajectories[0][i];  // 0s 0.6s ... 4.8s 10   i=18  3.6s
        Box2d obs_box(Vec2d(point.x(), point.y()), point.theta(),
                      agent->length(),
                      agent->width());  // 本时刻对应的box
        // log
        std::vector<planning_math::Vec2d> corners;
        corners = obs_box.GetAllCorners();
        for (size_t i = 0; i < corners.size(); ++i) {
          predict_corners.emplace_back(corners[i]);  // add  points
        }
      }
      // points to polygon
      planning_math::Polygon2d agent_prediction_polygon;
      planning_math::Polygon2d::ComputeConvexHull(predict_corners,
                                                  &agent_prediction_polygon);
      planning_math::Box2d flatted_box =
          agent_prediction_polygon.MinAreaBoundingBox();
      flatted_dynamic_obstacles_box_.emplace_back(flatted_box);

      // log
      std::vector<planning_math::Vec2d> flatted_corners;
      flatted_corners = flatted_box.GetAllCorners();
      std::vector<double> flatted_sl_boundary(4);
      flatted_sl_boundary.at(0) = std::numeric_limits<double>::lowest();
      flatted_sl_boundary.at(1) = std::numeric_limits<double>::max();
      flatted_sl_boundary.at(2) = std::numeric_limits<double>::lowest();
      flatted_sl_boundary.at(3) = std::numeric_limits<double>::max();
      for (size_t i = 0; i < flatted_corners.size(); ++i) {
        predict_corners.emplace_back(flatted_corners[i]);  // points
        double project_s = 0.0, project_l = 0.0;
        current_frenet_coord->XYToSL(
            flatted_corners[i].x(), flatted_corners[i].y(), &project_s,
            &project_l);  // 这是投影在路径上的 障碍物角点
        flatted_sl_boundary.at(3) =
            std::fmin(flatted_sl_boundary.at(3), project_l);
        flatted_sl_boundary.at(2) =
            std::fmax(flatted_sl_boundary.at(2), project_l);
        flatted_sl_boundary.at(1) =
            std::fmin(flatted_sl_boundary.at(1), project_s);
        flatted_sl_boundary.at(0) =
            std::fmax(flatted_sl_boundary.at(0), project_s);
      }
      StaticObstacleInfo flatted_obs_info{
          agent->agent_id(), flatted_sl_boundary.at(1),
          flatted_sl_boundary.at(0), flatted_sl_boundary.at(3),
          flatted_sl_boundary.at(2)};

      obstacles_info_.emplace_back(flatted_obs_info);
    } else {  // high speed
      continue;
    }
  }

  return true;
}

bool DPRoadGraph::DPSearchPath(const LaneBorrowStatus lane_borrow_status) {
  // get min cost path
  double dp_search_start_time = IflyTime::Now_ms();
  std::vector<std::vector<DPRoadGraphNode>> graph_nodes(sampled_points_.size());
  if (sampled_points_.size() < 2) {
    return false;
  }
  // TrajectoryCost trajectory_cost(
  //     init_sl_point_, sample_left_boundary_, sample_right_boundary_, config_,
  //     ref_path_curve_, flatted_dynamic_obstacles_box_, static_obstacles_box_,
  //     left_lane_ptr_, right_lane_ptr_, current_lane_ptr_, ego_frenet_state_);
  TrajectoryCost trajectory_cost(
      config_, session_, init_sl_point_, ref_path_curve_,
      flatted_dynamic_obstacles_box_, static_obstacles_box_);

  trajectory_cost.SetCostParams(coeff_l_cost_, coeff_dl_cost_, coeff_ddl_cost_,
                                path_resolution_, coeff_end_l_cost_,
                                coeff_collision_cost_, collision_distance_,
                                coeff_stitch_cost_);
  graph_nodes[0].emplace_back(init_sl_point_, nullptr, ComparableCost());

  const double lateral_resolution = l_range_;
  const double d_max = s_range_ * std::tan(theta_max_);

  // 转换为横向采样点数量
  const int d = ceil(d_max / lateral_resolution);
  const int valid_d = std::max(1, d);

  for (std::size_t level = 1; level < sampled_points_.size(); ++level) {
    const auto& prev_dp_nodes = graph_nodes[level - 1];
    const auto& level_points = sampled_points_[level];
    // 预排序前级节点的横向位置
    std::vector<double> prev_l_cache;
    if (prev_l_cache.size() != prev_dp_nodes.size()) {
      prev_l_cache.clear();
      for (const auto& node : prev_dp_nodes) {
        prev_l_cache.push_back(node.sl_point.l);
      }
      std::sort(prev_l_cache.begin(), prev_l_cache.end());
    }
    for (const auto& cur_point : level_points) {
      graph_nodes[level].emplace_back(cur_point, nullptr);
      auto& cur_node = graph_nodes[level].back();
      const double cur_l = cur_point.l;
      auto lower = std::lower_bound(prev_l_cache.begin(), prev_l_cache.end(),
                                    cur_l - d_max);
      auto upper = std::upper_bound(prev_l_cache.begin(), prev_l_cache.end(),
                                    cur_l + d_max);
      const int start_idx = std::distance(prev_l_cache.begin(), lower);
      const int end_idx = std::distance(prev_l_cache.begin(), upper);
      for (int k = start_idx; k < end_idx; ++k) {
        const auto& prev_dp_node = prev_dp_nodes[k];
        const auto& prev_sl_point = prev_dp_node.sl_point;

        if (std::abs(prev_sl_point.l - cur_l) > d_max) {
          continue;
        }
        if (prev_dp_node.min_cost_.has_collision_ == true ||
            prev_dp_node.min_cost_.out_boundary_ == true) {
          continue;
        }

        // for (const auto& prev_dp_node : prev_dp_nodes) {
        //   const auto& prev_sl_point = prev_dp_node.sl_point;
        double init_dl = 0.0;
        double init_ddl = 0.0;
        planning_math::QuinticPolynomialCurve1d curve(
            prev_sl_point.l, init_dl, init_ddl, cur_point.l, 0.0, 0.0,
            cur_point.s - prev_sl_point.s);  // s=0 s=sf
        const auto cost = trajectory_cost.Calculate(
                              curve, prev_sl_point.s, cur_point.s, level,
                              sampled_points_.size() - 1, lane_borrow_status) +
                          prev_dp_node.min_cost_;
        cur_node.UpdateCost(cost, &prev_dp_node, curve);
        // record
        iter_num_++;
      }
    }
  }
  path_cost_time_ = trajectory_cost.path_cost_time_;
  stitch_cost_time_ = trajectory_cost.stitch_cost_time_;
  safety_cost_time_ = trajectory_cost.safety_cost_time_;
  // find path
  DPRoadGraphNode fake_head;
  min_cost_path_.clear();
  dp_selected_points_.clear();
  // for (const auto& cur_dp_node :
  //      graph_nodes.back()) {  // 实际只是选择到达最后一层中代价最小的一个节点
  //   fake_head.UpdateCost(cur_dp_node.min_cost_, &cur_dp_node,
  //                        cur_dp_node.min_cost_curve_);
  // }
  for (int i = graph_nodes.size() - 1; i >= 2; --i) {
    for (const auto& cur_dp_node :
         graph_nodes[i]) {  // 实际只是选择到达最后一层中代价最小的一个节点
      fake_head.UpdateCost(cur_dp_node.min_cost_, &cur_dp_node,
                           cur_dp_node.min_cost_curve_);
    }
    if (fake_head.min_cost_.has_collision_ == false &&
        fake_head.min_cost_.out_boundary_ == false) {
      break;
    } else {
      fake_head.Reset();
    }
  }

  const auto* min_cost_node = &fake_head;
  while (min_cost_node->min_cost_prev_node_) {
    min_cost_node = min_cost_node->min_cost_prev_node_;
    min_cost_path_.push_back(*min_cost_node);
  }
  std::reverse(min_cost_path_.begin(), min_cost_path_.end());
  for (const auto& node : min_cost_path_) {
    Point2D sl_cart;
    dp_selected_points_.emplace_back(node.sl_point);
  }
  double dp_search_end_time = IflyTime::Now_ms();
  dp_search_cost_time_ = dp_search_end_time - dp_search_start_time;
  return true;
}
bool DPRoadGraph::FinedReferencePath() {
  std::vector<SLPoint> frenet_dp_path;

  double accumulated_s = init_sl_point_.s;
  const double path_resolution = 2.0;
  refined_paths_.clear();
  if (min_cost_path_.size() <= 1) {
    return false;
  }
  for (std::size_t i = 1; i < min_cost_path_.size(); i++) {
    const auto& prev_node = min_cost_path_[i - 1];
    const auto& current_node = min_cost_path_[i];
    if (prev_node.min_cost_.has_collision_ == true ||
        prev_node.min_cost_.out_boundary_ == true) {
      return false;
    }
    const double path_s_length = current_node.sl_point.s - prev_node.sl_point.s;
    double current_s = 0.0;
    const auto& curve = current_node.min_cost_curve_;
    // donnot delete, discard for pybind
    const auto& current_frenet_coord =
        current_reference_path_ptr_->get_frenet_coord();
    while (current_s + path_resolution * 0.5 < path_s_length) {
      // current_s += path_resolution;
      const double l = curve.Evaluate(0, current_s);
      const double dl = curve.Evaluate(1, current_s);
      const double ddl = curve.Evaluate(2, current_s);
      const double dddl = curve.Evaluate(3, current_s);
      const double theta = std::atan2(dl, 1);  // 计算heading角度 不是全局的
      const double kappa = curve.EvaluateKappa(current_s);  // 获取曲率
      const double denominator = std::pow(1 + dl * dl, 1.5);
      const double numerator1 = dddl;
      const double numerator2 = 3 * ddl * dl * ddl / std::pow(1 + dl * dl, 2.5);
      const double dkappa = numerator1 / denominator - numerator2;
      const double ddkappa = 0.;

      const SLPoint sl_point(accumulated_s + current_s, l);

      frenet_dp_path.emplace_back(sl_point);// no use?

      Point2D sl_cart(0, 0);  // nonsene
      // donnot delete, discard for pybind
      current_frenet_coord->SLToXY(accumulated_s + current_s, l, &sl_cart.x,
                                   &sl_cart.y);
      planning_math::PathPoint path_point{sl_cart.x,  sl_cart.y, sl_point.s,
                                          sl_point.l, theta,     kappa,
                                          dkappa,     ddkappa};
      refined_paths_.emplace_back(path_point);
      current_s += path_resolution;
    }
    if (i == min_cost_path_.size() - 1) {
      accumulated_s += current_s;
    } else {
      accumulated_s += path_s_length;
    }
  }

  return true;
}
bool DPRoadGraph::CartSpline(
    LaneBorrowDeciderOutput* lane_borrow_decider_output) {
  std::vector<double> s_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  if (refined_paths_.size() < 2) {
    return false;
  }
  last_frame_paths_ =
      refined_paths_;  // last paths is not  updated until CartSpline

  for (const auto& path_point : refined_paths_) {
    double cur_s = path_point.s();
    s_vec.emplace_back(cur_s);
    x_vec.emplace_back(path_point.x());
    y_vec.emplace_back(path_point.y());
  }
  ref_path_curve_.x_vec = x_vec;
  ref_path_curve_.y_vec = y_vec;
  ref_path_curve_.s_vec = s_vec;
  ref_path_curve_.x_s_spline.set_points(s_vec, x_vec);
  ref_path_curve_.y_s_spline.set_points(s_vec, y_vec);

  lane_borrow_decider_output->dp_path_coord =
      ConstructLaneBorrowKDPath(x_vec, y_vec);
  lane_borrow_decider_output->dp_path_ref = ref_path_curve_;
  return true;
}
bool DPRoadGraph::SetSampleParams(LaneBorrowStatus lane_borrow_status) {
  double total_length = ego_s_ + std::fmax(ego_v_ * 6.0,  // 6s
                                           config_.min_sample_distance);
  total_length_ = total_length;  // set_total_length(total_length); // length is
                                 // not update in pybind, same as .logs

  // const double level_distance = pnc::mathlib::Clamp(
  //     ego_v_ * config_.sample_forward_time, config_.min_level_distance,
  //     config_.max_level_distance);  // s_range  is update with v_cruise_ in
  //                                   // logs, but not in pybind

  s_range_ = (total_length - ego_s_)/3.0;
  l_range_ = 0.5;

  if (current_lane_ptr_ != nullptr) {
    sample_left_boundary_ =
        current_lane_ptr_->width() * 0.5 - vehicle_width_ * 0.5;
    sample_right_boundary_ =
        -current_lane_ptr_->width() * 0.5 + vehicle_width_ * 0.5;
  }
  if (left_lane_ptr_ != nullptr) {
    sample_left_boundary_ += left_lane_ptr_->width();  // unused
  }
  if (right_lane_ptr_ != nullptr) {
    sample_right_boundary_ -= right_lane_ptr_->width();  // unused
  }

  LaneBorrowStatus lane_borrow_state;
  lane_borrow_state = lane_borrow_status;
  if (lane_borrow_state == kNoLaneBorrow ||
      lane_borrow_state == kLaneBorrowDriving) {
    theta_max_ = 0.15;
  } else if (lane_borrow_state == kLaneBorrowCrossing) {
    theta_max_ = 0.12;
  } else if (lane_borrow_state == kLaneBorrowBackOriginLane) {
    theta_max_ = 0.1;
  }

  return true;
}
bool DPRoadGraph::SetDPCostParams(LaneBorrowStatus lane_borrow_status) {
  LaneBorrowStatus lane_borrow_state;
  lane_borrow_state = lane_borrow_status;
  if (lane_borrow_state == kNoLaneBorrow ||
      lane_borrow_state == kLaneBorrowDriving) {
    coeff_l_cost_ = config_.coeff_l_cost;
    coeff_dl_cost_ = config_.coeff_dl_cost;
    coeff_ddl_cost_ = config_.coeff_ddl_cost;
    path_resolution_ = config_.path_resolution;
    coeff_end_l_cost_ = config_.coeff_end_l_cost;
    coeff_collision_cost_ = config_.coeff_collision_cost;
    collision_distance_ = config_.collision_distance;
    coeff_stitch_cost_ = config_.coeff_stitch_cost;
  } else if (lane_borrow_state == kLaneBorrowCrossing) {
    coeff_l_cost_ = config_.coeff_l_cost2;
    coeff_dl_cost_ = config_.coeff_dl_cost2;
    coeff_ddl_cost_ = config_.coeff_ddl_cost2;
    path_resolution_ = config_.path_resolution2;
    coeff_end_l_cost_ = config_.coeff_end_l_cost2;
    coeff_collision_cost_ = config_.coeff_collision_cost2;
    collision_distance_ = config_.collision_distance2;
    coeff_stitch_cost_ = config_.coeff_stitch_cost2;
  } else if (lane_borrow_state == kLaneBorrowBackOriginLane) {
    coeff_l_cost_ = config_.coeff_l_cost3;
    coeff_dl_cost_ = config_.coeff_dl_cost3;
    coeff_ddl_cost_ = config_.coeff_ddl_cost3;
    path_resolution_ = config_.path_resolution3;
    coeff_end_l_cost_ = config_.coeff_end_l_cost3;
    coeff_collision_cost_ = config_.coeff_collision_cost3;
    collision_distance_ = config_.collision_distance3;
    coeff_stitch_cost_ = config_.coeff_stitch_cost3;
  }
  return true;
}
bool DPRoadGraph::SampleLanes(
    LaneBorrowDeciderOutput* lane_borrow_decider_output) {
  sampled_points_.clear();
  double accumulated_s = init_sl_point_.s;
  double prev_s = accumulated_s;
  sampled_points_.insert(sampled_points_.begin(),
                         std::vector<SLPoint>{init_sl_point_});
  for (size_t i = 0; accumulated_s < total_length_; ++i) {
    accumulated_s += s_range_;
    if (accumulated_s + s_range_ / 2.0 > total_length_) {
      accumulated_s = total_length_;
    }
    const double s_step = std::fmin(total_length_, accumulated_s);
    if (std::fabs(s_step - prev_s) < 1.0) {
      continue;
    }
    prev_s = s_step;
    // donnot delete more precise but abort for pybind

    // lateral sample  lanes
    double sample_left_boundary = 0.0;
    double sample_right_boundary = 0.0;
    // avaliable lane boundary
    if (current_lane_ptr_ == nullptr && left_lane_ptr_ == nullptr &&
        right_lane_ptr_ == nullptr) {
      LOG_ERROR("No avaliable lanes");
      return false;
    }
    double aheads = s_step - ego_s_;
    if (current_lane_ptr_ != nullptr) {
      sample_left_boundary =
          current_lane_ptr_->width_by_s(s_step) * 0.5 - vehicle_width_ * 0.5;
      sample_right_boundary =
          -current_lane_ptr_->width_by_s(s_step) * 0.5 + vehicle_width_ * 0.5;
    }
    if (left_lane_ptr_ != nullptr) {  // extend sample boundary
      if (lane_borrow_decider_output->lane_borrow_state ==
              kLaneBorrowCrossing ||
          i == 0) {
        sample_left_boundary += left_lane_ptr_->width_by_s(s_step) * 0.7;
      } else {
        sample_left_boundary += left_lane_ptr_->width_by_s(s_step) * 0.5;
      }
    }
    if (right_lane_ptr_ != nullptr) {
      if (lane_borrow_decider_output->lane_borrow_state ==
              kLaneBorrowCrossing ||
          i == 0) {
        sample_right_boundary -= right_lane_ptr_->width_by_s(s_step) * 0.7;
      } else {
        sample_right_boundary -= right_lane_ptr_->width_by_s(s_step) * 0.5;
      }
    }
    // if (left_lane_ptr_ != nullptr) {
    //   sample_left_boundary += left_lane_ptr_->width_by_s(s_step) * 0.5;
    // }
    // if (right_lane_ptr_ != nullptr) {
    //   sample_right_boundary -= right_lane_ptr_->width_by_s(s_step) * 0.5;
    // }
    // 间隔不变
    if (lane_borrow_decider_output->borrow_direction == LEFT_BORROW) {
      // l_range_ = 0.5;
      sample_right_boundary = 0.0;
    } else if (lane_borrow_decider_output->borrow_direction == RIGHT_BORROW) {
      // l_range_ = 0.5;
      sample_left_boundary = 0.0;
    } else {
      // l_range_ = 1.0;
    }
    // 考虑道路边缘和物理隔离
    ReferencePathPoint refpath_pt{};
    double distance_to_left_road_border = 100;
    double distance_to_right_road_border = 100;
    if (current_reference_path_ptr_ != nullptr &&
        current_reference_path_ptr_->get_reference_point_by_lon(ego_s_,
                                                                refpath_pt)) {
      distance_to_left_road_border = refpath_pt.distance_to_left_road_border;
      distance_to_right_road_border = refpath_pt.distance_to_right_road_border;
    }
    sample_right_boundary =
        std::max(sample_right_boundary, -distance_to_right_road_border);
    sample_left_boundary =
        std::min(sample_left_boundary, distance_to_left_road_border);

    // slice lateral range
    std::vector<double> samples_l;
    // 负方向采样
    for (double l = -l_range_; l >= sample_right_boundary; l -= l_range_) {
      samples_l.push_back(l);
    }
    std::reverse(samples_l.begin(), samples_l.end());
    // 正方向采样（包括0）
    for (double l = -1e-6; l <= sample_left_boundary; l += l_range_) {
      samples_l.push_back(l);
    }
    // sampled sl points
    double l_step = 0;
    std::vector<SLPoint> level_points;
    const auto& current_frenet_coord =
        current_reference_path_ptr_->get_frenet_coord();
    for (size_t j = 0; j < samples_l.size(); j++) {
      l_step = samples_l[j];
      SLPoint sl_point;
      sl_point.l = l_step;
      sl_point.s = s_step;
      bool has_overlap = false;
      for (const auto& obs : obstacles_info_) {
        if (sl_point.s > obs.s_start - vehicle_length_ * 0.5 &&
            sl_point.s < obs.s_end + vehicle_length_ * 0.5 &&
            sl_point.l < obs.l_end + vehicle_width_ * 0.5 &&
            sl_point.l > obs.l_start - vehicle_width_ * 0.5) {
          has_overlap = true;
        }
      }
      if (has_overlap) {
        continue;
      }
      level_points.push_back(sl_point);
    }
    if (!level_points.empty()) {
      sampled_points_.emplace_back(level_points);
    }
  }

  return true;
}
bool DPRoadGraph::LastFramePath() {
  if (ref_path_curve_.s_vec.empty()) {
    return false;
  }
  const double s_start = ref_path_curve_.s_vec.front();
  const double s_end = ref_path_curve_.s_vec.back();
  const auto& planning_init_point =
      current_reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  double init_x = planning_init_point.x;
  double init_y = planning_init_point.y;
  double init_s = init_sl_point_.s;
  double init_l = init_sl_point_.l;
  double theta = planning_init_point.heading_angle;
  planning_math::PathPoint init_path_point{init_x, init_y, init_s, init_l,
                                           theta,  0,      0,      0};
  const auto& current_frenet_coord =
      current_reference_path_ptr_->get_frenet_coord();
  // Point2D cart_point;
  // current_frenet_coord->SLToXY(sl_point, cart_point);
  Eigen::Vector2d planning_init_to_proj(init_x, init_y);
  pnc::spline::Projection projection_tool;
  projection_tool.CalProjectionPoint(ref_path_curve_.x_s_spline,
                                     ref_path_curve_.y_s_spline, s_start, s_end,
                                     planning_init_to_proj);
  // 检查投影结果
  if (!projection_tool.GetOutput().success) {
    return false;
  }
  const double s_proj =
      projection_tool.GetOutput().s_proj;  // s wrt. last frenet frame
  // refined_paths_ invalid this frame
  refined_paths_ = last_frame_paths_;
  auto it = std::lower_bound(
      refined_paths_.begin(), refined_paths_.end(), s_proj,
      [](const planning_math::PathPoint& pt, double s) { return pt.s() < s; });
  if (it != refined_paths_.begin() && it != refined_paths_.end()) {
    auto prev = it - 1;
    if (std::abs(prev->s() - s_proj) <= std::abs(it->s() - s_proj)) {
      it = prev;
    }
  } else if (it == refined_paths_.end()) {
    --it;
  }
  // 删除 it 及其之前的所有点
  if (std::next(it) != refined_paths_.end()) {
    refined_paths_.erase(refined_paths_.begin(), it + 1);  // 安全
  } else {
    return false;
  }
  refined_paths_.insert(refined_paths_.begin(), init_path_point);
  // recalculate sl wtr. current frame
  for (auto& path_point : refined_paths_) {
    SLPoint cur_frame_sl;
    const Point2D cart_point(path_point.x(), path_point.y());
    current_frenet_coord->XYToSL(cart_point.x, cart_point.y, &cur_frame_sl.s,
                                 &cur_frame_sl.l);
    path_point.set_s(cur_frame_sl.s);
    path_point.set_l(cur_frame_sl.l);
    path_point.set_theta(0.);
  }
  return true;
}

void DPRoadGraph::ClearDPInfo() {
  obstacles_info_.clear();
  static_obstacles_box_.clear();
  flatted_dynamic_obstacles_box_.clear();
  refined_paths_.clear();
  // sampled_points_.clear();
  dp_selected_points_.clear();
  min_cost_path_.clear();
}
std::shared_ptr<planning_math::KDPath> DPRoadGraph::ConstructLaneBorrowKDPath(
    const std::vector<double>& x_vec, const std::vector<double>& y_vec) {
  std::vector<planning_math::PathPoint> dp_path_points;
  dp_path_points.reserve(x_vec.size());
  for (int i = 0; i <= x_vec.size() - 1; ++i) {
    if (std::isnan(x_vec[i]) || std::isnan(y_vec[i])) {
      LOG_ERROR("skip NaN point");
      continue;
    }
    planning_math::PathPoint path_point{x_vec[i], y_vec[i]};
    dp_path_points.emplace_back(path_point);
    if (!dp_path_points.empty()) {
      auto& last_pt = dp_path_points.back();
      if (planning_math::Vec2d(last_pt.x() - path_point.x(),
                               last_pt.y() - path_point.y())
              .Length() < 1e-3) {
        continue;
      }
    }
  }
  if (dp_path_points.size() <= 2) {
    return nullptr;
  }
  return std::make_shared<planning_math::KDPath>(std::move(dp_path_points));
}

void DPRoadGraph::AddLaneBorrowVirtualObstacle(double obs_inner_l, double obs_start_s){
  const auto frenet_coord = current_reference_path_ptr_->get_frenet_coord();

/*
  if (min_cost_path_.size() <= 1) { // No Dp path add on center line
    return;
  }
  // along dp path to crossing lane boundary
  double dp_virtual_obs_s = 0;
  double dp_virtual_obs_x = 0;
  double dp_virtual_obs_y = 0;
  double dp_virtual_obs_theta = 0.;
  for(const auto& path_point: refined_paths_){
    if(path_point.s() > 120.0){
      return;
    }
    double path_x = path_point.x();
    double path_y = path_point.y();
    double path_s = path_point.s();
    double path_theta_cart = path_point.theta() + frenet_coord->GetPathCurveHeading(path_s);
    if(NudgeOutPose(path_x, path_y,path_theta_cart,borrow_dir)){
      dp_virtual_obs_x = path_x;
      dp_virtual_obs_y = path_y;
      dp_virtual_obs_theta = path_theta_cart;
      dp_virtual_obs_s = path_s;
      break;
    }
  }
  */
  // center line
  double distance_to_blocking = obs_start_s - ego_s_;

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double mini_gap = 3.0;
  double virtual_length = 1.0;// 1+6 = 7
  double center_virtual_s = obs_start_s - mini_gap + virtual_length* 0.5;
  double virtual_l = obs_inner_l;
  // if (center_virtual_s < 60.0 || center_virtual_s - virtual_length* 0.5 - vehicle_param.front_edge_to_rear_axle - 1.0 < ego_s_){
  //   return;
  // }
  Point2D end_sl_point(center_virtual_s,virtual_l);
  Point2D  cart_point;
  frenet_coord->SLToXY(end_sl_point, cart_point);
  // ReferencePathPoint refpath_pt;
  // current_reference_path_ptr_->get_reference_point_by_lon(center_virtual_s, refpath_pt);
  // double virtual_obs_x = refpath_pt.path_point.x();
  // double virtual_obs_y = refpath_pt.path_point.y();
  double virtual_obs_x = cart_point.x;
  double virtual_obs_y = cart_point.y;
  double virtual_obs_theta = frenet_coord->GetPathCurveHeading(obs_start_s);
  // build virtual obs
  planning::agent::Agent virtual_agent;
  virtual_agent.set_agent_id(999999);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_lane_borrow_virtual_obs(true);
  virtual_agent.set_x(virtual_obs_x);  //几何中心
  virtual_agent.set_y(virtual_obs_y);
  virtual_agent.set_length(virtual_length);
  virtual_agent.set_width(7.0);
  virtual_agent.set_fusion_source(1);
  virtual_agent.set_is_static(true);

  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(virtual_obs_theta);
  virtual_agent.set_accel(0.0);
  virtual_agent.set_time_range({0.0, 5.0});

  planning::planning_math::Box2d box(
      planning::planning_math::Vec2d(virtual_agent.x(), virtual_agent.y()),
      virtual_agent.theta(), virtual_agent.length(), virtual_agent.width());

  virtual_agent.set_box(box);
  virtual_agent.set_timestamp_s(0.0);
  virtual_agent.set_timestamp_us(0.0);
  auto *agent_manager = session_->environmental_model()
                            .get_dynamic_world()
                            ->mutable_agent_manager();
  std::unordered_map<int32_t, planning::agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);


  // reset path
  SetPullOverPath(obs_start_s,obs_inner_l);
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_x", virtual_agent.x())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_y", virtual_agent.y())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_theta",
                   virtual_agent.theta())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_id",
                   virtual_agent.agent_id())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_width",
                   virtual_agent.width())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_length",
                   virtual_agent.length())

  return ;

}
bool DPRoadGraph::NudgeOutPose(double path_ego_x, double path_ego_y, double path_ego_theta,BorrowDirection borrow_dir) {
  const auto& current_frenet_coord = current_lane_ptr_->get_lane_frenet_coord();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double heading_angle = path_ego_theta;
      // session_->environmental_model().get_ego_state_manager()->ego_pose().theta;
  double ego_x = path_ego_x;
  double ego_y = path_ego_y;

  // Get the corner points' Cartesian coordinates while traveling straight
  Point2D corner_front_left_point_xy(vehicle_param.front_edge_to_rear_axle,
                                     vehicle_param.width * 0.5);
  Point2D corner_front_right_point_xy(vehicle_param.front_edge_to_rear_axle,
                                      -vehicle_param.width * 0.5);
  Point2D corner_rear_left_point_xy(-vehicle_param.rear_edge_to_rear_axle,
                                    vehicle_param.width * 0.5);
  Point2D corner_rear_right_point_xy(-vehicle_param.rear_edge_to_rear_axle,
                                     -vehicle_param.width * 0.5);

  SLPoint corner_front_left, corner_rear_left, corner_front_right,
      corner_rear_right;

  if (borrow_dir == LEFT_BORROW) {
    Point2D corner_front_left_xy = CarRotattion(
        corner_front_left_point_xy, heading_angle, ego_x, ego_y);
    Point2D corner_rear_left_xy = CarRotattion(
        corner_rear_left_point_xy, heading_angle, ego_x, ego_y);

    // Back to the SL coordinate system and compare with the lane lines.
    current_frenet_coord->XYToSL(corner_front_left_xy.x, corner_front_left_xy.y,
                                 &corner_front_left.s, &corner_front_left.l);
    current_frenet_coord->XYToSL(corner_rear_left_xy.x, corner_rear_left_xy.y,
                                 &corner_rear_left.s, &corner_rear_left.l);

    const double current_front_left_lane_l =
        current_lane_ptr_->width_by_s(corner_front_left.s) * 0.5;
    const double current_rear_left_lane_l =
        current_lane_ptr_->width_by_s(corner_rear_left.s) * 0.5;
    if (corner_front_left.l > current_front_left_lane_l&&
        corner_rear_left.l > current_rear_left_lane_l) {
      return true;
    }else{
      return false;
    }
  } else {
    Point2D corner_front_right_xy = CarRotattion(
        corner_front_right_point_xy, heading_angle, ego_x, ego_y);
    Point2D corner_rear_right_xy = CarRotattion(
        corner_rear_right_point_xy, heading_angle, ego_x, ego_y);

    current_frenet_coord->XYToSL(corner_front_right_xy.x,
                                 corner_front_right_xy.y, &corner_front_right.s,
                                 &corner_front_right.l);
    current_frenet_coord->XYToSL(corner_rear_right_xy.x, corner_rear_right_xy.y,
                                 &corner_rear_right.s, &corner_rear_right.l);

    const double current_front_right_lane_l =
        current_lane_ptr_->width_by_s(corner_front_right.s) * 0.5;
    const double current_rear_right_lane_l =
        current_lane_ptr_->width_by_s(corner_rear_right.s) * 0.5;

    if (corner_front_right.l < -current_front_right_lane_l&&
        corner_rear_right.l < -current_rear_right_lane_l) {
      return true;
    }else{
      return false;
    }
  }
}
Point2D DPRoadGraph::CarRotattion(const Point2D& Cartesian_point,
                                             double heading_angle, double ego_x,
                                             double ego_y) {
  double cos_theta = cos(heading_angle);
  double sin_theta = sin(heading_angle);
  return {
      Cartesian_point.x * cos_theta - Cartesian_point.y * sin_theta + ego_x,
      Cartesian_point.x * sin_theta + Cartesian_point.y * cos_theta + ego_y};
};
void DPRoadGraph::SetPullOverPath(double end_s, double end_l){ // re-set refined_paths_
  if(end_s - init_sl_point_.s < 0){
    return;
  }
  const auto& current_frenet_coord = current_lane_ptr_->get_lane_frenet_coord();
  refined_paths_.clear();
  planning_math::QuinticPolynomialCurve1d curve(
    init_sl_point_.l, 0, 0, end_l, 0.0, 0.0,
    end_s - init_sl_point_.s);
  double path_s_length = end_s - init_sl_point_.s;
  double path_resolution = std::min(0.5, (path_s_length/4));
  double current_s = 0.0;

  while (current_s + path_resolution * 0.5 < path_s_length) {
      // current_s += path_resolution;
      const double l = curve.Evaluate(0, current_s);
      const double dl = curve.Evaluate(1, current_s);
      const double ddl = curve.Evaluate(2, current_s);
      const double dddl = curve.Evaluate(3, current_s);
      const double theta = std::atan2(dl, 1);  // 计算heading角度 不是全局的
      const double kappa = curve.EvaluateKappa(current_s);  // 获取曲率
      const double denominator = std::pow(1 + dl * dl, 1.5);
      const double numerator1 = dddl;
      const double numerator2 = 3 * ddl * dl * ddl / std::pow(1 + dl * dl, 2.5);
      const double dkappa = numerator1 / denominator - numerator2;
      const double ddkappa = 0.;

      const SLPoint sl_point(init_sl_point_.s + current_s, l);

      Point2D sl_cart(0, 0);  // nonsene
      // donnot delete, discard for pybind
      current_frenet_coord->SLToXY(init_sl_point_.s + current_s, l, &sl_cart.x,
                                   &sl_cart.y);
      planning_math::PathPoint path_point{sl_cart.x,  sl_cart.y, sl_point.s,
                                          sl_point.l, theta,     kappa,
                                          dkappa,     ddkappa};
      refined_paths_.emplace_back(path_point);
      current_s += path_resolution;
    }
}


void DPRoadGraph::LogDebugInfo() {
  auto dp_road_pb_info =
      DebugInfoManager::GetInstance().GetDebugInfoPb()->mutable_dp_road_info();
  dp_road_pb_info->Clear();
  // env
  dp_road_pb_info->mutable_sample_lanes_info()->set_left_boundary(
      sample_left_boundary_);
  dp_road_pb_info->mutable_sample_lanes_info()->set_right_boundary(
      sample_right_boundary_);
  dp_road_pb_info->set_s_range(s_range_);
  dp_road_pb_info->set_l_range(l_range_);
  dp_road_pb_info->set_total_length(total_length_);
  dp_road_pb_info->set_v_cruise(v_cruise_);
  const auto& current_frenet_coord =
      current_reference_path_ptr_->get_frenet_coord();
  for (const auto& level_points : sampled_points_) {
    auto* new_level_points = dp_road_pb_info->mutable_sample_lanes_info()
                                 ->add_sampled_points();  // 添加 LevelPoints
    for (const auto& point : level_points) {
      auto* sample_sl_point =
          new_level_points->add_level_points();  // 添加 SampleSLPoint
      sample_sl_point->set_s(point.s);
      sample_sl_point->set_l(point.l);
      // 转换 SL 坐标到 XY 坐标
      Point2D sl_cart;
      current_frenet_coord->SLToXY(point.s, point.l, &sl_cart.x, &sl_cart.y);
      // 添加转换后的坐标
      dp_road_pb_info->mutable_sample_lanes_info()->add_sampled_xs(sl_cart.x);
      dp_road_pb_info->mutable_sample_lanes_info()->add_sampled_ys(sl_cart.y);
    }
  }

  for (const auto& static_ob : obstacles_info_) {  // static
    auto* static_obs_info = dp_road_pb_info->mutable_obstacles_info()->Add();
    static_obs_info->set_id(static_ob.id);
    static_obs_info->set_s_start(static_ob.s_start);
    static_obs_info->set_s_end(static_ob.s_end);
    static_obs_info->set_l_start(static_ob.l_start);
    static_obs_info->set_l_end(static_ob.l_end);
  }
  dp_road_pb_info->mutable_print_info()->set_ego_s(ego_s_);
  dp_road_pb_info->mutable_print_info()->set_ego_l(ego_l_);
  dp_road_pb_info->mutable_print_info()->set_ego_v(ego_v_);
  double safe_cost = 0.0;
  double smooth_cost = 0.0;
  double stitch_cost = 0.0;
  for (const auto& node : min_cost_path_) {
    safe_cost += node.min_cost_.safety_cost_;
    smooth_cost += node.min_cost_.smooth_cost_;
    stitch_cost += node.min_cost_.stitch_cost_;
  }
  dp_road_pb_info->mutable_print_info()->set_safe_cost(safe_cost);
  dp_road_pb_info->mutable_print_info()->set_smooth_cost(smooth_cost);
  dp_road_pb_info->mutable_print_info()->set_stitch_cost(stitch_cost);

  dp_road_pb_info->mutable_print_info()->set_ego_l(ego_l_);
  dp_road_pb_info->mutable_print_info()->set_ego_v(ego_v_);
  // sample result
  for (const auto& selected_point : dp_selected_points_) {
    auto* selected_points_pb = dp_road_pb_info->mutable_dp_result_path()
                                   ->mutable_selected_points()
                                   ->Add();
    selected_points_pb->set_s(selected_point.s);
    selected_points_pb->set_l(selected_point.l);
    Point2D sl_cart;
    current_frenet_coord->SLToXY(selected_point.s, selected_point.l, &sl_cart.x,
                                 &sl_cart.y);
    dp_road_pb_info->mutable_dp_result_path()->mutable_selected_xs()->Add(
        sl_cart.x);
    dp_road_pb_info->mutable_dp_result_path()->mutable_selected_ys()->Add(
        sl_cart.y);
  }
  // fined path
  for (const auto& path_point : refined_paths_) {
    auto* fined_points_pb = dp_road_pb_info->mutable_dp_result_path()
                                ->mutable_fined_points()
                                ->Add();
    dp_road_pb_info->mutable_dp_result_path()->mutable_fined_xs()->Add(
        path_point.x());
    dp_road_pb_info->mutable_dp_result_path()->mutable_fined_ys()->Add(
        path_point.y());
    fined_points_pb->set_s(path_point.s());
    fined_points_pb->set_l(path_point.l());
  }
  // cost params
  dp_road_pb_info->mutable_dp_param()->set_coeff_l_cost(coeff_l_cost_);
  dp_road_pb_info->mutable_dp_param()->set_coeff_dl_cost(coeff_dl_cost_);
  dp_road_pb_info->mutable_dp_param()->set_coeff_ddl_cost(coeff_ddl_cost_);
  dp_road_pb_info->mutable_dp_param()->set_coeff_end_l_cost(coeff_end_l_cost_);
  dp_road_pb_info->mutable_dp_param()->set_path_resolution(path_resolution_);
  dp_road_pb_info->mutable_dp_param()->set_coeff_collision_cost(
      coeff_collision_cost_);
  dp_road_pb_info->mutable_dp_param()->set_collision_distance(
      collision_distance_);
  dp_road_pb_info->mutable_dp_param()->set_coeff_stitch_cost(
      coeff_stitch_cost_);
  // cost end_time
  dp_road_pb_info->mutable_print_info()->set_dp_cost_time(dp_cost_time_);
  dp_road_pb_info->mutable_print_info()->set_safety_cost_time(
      safety_cost_time_);
  dp_road_pb_info->mutable_print_info()->set_stitch_cost_time(
      stitch_cost_time_);
  dp_road_pb_info->mutable_print_info()->set_path_cost_time(path_cost_time_);
  dp_road_pb_info->mutable_print_info()->set_iter_num(iter_num_);

  dp_road_pb_info->mutable_print_info()->set_dp_search_cost_time(
      dp_search_cost_time_);
  dp_road_pb_info->mutable_print_info()->set_concerned_obs_num(
      flatted_dynamic_obstacles_box_.size() + static_obstacles_box_.size());

}  // namespace planning
}  // namespace planning