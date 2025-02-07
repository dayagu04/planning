#include "hybrid_ara_star.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>

#include "debug_info_log.h"
#include "edt_manager.h"
#include "environmental_model.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "task_basic_types.h"
#include "vehicle_config_context.h"

namespace planning {

namespace {
constexpr double kDistanceCrossLine = 50.0;

#ifdef X86
constexpr double kSkipAppendSearchTimeLimit = 50;  // ms
constexpr double kTotalSearchTimeLimit = 50;       // ms
#else
constexpr double kSkipAppendSearchTimeLimit = 10;  // ms
constexpr double kTotalSearchTimeLimit = 8;        // ms
#endif
// zkxie(TODO): 根据场景而定
constexpr double kLookAheadTime = 5.0;  // second
// zkxie(TODO): 根据场景而定
constexpr double kExtendARASearchRange = 20.0;
constexpr double kDefaultLaneWidth = 3.5;
// zkxie(TODO): 1 or 2
constexpr double kMinHeuristicFactor = 2.0;
constexpr int kCollisionStepSize = 3;
constexpr double kBendRoadRadius = 30;
}  // namespace

HybridARAStar::HybridARAStar(framework::Session* session) {
  session_ = session;
  auto config_builder = session_->environmental_model().hpp_config_builder();
  hybrid_ara_star_conf_ = config_builder->cast<HybridAraStarConfig>();
  hpp_general_lateral_decider_config_ =
      config_builder->cast<HppGeneralLateralDeciderConfig>();
  vehicle_param_ = VehicleConfigurationContext::Instance()->get_vehicle_param();
  next_node_num_ = hybrid_ara_star_conf_.next_node_num;
  max_front_wheel_angle_ =
      vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio *
      hybrid_ara_star_conf_.use_percentage_of_steering;  // need to be rad
  step_size_ = hybrid_ara_star_conf_.step_size;
  one_shot_distance_ = hybrid_ara_star_conf_.one_shot_distance;
  small_shot_distance_ = hybrid_ara_star_conf_.small_shot_distance;
  x_grid_resolution_ = hybrid_ara_star_conf_.x_grid_resolution;
  y_grid_resolution_ = hybrid_ara_star_conf_.y_grid_resolution;
  phi_grid_resolution_ = hybrid_ara_star_conf_.phi_grid_resolution;
  heuristic_factor_ = hybrid_ara_star_conf_.heuristic_factor;
  ego_half_width_ = vehicle_param_.max_width / 2.0;
  ego_half_length_ = vehicle_param_.length / 2.0;
  center_cost_weight_ = hybrid_ara_star_conf_.center_cost_weight;
  agent_cost_weight_ = hybrid_ara_star_conf_.agent_cost_weight;
  boundary_cost_weight_ = hybrid_ara_star_conf_.boundary_cost_weight;
  motion_cost_weight_ = hybrid_ara_star_conf_.motion_cost_weight;
  boundary_soft_extra_buffer_ =
      hybrid_ara_star_conf_.boundary_soft_extra_buffer;
  crosslinebuffer_ = hybrid_ara_star_conf_.crosslinebuffer;
  enable_middle_final_node_ = hybrid_ara_star_conf_.enable_middle_final_node;
  l_limit_ = hybrid_ara_star_conf_.l_limit;
  collision_buffer_ = hybrid_ara_star_conf_.collision_buffer;
  rear_obs_s_ = hybrid_ara_star_conf_.rear_obs_s;
  front_obs_s_ = hybrid_ara_star_conf_.front_obs_s;
  longitudinal_extend_ = hybrid_ara_star_conf_.longitudinal_extend;
  lateral_extend_ = hybrid_ara_star_conf_.lateral_extend;
  hpp_min_search_range_ = hybrid_ara_star_conf_.hpp_min_search_range;

  std::cout << "HybridARAStar::HybridARAStar() ===========" << std::endl;
  std::cout << "x_grid_resolution_: " << x_grid_resolution_ << std::endl;
  std::cout << "y_grid_resolution_: " << y_grid_resolution_ << std::endl;
  std::cout << "phi_grid_resolution_: " << phi_grid_resolution_ << std::endl;
  std::cout << "next_node_num_: " << next_node_num_ << std::endl;
  std::cout << "step_size_: " << step_size_ << std::endl;
  std::cout << "max_front_wheel_angle_: " << max_front_wheel_angle_
            << std::endl;
  std::cout << "agent cost weight: " << agent_cost_weight_ << std::endl;
  std::cout << "center cost weight: " << center_cost_weight_ << std::endl;
  std::cout << "motion cost weight: " << motion_cost_weight_ << std::endl;
  std::cout << "boundary cost weight: " << boundary_cost_weight_ << std::endl;
  std::cout << "collision_buffer: " << collision_buffer_ << std::endl;
  std::cout << "l_limit: " << l_limit_ << std::endl;
  std::cout << "enable_middle_final_node: " << enable_middle_final_node_
            << std::endl;
}

// void HybridARAStar::BuildRBLineSeg(
//     const std::vector<const cp_common::math::LineSegment2d*>&
//     lidar_rb_vector, const std::vector<cp_common::math::LineSegment2d>&
//     camera_rb_vector) {
//   // TODO:update obstacles_linesegments_vec_
//   // TODO: this may need to be changed, now all rb in one vector
//   Lidar_RB_linesegment_vec_.clear();
//   Camera_RB_linesegment_vec_.clear();
//   for (const auto lidar_rb_ptr : lidar_rb_vector) {
//     Lidar_RB_linesegment_vec_.emplace_back(*lidar_rb_ptr);
//   }
//   for (const auto camera_rb : camera_rb_vector) {
//     Camera_RB_linesegment_vec_.emplace_back(camera_rb);
//   }
//   obstacles_linesegments_vec_.emplace_back(Lidar_RB_linesegment_vec_);
//   obstacles_linesegments_vec_.emplace_back(Camera_RB_linesegment_vec_);
// }

void HybridARAStar::UpdateOpenSetWithHeuristicFactor() {
  open_pq_ = decltype(open_pq_)();
  for (auto& node : open_set_) {
    open_pq_.emplace(node.first, node.second->GetCost(heuristic_factor_));
  }
}

void HybridARAStar::UpdateHeuristicFactor() {
  std::string current_id = open_pq_.top().first;
  // 如果该node不在open_set中，则舍弃
  while (open_set_.find(current_id) == open_set_.end()) {
    open_pq_.pop();
    if (open_pq_.empty()) {
      std::cout << "open_pq_ is empty" << std::endl;
      return;
    }
    current_id = open_pq_.top().first;
  }
  std::shared_ptr<Node3D> current_node = open_set_[current_id];
  double best_fvalue = current_node->GetCost();

  // 更新epsilon的逻辑，依据？
  double ref_heuristic_factor =
      std::max(heuristic_factor_ / 3.0, kMinHeuristicFactor);
  heuristic_factor_ = std::min(ref_heuristic_factor,
                               (final_node_->GetTrajCost() / best_fvalue));
  heuristic_factor_ =
      std::max(std::floor(heuristic_factor_), kMinHeuristicFactor);
}

bool HybridARAStar::GetResult(ara_star::HybridARAStarResult& result) const {
  if (final_node_ == nullptr) {
    return false;
  }
  std::cout << "final node: " << final_node_->GetS() << " "
            << final_node_->GetL() << std::endl;

  std::shared_ptr<Node3D> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  std::vector<double> hybrid_a_s;
  std::vector<double> hybrid_a_l;
  if (current_node->GetPreNode() == nullptr) {
    return false;
  }
  auto& planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto hybrid_ara_path_cost = planning_debug_data->mutable_hybrid_ara_info()
                                  ->mutable_hybrid_ara_path_cost();
  hybrid_ara_path_cost->Clear();

  while (current_node->GetPreNode() != nullptr) {
    hybrid_ara_path_cost->add_x(current_node->GetX());
    hybrid_ara_path_cost->add_y(current_node->GetY());
    hybrid_ara_path_cost->add_phi(current_node->GetPhi());
    hybrid_ara_path_cost->add_agent_cost(current_node->GetAgentCost());
    hybrid_ara_path_cost->add_boundary_cost(current_node->GetBoundaryCost());
    hybrid_ara_path_cost->add_center_cost(current_node->GetCenterCost());
    hybrid_ara_path_cost->add_motion_cost(current_node->GetMotionCost());

    /*
    // here skip the final node which we don't control due to the stop
    condition
    // it may leave the center line
    // current_node = current_node->GetPreNode();
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();

    if (x.empty() || y.empty() || phi.empty()) {
      // std::cout << "result size check failed" << std::endl;
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      // std::cout << "states sizes are not equal" << std::endl;
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    */

    hybrid_a_x.emplace_back(current_node->GetX());
    hybrid_a_y.emplace_back(current_node->GetY());
    hybrid_a_phi.emplace_back(current_node->GetPhi());
    hybrid_a_s.emplace_back(current_node->GetS());
    hybrid_a_l.emplace_back(current_node->GetL());

    current_node = current_node->GetPreNode();
  }

  hybrid_a_x.emplace_back(current_node->GetX());
  hybrid_a_y.emplace_back(current_node->GetY());
  hybrid_a_phi.emplace_back(current_node->GetPhi());
  hybrid_a_s.emplace_back(current_node->GetS());
  hybrid_a_l.emplace_back(current_node->GetL());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  std::reverse(hybrid_a_s.begin(), hybrid_a_s.end());
  std::reverse(hybrid_a_l.begin(), hybrid_a_l.end());
  result.x = std::move(hybrid_a_x);
  result.y = std::move(hybrid_a_y);
  result.phi = std::move(hybrid_a_phi);
  result.s = std::move(hybrid_a_s);
  result.l = std::move(hybrid_a_l);

  return true;
}

void HybridARAStar::SetMiddleFinalNode() {
  // find max s from close_set_ and set it as final node
  if (close_set_.empty()) {
    return;
  }
  auto min_cost_node_iter = close_set_.begin();
  double min_cost = std::numeric_limits<double>::max();

  for (auto it = close_set_.begin(); it != close_set_.end(); ++it) {
    double current_cost = it->second->GetCost(1.0);
    if (current_cost < min_cost) {
      min_cost = current_cost;
      min_cost_node_iter = it;
    }
  }
  final_node_ = min_cost_node_iter->second;
}

// s是否达到终点的s
bool HybridARAStar::ReachDestination(const std::shared_ptr<Node3D> node) const {
  // TODO:: redefine find the goal
  if (node->GetS() >= end_s_) {  //&& std::abs(node->GetL() - end_l_) < 1.0) {
    return true;
  }
  return false;
}

std::shared_ptr<Node3D> HybridARAStar::NextNodeGenerator(
    const std::shared_ptr<Node3D> current_node, const size_t next_node_index) {
  double steering = 0.0;
  // forward motion
  // next_node_index from 0;
  double middle_index = (next_node_num_ - 1) / 2.0;
  double scale_factor = 1.0;
  if (max_front_wheel_angle_ < 0.25) {
    scale_factor = 1.0;
  } else {
    scale_factor = 1.2;
  }
  // use non Uniform Sampling
  if (next_node_index < next_node_num_) {
    double scale =
        std::pow(std::abs((next_node_index - middle_index)) / middle_index,
                 scale_factor);
    if (next_node_index < middle_index) {
      steering = -max_front_wheel_angle_ * scale;
    } else {
      steering = max_front_wheel_angle_ * scale;
    }
  } else {
    std::cout << "next node index error" << std::endl;
    return nullptr;
  }

  // motion integration

  // 最小是0.5，自车以当前速度0.2s走的距离，速度越大，这个值越大，魔数
  // 有障碍物时，步长小一点，没有障碍物时，步长大一点
  // HPP中速度变化较小，取消和速度的关系
  double one_shot_distance_update = one_shot_distance_;
  if (current_node->GetS() <
          obs_max_s_ + vehicle_param_.rear_edge_to_rear_axle &&
      current_node->GetS() >
          obs_min_s_ - vehicle_param_.front_edge_to_rear_axle) {
    one_shot_distance_update = small_shot_distance_;
  }
  if (in_bend_) {
    one_shot_distance_update = small_shot_distance_;
  }

  // one_shot_distance_的取值要注意，apollo中该值是xy_grid_resolution_的根号2倍，这样保证了一个格子里，最多只有一个节点
  int num_steps = static_cast<int>(one_shot_distance_update / step_size_);

  // std::cout << "one_shot_distance_update: " << one_shot_distance_update
  //           << std::endl;
  // std::cout << "num_steps: " << num_steps << std::endl;

  std::vector<double> intermediate_x;
  intermediate_x.reserve(num_steps + 2);
  std::vector<double> intermediate_y;
  intermediate_y.reserve(num_steps + 2);
  std::vector<double> intermediate_phi;
  intermediate_phi.reserve(num_steps + 2);

  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.emplace_back(last_x);
  intermediate_y.emplace_back(last_y);
  intermediate_phi.emplace_back(last_phi);

  // 保持前后帧扩展节点的s尽量一致
  if (expand_num_ <= 2) {
    for (auto s : last_result_s_) {
      double dist = s - compensation_s_ - current_node->GetS();
      if (dist > 0.2 && dist < 1.5) {
        std::array<double, 5> num_steps_x{0.2, 0.4, 0.6, 0.8, 1.0};
        std::array<double, 5> num_steps_y{1, 2, 3, 4, 5};
        num_steps = interp(dist, num_steps_x, num_steps_y);
        step_size_ = dist / num_steps;
        break;
      }
    }
  } else {
    step_size_ = hybrid_ara_star_conf_.step_size;
  }

  // step_size_永远是正，只向前，不后退
  for (size_t i = 0; i < num_steps; ++i) {
    const double next_x = last_x + step_size_ * std::cos(last_phi);
    const double next_y = last_y + step_size_ * std::sin(last_phi);
    const double next_phi = planning_math::NormalizeAngle(
        last_phi + step_size_ / vehicle_param_.wheel_base * std::tan(steering));
    intermediate_x.emplace_back(next_x);
    intermediate_y.emplace_back(next_y);
    intermediate_phi.emplace_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }

  // check if the vehicle has no benifit to move forward
  double current_s = current_node->GetS();
  double next_s = 0.0;
  double next_l = 0.0;

  if (!fix_lane_->XYToSL(intermediate_x.back(), intermediate_y.back(), &next_s,
                         &next_l)) {
    std::cout << "next node out of lane" << std::endl;
    return nullptr;
  }

  if (next_s <= current_s) {
    std::cout << "do not drive back in s derection" << std::endl;
    return nullptr;
  }

  // check if the vehicle runs outside ego lane
  ReferencePathPoint refpath_pt{};
  if (!reference_path_ptr_->get_reference_point_by_lon(next_s, refpath_pt)) {
    std::cout << "Hybrid ara*: Get reference point by lon failed!" << std::endl;
  }
  if (next_l > refpath_pt.distance_to_left_lane_border - ego_half_width_ +
                   crosslinebuffer_ ||
      next_l < -(refpath_pt.distance_to_right_lane_border - ego_half_width_ +
                 crosslinebuffer_)) {
    return nullptr;
  }

  constexpr double kIgnoreLThreshold = 0.0;
  if (no_left_ && next_l > kIgnoreLThreshold) {
    std::cout << "Ignore left point" << std::endl;
    return nullptr;
  } else if (no_right_ && next_l < -kIgnoreLThreshold) {
    std::cout << "Ignore right point" << std::endl;
    return nullptr;
  }

  auto next_node = std::make_shared<Node3D>(
      intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
      x_grid_resolution_, y_grid_resolution_, phi_grid_resolution_);

  next_node->SetSteer(steering);
  next_node->SetS(next_s);
  next_node->SetL(next_l);
  next_node->SetDeltaS(next_s - ego_s_);

  return next_node;
}

void HybridARAStar::RegisterCost(ara_star::CostManager& cost_manager) const {
  // 自车横向偏距的cost
  auto center_cost_ptr = std::make_shared<ara_star::CenterCost>(
      center_cost_weight_, vehicle_param_.wheel_base, fix_lane_);
  cost_manager.AddCost(center_cost_ptr);

  // 与agent的距离越近，cost越大
  auto agent_cost_ptr = std::make_shared<ara_star::AgentCost>(
      agent_cost_weight_, vehicle_param_.wheel_base, vehicle_param_.length,
      ego_half_width_, bounding_box_vec_, fix_lane_, agent_box_tree_,
      reference_path_ptr_, nudge_agents_,
      vehicle_param_.front_edge_to_rear_axle,
      vehicle_param_.rear_edge_to_rear_axle, pass_interval_, left_turn_,
      right_turn_);
  cost_manager.AddCost(agent_cost_ptr);

  // 与道路边缘越近，cost越大
  // auto boundary_cost_ptr = std::make_shared<ara_star::BoundaryCost>(
  //     boundary_cost_weight_, init_v_, vehicle_param_.wheel_base,
  //     ego_half_width_, lane_width_, fix_lane_,
  //     hard_safe_distance_, soft_safe_distance_);
  // cost_manager.AddCost(boundary_cost_ptr);
}

bool HybridARAStar::ImprovePath() {
  // auto start_time = (uint64_t)IflyTime::Now_us();

  bool has_found_new_path = false;
  bool search_find_result = true;
  std::unordered_map<std::string, std::shared_ptr<Node3D>> incons_set;

  // register cost manager
  ara_star::CostManager cost_manager;
  RegisterCost(cost_manager);
  // motion cost rely on last steer angle, this is ugly. Reactor needed.
  // TODO: or we may just delete this cost
  auto motion_cost_ptr = std::make_shared<ara_star::MotionCost>(
      motion_cost_weight_, max_front_wheel_angle_, 0.0);
  cost_manager.AddCost(motion_cost_ptr);

  // auto end_time = (uint64_t)IflyTime::Now_us();
  // std::cout << "ImprovePath: prepare time " << (end_time - start_time)
  //           << std::endl;

  auto& planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto hybrid_ara_expand = planning_debug_data->mutable_hybrid_ara_info()
                               ->mutable_hybrid_ara_expand();

  while (true) {
    auto current_time = (uint64_t)IflyTime::Now_ms();
    auto diff = current_time - start_search_time_;

    if (diff > kTotalSearchTimeLimit) {
      std::cout << "ARA search exceed time limit: " << kTotalSearchTimeLimit
                << "ms" << std::endl;
      if (!has_found_new_path) {
        search_find_result = false;
      }
      break;
    }

    // auto time1 = (uint64_t)IflyTime::Now_us();

    // double goal_fvalue = final_node_->GetCost();
    double goal_fvalue = final_node_->GetCost(heuristic_factor_);
    double best_fvalue = open_pq_.top().second;

    // 找到终点后的退出机制
    if (goal_fvalue <= best_fvalue) {
      std::cout << "ImprovePath success!!! goal_fvalue: " << goal_fvalue
                << " best_fvalue: " << best_fvalue << std::endl;
      break;
    }

    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();

    if (open_set_.find(current_id) == open_set_.end()) {
      // since we may add same index twice in open pq, so we need do this
      // protection.
      continue;
    }

    std::shared_ptr<Node3D> current_node = open_set_[current_id];
    open_set_.erase(current_id);

    // put into close set
    close_set_.emplace(current_node->GetIndex(), current_node);

    // 判断此时的s是否大于终点的s
    if (current_node->ReachDest()) {
      continue;
    }

    // auto time2 = (uint64_t)IflyTime::Now_us();
    // std::cout << "pq pop time: " << (time2 - time1) << std::endl;

    expand_num_++;
    for (size_t i = 0; i < next_node_num_; ++i) {
      // auto time3 = (uint64_t)IflyTime::Now_us();

      std::shared_ptr<Node3D> next_node = NextNodeGenerator(current_node, i);
      num_node_expand_++;

      // auto time4 = (uint64_t)IflyTime::Now_us();
      // std::cout << "next node generate time: " << (time4 - time3) <<
      // std::endl;

      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }

      // auto time5 = (uint64_t)IflyTime::Now_us();
      // std::cout << "ValidityCheck time: " << (time5 - time4) << std::endl;

      // check if the node is already in the observed set
      // observed_set_ 不清空
      auto iter = observed_set_.find(next_node->GetIndex());
      if (iter != observed_set_.end()) {
        next_node->SetTrajCost(iter->second->GetTrajCost());
        next_node->SetHeuCost(iter->second->GetHeuCost());
      } else {
        next_node->SetTrajCost(std::numeric_limits<double>::max());
        next_node->SetHeuCost(0.0);
        observed_set_.emplace(next_node->GetIndex(), next_node);
      }

      // calculate g cost
      motion_cost_ptr->SetLastSteeringAngle(current_node->GetSteer());
      double cost = cost_manager.ComputeCost(*next_node);
      double new_traj_cost = current_node->GetTrajCost() + cost;

      // auto time6 = (uint64_t)IflyTime::Now_us();
      // std::cout << "ComputeCost time: " << (time6 - time5) << std::endl;

      if (next_node->GetTrajCost() > new_traj_cost) {
        // use new node connection
        next_node->SetTrajCost(new_traj_cost);
        next_node->SetHeuCost(CalculateBaseHeuCost(next_node));
        next_node->SetPre(current_node);
        // update observed_set_
        if (iter != observed_set_.end()) {
          iter->second->SetTrajCost(next_node->GetTrajCost());
          iter->second->SetHeuCost(next_node->GetHeuCost());
          iter->second->SetPre(current_node);
        }

        if (close_set_.find(next_node->GetIndex()) == close_set_.end()) {
          if (ReachDestination(next_node)) {
            std::cout << "reach destination, traj cost " << new_traj_cost
                      << std::endl;
            next_node->SetHeuCost(0.0);
            next_node->SetReachDest(true);
            if (final_node_->GetTrajCost() > next_node->GetTrajCost()) {
              final_node_ = next_node;
            }
            has_found_new_path = true;
          }

          // this will update map value if key exist, otherwise add new key;
          open_set_[next_node->GetIndex()] = next_node;
          open_pq_.emplace(next_node->GetIndex(),
                           next_node->GetCost(heuristic_factor_));
        } else {
          // 如果这个节点被加入过close set，则把他加入 incons set
          incons_set.emplace(next_node->GetIndex(), next_node);
        }
      }
      // auto time7 = (uint64_t)IflyTime::Now_us();
      // std::cout << "incons cost time: " << (time7 - time6) << std::endl;

    }  // end node explore loop

    if (open_set_.empty() && !has_found_new_path) {
      std::cout << "ImprovePath: open_set_ is empty and find no path!!!"
                << std::endl;
      search_find_result = false;
      break;
    }

    LogNodeDebugInfo(current_node, hybrid_ara_expand);

  }  // end while loop

  if (!search_find_result) {
    std::cout << "ImprovePath: search find no path" << std::endl;
    return false;
  }

  if (!incons_set.empty()) {
    // auto time8 = (uint64_t)IflyTime::Now_us();
    // merge incons_set_ into open_set_
    for (auto& node : incons_set) {
      open_set_.emplace(node.first, node.second);
    }
    // auto time9 = (uint64_t)IflyTime::Now_us();
    // std::cout << "merge incons_set_ into open_set_ time: " << (time9 - time8)
    //           << std::endl;

    // rebuild open_pq_
    open_pq_ = decltype(open_pq_)();
    for (auto& node : open_set_) {
      // 这里的epsilon无所谓，因为后面会更新epsilon然后更新open list
      open_pq_.emplace(node.first, node.second->GetCost());
    }
    // auto time10 = (uint64_t)IflyTime::Now_us();
    // std::cout << "rebuild open_pq_ time: " << (time10 - time9) << std::endl;
  }

  return true;
}

void HybridARAStar::LogNodeDebugInfo(
    const std::shared_ptr<Node3D>& current_node,
    common::HybridARAExpand* hybrid_ara_expand) {
  // put data into debug info
  auto expand_node = hybrid_ara_expand->add_hybrid_ara_expand();
  auto expand_current_node = expand_node->mutable_current_node();
  expand_current_node->set_x(current_node->GetX());
  expand_current_node->set_y(current_node->GetY());
  expand_current_node->set_phi(current_node->GetPhi());
  expand_current_node->set_agent_cost(current_node->GetAgentCost());
  expand_current_node->set_boundary_cost(current_node->GetBoundaryCost());
  expand_current_node->set_center_cost(current_node->GetCenterCost());
  expand_current_node->set_motion_cost(current_node->GetMotionCost());
  expand_current_node->set_heuristic_cost(current_node->GetHeuCost());
  expand_current_node->set_total_cost(current_node->GetCost(heuristic_factor_));
  expand_current_node->set_min_dist(current_node->GetMinDist());
  expand_current_node->set_dist_cost(current_node->GetDistCost());
  expand_current_node->set_area_cost(current_node->GetAreaCost());
  expand_current_node->set_directly_behind_cost(
      current_node->GetDirectlyBehindCost());
  expand_current_node->set_pass_interval_cost(
      current_node->GetPassIntervalCost());

  for (auto& node : open_set_) {
    auto open_list = expand_node->add_open_list();
    open_list->set_x(node.second->GetX());
    open_list->set_y(node.second->GetY());
    open_list->set_phi(node.second->GetPhi());
    open_list->set_agent_cost(node.second->GetAgentCost());
    open_list->set_boundary_cost(node.second->GetBoundaryCost());
    open_list->set_center_cost(node.second->GetCenterCost());
    open_list->set_motion_cost(node.second->GetMotionCost());
    open_list->set_heuristic_cost(node.second->GetHeuCost() *
                                  heuristic_factor_);
    open_list->set_total_cost(node.second->GetCost(heuristic_factor_));
    open_list->set_min_dist(node.second->GetMinDist());
    open_list->set_dist_cost(node.second->GetDistCost());
    open_list->set_area_cost(node.second->GetAreaCost());
    open_list->set_directly_behind_cost(node.second->GetDirectlyBehindCost());
    open_list->set_pass_interval_cost(node.second->GetPassIntervalCost());
  }
}

// 与终点s的差值，最小为0
double HybridARAStar::CalculateBaseHeuCost(
    const std::shared_ptr<Node3D> current_node) const {
  // without heuristic factor
  // 为了避免临近终点时，h太小造成的选节点错误
  constexpr uint8_t kEndSBuffer = 5;
  double cost = end_s_ - current_node->GetS() + kEndSBuffer;
  double longitudinal_cost = cost > 0.0 ? cost : 0.0;
  return longitudinal_cost;
}

planning_math::Box2d HybridARAStar::GetBoundingBox(const double x,
                                                   const double y,
                                                   const double phi) const {
  double ego_length = vehicle_param_.length;
  double ego_width = vehicle_param_.max_width;
  double shift_distance = vehicle_param_.rear_axle_to_center;
  planning_math::Box2d ego_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, ego_length, ego_width);
  return ego_box;
}

bool HybridARAStar::ValidityCheck(const std::shared_ptr<Node3D> node) {
  // prepare for edt
  const AstarPathGear gear = AstarPathGear::DRIVE;
  float dist = 100.0;
  float min_dist = 100.0;

  // implement collision check here with small step size
  size_t node_step_num = node->GetStepNum();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  const int check_start_index = (node_step_num == 1) ? 0 : 1;

  // since step size is 0.2m for motion integration, we check every 1.0m for
  // collision is enough
  // 为了加快速度，省略了一些中间的点，这样理论上会有碰撞的风险，但是概率很小
  for (int i = node_step_num - 1; i >= check_start_index;
       i -= kCollisionStepSize) {
    const double cos_phi = std::cos(traversed_phi[i]);
    const double sin_phi = std::sin(traversed_phi[i]);
    const double front_x =
        traversed_x[i] + vehicle_param_.front_edge_to_rear_axle * cos_phi;
    const double front_y =
        traversed_y[i] + vehicle_param_.front_edge_to_rear_axle * sin_phi;

    planning_math::Vec2d back_axis_position(traversed_x[i], traversed_y[i]);
    planning_math::Vec2d ego_head_position(front_x, front_y);
    planning_math::Vec2d center_position((traversed_x[i] + front_x) / 2,
                                         (traversed_y[i] + front_y) / 2);

    Pose2D local_point;
    ego_base_.GlobalPointToULFLocal(&local_point,
                                    Pose2D(traversed_x[i], traversed_y[i], 0));
    double relative_theta = planning_math::NormalizeAngle(
        traversed_phi[i] - ego_base_.GetConstBasePose().GetPhi());
    Transform2d tf;
    tf.SetBasePose(Pose2D(local_point.x, local_point.y, relative_theta));
    if (edt_->DistanceCheckForPoint(&dist, &tf, gear)) {
      return false;
    }
    if (dist < min_dist) {
      min_dist = dist;
    }

    // max l check
    double ego_front_s = 0.0;
    double ego_front_l = 0.0;
    if (!fix_lane_->XYToSL(ego_head_position.x(), ego_head_position.y(),
                           &ego_front_s, &ego_front_l)) {
      std::cout << "ValidityCheck: ego out of lane" << std::endl;
      return false;
    }
    double max_l = std::max(std::abs(node->GetL()), std::abs(ego_front_l));
    if (max_l > l_limit_) {
      return false;
    }

    // virtual line check
    // 只允许车辆从绕行方向通过
    // if (virtual_lineseg_tree_ != nullptr) {
    //   const auto* nearest_object =
    //       virtual_lineseg_tree_->GetNearestObject(back_axis_position);
    //   if (nearest_object != nullptr) {
    //     const auto* nearest_lineseg = nearest_object->line_segment();
    //     if (nearest_lineseg != nullptr) {
    //       if (bounding_box.DistanceTo(*nearest_lineseg) < kCollisionBuffur) {
    //         return false;
    //       }
    //     }
    //   }
    // }
  }
  node->SetMinDist(min_dist);
  return true;
}

bool HybridARAStar::LeftOrRightTurn() {
  ReferencePathPoint temp_ref_path_point;
  std::array<int8_t, 3> s_range{11, 4, -1};
  for (auto s : s_range) {
    if (reference_path_ptr_->get_reference_point_by_lon(ego_s_ + s,
                                                        temp_ref_path_point)) {
      if (std::abs(temp_ref_path_point.path_point.kappa()) >
          1 / kBendRoadRadius) {
        if (temp_ref_path_point.path_point.kappa() > 0) {
          left_turn_ = true;
          return true;
        } else {
          right_turn_ = true;
          return true;
        }
      }
    }
  }
  return false;
}

bool HybridARAStar::DetectBend() {
  ReferencePathPoint temp_ref_path_point;
  std::array<int8_t, 5> s_range{-1, 5, 10, 15, 20};
  for (auto s : s_range) {
    if (reference_path_ptr_->get_reference_point_by_lon(ego_s_ + s,
                                                        temp_ref_path_point)) {
      if (std::abs(temp_ref_path_point.path_point.kappa()) >
          1 / kBendRoadRadius) {
        return true;
      }
    }
  }
  return false;
}

double HybridARAStar::ReferencePathLength() {
  constexpr double kMaxAcc = 0.2;
  constexpr double kMinAcc = -5.5;
  double cruise_v = session_->planning_context().v_ref_cruise();
  double ego_v = std::max(
      init_v_,
      std::min(hpp_general_lateral_decider_config_.min_v_cruise, cruise_v));
  // if (CalCruiseVelByCurvature(ego_v, flane->get_center_line(), cruise_v)) {
  //   limit_ref_vel_on_ramp_valid = true;
  // }
  double s = 0.0;
  double span_t = hpp_general_lateral_decider_config_.delta_t *
                  hpp_general_lateral_decider_config_.num_step;
  if (ego_v < cruise_v) {
    double t = (cruise_v - ego_v) / kMaxAcc;
    if (t > span_t) {
      s = ego_v * span_t + 0.5 * kMaxAcc * span_t * span_t;
    } else {
      s = ego_v * t + 0.5 * kMaxAcc * t * t;
      s += (span_t - t) * cruise_v;
    }
  } else {
    double t = (cruise_v - ego_v) / kMinAcc;
    if (t > span_t) {
      s = ego_v * span_t + 0.5 * kMinAcc * span_t * span_t;
    } else {
      s = ego_v * t + 0.5 * kMinAcc * t * t;
      s += (span_t - t) * cruise_v;
    }
  }
  const double max_ref_length =
      session_->planning_context().v_ref_cruise() * span_t;
  s = std::min(s, max_ref_length);
  return s;
}

void HybridARAStar::CalculateSearchBounds(
    const std::vector<TrajectoryPoint>& plan_history_traj) {
  // TODO: this may wrong in u turn and could be very large for left right turn
  // this XYbounds is only used for get the node index
  // bound check is now used ego lane.
  XYbounds_.clear();
  // 找到上一帧轨迹和A*搜索起止点的最大，最小x和y。将这个x和y都往外扩了50m
  double min_x = std::min(start_pose_.x, end_pose_.x);
  double max_x = std::max(start_pose_.x, end_pose_.x);
  for (auto traj_poit : plan_history_traj) {
    min_x = std::min(traj_poit.x, min_x);
    max_x = std::max(traj_poit.x, max_x);
  }
  XYbounds_.push_back(min_x - kExtendARASearchRange);
  XYbounds_.push_back(max_x + kExtendARASearchRange);
  double min_y = std::min(start_pose_.y, end_pose_.y);
  double max_y = std::max(start_pose_.y, end_pose_.y);
  for (const auto& traj_poit : plan_history_traj) {
    min_y = std::min(traj_poit.y, min_y);
    max_y = std::max(traj_poit.y, max_y);
  }
  XYbounds_.push_back(min_y - kExtendARASearchRange);
  XYbounds_.push_back(max_y + kExtendARASearchRange);
}

bool HybridARAStar::SetStartAndEndPose(
    const PlanningInitPoint& planning_init_point,
    const std::shared_ptr<KDPath>& fix_lane,
    const vector<TrajectoryPoint>& plan_history_traj, const double target_v) {
  // double ego_s = 0.0;
  // double ego_l = 0.0;
  // if (!fix_lane->XYToSL(planning_init_point.x, planning_init_point.y, &ego_s,
  //                       &ego_l)) {
  //   std::cout << "Hybrid ara*: planning_init_point frenet failed!!!"
  //             << std::endl;
  //   return false;
  // }
  // auto start_point = fix_lane.GetCenterLinePathPointByS(ego_s);
  // start_pose_.x = start_point.x();
  // start_pose_.y = start_point.y();
  // start_pose_.theta = start_point.theta();

  // 规划起点 或 规划起点投影点
  start_pose_.x = planning_init_point.x;
  start_pose_.y = planning_init_point.y;
  start_pose_.theta = planning_init_point.heading_angle;

  double start_s = 0.0;
  double start_l = 0.0;
  if (!fix_lane_->XYToSL(start_pose_.x, start_pose_.y, &start_s, &start_l)) {
    std::cout << "start node out of lane" << std::endl;
    return false;
  }

  // 终点
  double s = ReferencePathLength();
  constexpr uint8_t kExtendS = 2;
  constexpr double kMinS = 13;
  if (in_bend_) {
    s = s + kExtendS;
  } else {
    s = std::max(s + kExtendS, hpp_min_search_range_);
  }
  if (!left_turn_ && !right_turn_) {
    s = std::max(kMinS, s);
  }
  double end_s = start_s + s;
  end_s = std::min(end_s, fix_lane->path_points().back().s() - 1);
  // 目标车道中心线上对应的终点，如果s超过了线长呢，在外层做了判断保护,直接搜索失败
  auto end_point = fix_lane->GetPathPointByS(end_s);

  end_pose_.x = end_point.x();
  end_pose_.y = end_point.y();
  end_pose_.theta = end_point.theta();
  if (!fix_lane_->XYToSL(end_pose_.x, end_pose_.y, &end_s_, &end_l_)) {
    std::cerr << "end node out of lane" << std::endl;
    return false;
  }
  std::cout << "  end s l: " << end_s_ << " " << end_l_ << std::endl;

  // 扩展搜索范围
  CalculateSearchBounds(plan_history_traj);

  // load nodes
  start_node_.reset(new Node3D(start_pose_.x, start_pose_.y, start_pose_.theta,
                               XYbounds_, x_grid_resolution_,
                               y_grid_resolution_, phi_grid_resolution_));
  end_node_.reset(new Node3D(end_pose_.x, end_pose_.y, end_pose_.theta,
                             XYbounds_, x_grid_resolution_, y_grid_resolution_,
                             phi_grid_resolution_));

  start_node_->SetS(start_s);
  start_node_->SetL(start_l);
  start_node_->SetTrajCost(0.0);
  // 与终点的s的差值，最小为0
  start_node_->SetHeuCost(CalculateBaseHeuCost(start_node_));
  end_node_->SetTrajCost(std::numeric_limits<double>::max());
  final_node_ = end_node_;

  return true;
}

void HybridARAStar::BuildVirturalKDTree(
    const std::vector<planning_math::LineSegment2d>& virtual_lineseg_vec) {
  std::vector<planning_math::GeometryObject> agent_objects;
  agent_objects.reserve(virtual_lineseg_vec.size());
  int i = 0;
  for (const auto& lin_seg : virtual_lineseg_vec) {
    agent_objects.emplace_back(lin_seg, i);
    i++;
  }

  planning_math::AABoxKDTreeParams kdtree_params;
  kdtree_params.max_depth = 8;
  kdtree_params.max_leaf_dimension = 25;
  kdtree_params.max_leaf_size = 4;
  virtual_lineseg_tree_ = std::make_shared<
      planning_math::AABoxKDTree2d<planning_math::GeometryObject>>(
      agent_objects, kdtree_params);
}

// void HybridARAStar::BuildAgentKDTree(
//     const std::vector<const Obstacle*>& nudge_agents) {
//   std::vector<planning_math::GeometryObject> agent_objects;
//   agent_objects.reserve(nudge_agents.size());
//   int i = 0;
//   for (const auto& agent : nudge_agents) {
//     if (agent == nullptr) {
//       continue;
//     }
//     const bool is_traffic_facilities = agent->is_traffic_facilities();
//     agent_objects.emplace_back(agent->perception_bounding_box(), i,
//     agent->id(),
//                                agent);
//     i++;
//   }

//   planning_math::AABoxKDTreeParams kdtree_params;
//   kdtree_params.max_depth = 8;
//   kdtree_params.max_leaf_dimension = 25;
//   kdtree_params.max_leaf_size = 4;
//   agent_box_tree_ = std::make_shared<
//       planning_math::AABoxKDTree2d<planning_math::GeometryObject>>(
//       agent_objects, kdtree_params);
// }

void HybridARAStar::ChooseDirection() {
  constexpr double kLThresehold = 2;
  const double DistanceThreshold = (ego_half_width_ + 0.2) * 2;
  for (const auto& frenet_obstacle : nudge_agents_) {
    if ((!no_right_ || !no_left_) &&
        frenet_obstacle->source_type() != SourceType::OCC) {
      const auto& polygon_sequence =
          frenet_obstacle->frenet_polygon_sequence()[0].second;
      double min_s = polygon_sequence.min_x();
      double max_s = polygon_sequence.max_x();
      double min_l = polygon_sequence.min_y();
      double max_l = polygon_sequence.max_y();
      if (min_s > ego_s_ + vehicle_param_.front_edge_to_rear_axle &&
          min_l < 0 && max_l > 0) {
        for (const auto& another_frenet_obstacle : nudge_agents_) {
          const auto& another_polygon_sequence =
              another_frenet_obstacle->frenet_polygon_sequence()[0].second;
          double another_min_s = another_polygon_sequence.min_x();
          double another_max_s = another_polygon_sequence.max_x();
          double another_min_l = another_polygon_sequence.min_y();
          double another_max_l = another_polygon_sequence.max_y();
          double start_s = std::max(min_s, another_min_s - 0.5);
          double end_s = std::min(max_s, another_max_s + 0.5);
          bool lon_overlap = start_s < end_s;
          if (another_frenet_obstacle->source_type() != SourceType::OCC &&
              another_min_s > ego_s_ + vehicle_param_.front_edge_to_rear_axle &&
              std::abs(another_frenet_obstacle->frenet_l() -
                       frenet_obstacle->frenet_l()) > 1 &&
              lon_overlap) {
            if (!no_right_ &&
                frenet_obstacle->frenet_l() >
                    another_frenet_obstacle->frenet_l() &&
                min_l - another_max_l < DistanceThreshold &&
                another_min_l < -kLThresehold &&
                frenet_obstacle->obstacle()->perception_polygon().DistanceTo(
                    another_frenet_obstacle->obstacle()->perception_polygon()) <
                    DistanceThreshold) {
              no_right_ = true;
            } else if (!no_left_ &&
                       frenet_obstacle->frenet_l() <
                           another_frenet_obstacle->frenet_l() &&
                       another_min_l - max_l < DistanceThreshold &&
                       another_max_l > kLThresehold &&
                       frenet_obstacle->obstacle()
                               ->perception_polygon()
                               .DistanceTo(another_frenet_obstacle->obstacle()
                                               ->perception_polygon()) <
                           DistanceThreshold) {
              no_left_ = true;
            }
          }
        }
      }
    }
  }
}

// 如果静止障碍物是需要nudge的，那将其加入到 bounding_box_vec_ ,
// virtual_lineseg_tree_ , agent_box_tree_
bool HybridARAStar::ProcessStaticAgents() {
  // std::vector<planning_math::LineSegment2d> virtual_lineseg_vec;
  uint32_t obs_num = 0;
  constexpr double kLBuffer = 4.5;
  constexpr double kCareLBuffer = 1.5;
  for (const auto& frenet_obstacle : reference_path_ptr_->get_obstacles()) {
    if (frenet_obstacle->b_frenet_valid() &&
        !frenet_obstacle->b_frenet_polygon_sequence_invalid()) {
      const auto& polygon =
          frenet_obstacle->frenet_polygon_sequence()[0].second;
      double min_s = polygon.min_x();
      double max_s = polygon.max_x();
      double min_l = polygon.min_y();
      double max_l = polygon.max_y();
      if (min_s < end_s_ + vehicle_param_.front_edge_to_rear_axle &&
          max_s > ego_s_ + rear_obs_s_ &&
          (std::abs(min_l) < kLBuffer || std::abs(max_l) < kLBuffer) &&
          EdtManager::FilterObstacleForAra(*frenet_obstacle)) {
        if (std::abs(min_l) < kCareLBuffer || std::abs(max_l) < kCareLBuffer) {
          obs_max_s_ = std::max(obs_max_s_, max_s);
          obs_min_s_ = std::min(obs_min_s_, min_s);
        }
        obs_num++;
        nudge_agents_.emplace_back(frenet_obstacle);

        // 注释掉bounding_box_vec_
        /*
        ara_star::SLBox2d sl_box;
        sl_box.box = frenet_obstacle->obstacle()->perception_bounding_box();
        // 找到agent的最大和最小的 s l
        sl_box.min_s = frenet_obstacle->frenet_obstacle_boundary().s_start;
        sl_box.max_s = frenet_obstacle->frenet_obstacle_boundary().s_end;
        sl_box.min_l = frenet_obstacle->frenet_obstacle_boundary().l_start;
        sl_box.max_l = frenet_obstacle->frenet_obstacle_boundary().l_end;
        sl_box.s = frenet_obstacle->frenet_s();
        sl_box.id = frenet_obstacle->id();
        bounding_box_vec_.emplace_back(sl_box);
        */

        // 注释掉virtual_lineseg_vec
        /*
        // generate virtual line segments for nudge obs
        double virtual_s = area_s;
        double virtual_l = area_l;
        double virtual_x = 0.0;
        double virtual_y = 0.0;
        if (lat_obstacle_decision.at(obstacle->id()) ==
            LatObstacleDecisionType::LEFT) {
          // -50
          virtual_l = l - kDistanceCrossLine;
        }
        if (lat_obstacle_decision.at(obstacle->id()) ==
            LatObstacleDecisionType::RIGHT) {
          // +50
          virtual_l = l + kDistanceCrossLine;
        }

        if (!fix_lane_->SLToXY(virtual_s, virtual_l, &virtual_x, &virtual_y))
        { continue;
        }
        // 原始点和偏移50后的点组成一个线段，限制自车只能往一个方向通过
        planning_math::LineSegment2d virtural_line_segment(
            {obstacle->x_center(), obstacle->y_center()}, {virtual_x,
            virtual_y});
        virtual_lineseg_vec.emplace_back(virtural_line_segment);
        */
      }
    }
  }

  // only for static obs
  if (0 == obs_num) {
    return false;
  }

  // 将virtual_lineseg_vec中线段变成agent并放入virtual_lineseg_tree_
  // BuildVirturalKDTree(virtual_lineseg_vec);

  // 将nudge_agents中的agent放入agent_box_tree_
  // BuildAgentKDTree(nudge_agents_);

  return true;
}

void HybridARAStar::LogAgent() {
  auto& planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning_debug_data->mutable_hybrid_ara_info()
      ->mutable_ara_obstacles()
      ->Clear();
  for (auto& obstacle_ptr : nudge_agents_) {
    planning::common::HybridARAObstacle* obstacle =
        planning_debug_data->mutable_hybrid_ara_info()->add_ara_obstacles();
    obstacle->set_id(obstacle_ptr->obstacle()->id());
    obstacle->set_x_center(obstacle_ptr->obstacle()->x_center());
    obstacle->set_y_center(obstacle_ptr->obstacle()->y_center());
    obstacle->set_heading_angle(obstacle_ptr->obstacle()->heading_angle());
    obstacle->set_x_relative_center(
        obstacle_ptr->obstacle()->x_relative_center());
    obstacle->set_y_relative_center(
        obstacle_ptr->obstacle()->y_relative_center());
    obstacle->set_relative_heading_angle(
        obstacle_ptr->obstacle()->relative_heading_angle());
    obstacle->set_length(obstacle_ptr->obstacle()->length());
    obstacle->set_width(obstacle_ptr->obstacle()->width());
    for (const auto& polygon :
         obstacle_ptr->obstacle()->perception_polygon().points()) {
      planning::common::Point2d* obstacle_polygon =
          obstacle->add_polygon_points();
      obstacle_polygon->set_x(polygon.x());
      obstacle_polygon->set_y(polygon.y());
    }
  }
}

void HybridARAStar::MergeCloseAgent() {
  bool merged = true;
  int id = 5900000;
  while (merged) {
    merged = false;
    std::unordered_set<size_t> erased_indices;
    std::vector<std::shared_ptr<planning::FrenetObstacle>> new_nudge_agents;
    for (size_t i = 0; i < nudge_agents_.size(); ++i) {
      if (!nudge_agents_[i]->b_frenet_valid() ||
          nudge_agents_[i]->b_frenet_polygon_sequence_invalid()) {
        continue;
      }
      if (erased_indices.count(i)) {
        continue;
      }
      auto current = nudge_agents_[i];
      for (size_t j = i + 1; j < nudge_agents_.size(); ++j) {
        if (!nudge_agents_[j]->b_frenet_valid() ||
            nudge_agents_[j]->b_frenet_polygon_sequence_invalid() ||
            !current->b_frenet_valid() ||
            current->b_frenet_polygon_sequence_invalid()) {
          continue;
        }
        if (erased_indices.count(j)) {
          continue;
        }
        if (IsClose(current, nudge_agents_[j])) {
          id++;
          current = MergeAgents(id, current, nudge_agents_[j]);
          erased_indices.insert(j);
          merged = true;
        }
      }
      new_nudge_agents.push_back(current);
    }
    nudge_agents_ = std::move(new_nudge_agents);
  }
}

bool HybridARAStar::IsClose(
    const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
    const std::shared_ptr<planning::FrenetObstacle>& another_frenet_obstacle) {
  const double DistanceThreshold = ego_half_width_ * 2 + 0.15 * 2;
  constexpr double kMinDistance = 1.0;
  const auto& sequence = frenet_obstacle->frenet_polygon_sequence()[0].second;
  const auto& another_sequence =
      another_frenet_obstacle->frenet_polygon_sequence()[0].second;
  double min_s = sequence.min_x();
  double max_s = sequence.max_x();
  double min_l = sequence.min_y();
  double max_l = sequence.max_y();
  double another_min_s = another_sequence.min_x();
  double another_max_s = another_sequence.max_x();
  double another_min_l = another_sequence.min_y();
  double another_max_l = another_sequence.max_y();
  double start_s = std::max(min_s, another_min_s - 0.5);
  double end_s = std::min(max_s, another_max_s + 0.5);
  bool lon_overlap = start_s < end_s;

  if (min_s > ego_s_ + vehicle_param_.front_edge_to_rear_axle &&
      another_min_s > ego_s_ + vehicle_param_.front_edge_to_rear_axle &&
      lon_overlap &&
      ((frenet_obstacle->frenet_l() > another_frenet_obstacle->frenet_l() &&
        min_l - another_max_l > kMinDistance &&
        min_l - another_max_l < DistanceThreshold) ||
       (frenet_obstacle->frenet_l() < another_frenet_obstacle->frenet_l() &&
        another_min_l - max_l > kMinDistance &&
        another_min_l - max_l < DistanceThreshold)) &&
      frenet_obstacle->obstacle()->perception_polygon().DistanceTo(
          another_frenet_obstacle->obstacle()->perception_polygon()) <
          DistanceThreshold) {
    return true;
  }
  return false;
}

std::shared_ptr<planning::FrenetObstacle> HybridARAStar::MergeAgents(
    int id, const std::shared_ptr<planning::FrenetObstacle>& frenet_obstacle,
    const std::shared_ptr<planning::FrenetObstacle>& another_frenet_obstacle) {
  auto mergePoints =
      [](const std::shared_ptr<planning::FrenetObstacle>& obstacle,
         std::vector<planning_math::Vec2d>& points) {
        if (obstacle->source_type() == SourceType::OD) {
          const auto& corners =
              obstacle->obstacle()->perception_bounding_box().GetAllCorners();
          points.insert(points.end(), corners.begin(), corners.end());
        } else {
          const auto& perception_points =
              obstacle->obstacle()->perception_points();
          points.insert(points.end(), perception_points.begin(),
                        perception_points.end());
        }
      };

  std::vector<planning_math::Vec2d> merged_points;
  mergePoints(frenet_obstacle, merged_points);
  mergePoints(another_frenet_obstacle, merged_points);

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  auto obstacle = std::make_shared<Obstacle>(id, merged_points);
  obstacle_pool_.push_back(obstacle);
  return std::make_shared<planning::FrenetObstacle>(
      obstacle.get(), *reference_path_ptr_, ego_state_manager, true);
}

void HybridARAStar::FindClosestUncoveredInterval() {
  const double kMinLength = ego_half_width_ * 2 + 0.15 * 2;
  const double kMaxLength = ego_half_width_ * 2 + 0.5 * 2;
  constexpr double kConsiderS = 16.0;
  std::vector<std::pair<double, double>> intervals;
  intervals.reserve(nudge_agents_.size());
  for (const auto& frenet_obstacle : nudge_agents_) {
    double min_s = frenet_obstacle->frenet_polygon_sequence()[0].second.min_x();
    if (min_s > ego_s_ + vehicle_param_.front_edge_to_rear_axle + 0.3 &&
        min_s < ego_s_ + kConsiderS) {
      intervals.emplace_back(
          frenet_obstacle->frenet_polygon_sequence()[0].second.min_y(),
          frenet_obstacle->frenet_polygon_sequence()[0].second.max_y());
    }
  }

  std::sort(intervals.begin(), intervals.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  // 合并区间并查找未覆盖的目标区间
  bool find_pass_interval = false;
  double min_distance = std::numeric_limits<double>::max();
  double last_end = intervals[0].second;

  for (size_t i = 1; i < intervals.size(); ++i) {
    if (intervals[i].first <= last_end) {
      // 合并重叠区间
      last_end = std::max(last_end, intervals[i].second);
    } else {
      double gap_start = last_end;
      double gap_end = intervals[i].first;
      double gap_length = gap_end - gap_start;
      if (gap_length > kMinLength) {
        double distance_to_zero =
            std::min(std::abs(gap_start), std::abs(gap_end));
        if (distance_to_zero < min_distance) {
          min_distance = distance_to_zero;
          pass_interval_ = {gap_start, gap_end};
          find_pass_interval = true;
        }
      }
      last_end = intervals[i].second;
    }
  }

  if (find_pass_interval) {
    double gap_start = pass_interval_.first;
    double gap_end = pass_interval_.second;
    double gap_length = gap_end - gap_start;
    if (gap_length > kMaxLength) {
      if (gap_start < -1 * kMaxLength / 2 && gap_end > kMaxLength / 2) {
        pass_interval_ = {0, 0};
        return;
      }
      if (std::abs(gap_start) < std::abs(gap_end)) {
        gap_end = gap_start + kMaxLength;
      } else {
        gap_start = gap_end - kMaxLength;
      }
      pass_interval_ = {gap_start, gap_end};
    }
  }
}

void HybridARAStar::Reset() {
  // initialization
  open_pq_ = decltype(open_pq_)();
  open_set_.clear();
  close_set_.clear();
  observed_set_.clear();
  bounding_box_vec_.clear();
  nudge_agents_.clear();
  obstacle_pool_.clear();
  pass_interval_ = {0.0, 0.0};
  expand_num_ = 0;
  num_node_expand_ = 0;
  no_left_ = false;
  no_right_ = false;
  in_bend_ = false;
  left_turn_ = false;
  right_turn_ = false;
  obs_max_s_ = std::numeric_limits<double>::lowest();
  obs_min_s_ = std::numeric_limits<double>::max();
  heuristic_factor_ = hybrid_ara_star_conf_.heuristic_factor;
  final_node_ = nullptr;
  auto& planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  planning_debug_data->mutable_hybrid_ara_info()
      ->mutable_hybrid_ara_expand()
      ->Clear();
}

bool HybridARAStar::Init(const SearchResult search_result) {
  // 根据上一次搜索结果调整膨胀参数
  if (search_result == SearchResult::NO_SEARCH) {
    longitudinal_extend_ = hybrid_ara_star_conf_.longitudinal_extend;
    lateral_extend_ = hybrid_ara_star_conf_.lateral_extend;
  } else if (search_result == SearchResult::SUCCESS) {
    longitudinal_extend_ = hybrid_ara_star_conf_.longitudinal_extend - 0.05;
    lateral_extend_ = hybrid_ara_star_conf_.lateral_extend - 0.05;
  } else if (search_result == SearchResult::FAILED) {
    longitudinal_extend_ = hybrid_ara_star_conf_.longitudinal_extend + 0.05;
    lateral_extend_ = hybrid_ara_star_conf_.lateral_extend + 0.05;
  }

  const auto& coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  lane_change_direction_ =
      ((coarse_planning_info.target_state == kLaneChangeExecution) ||
       (coarse_planning_info.target_state == kLaneChangeComplete));
  // disable lane_change condition
  if (lane_change_direction_) {
    return false;
  }

  const auto& reference_path_ptr = coarse_planning_info.reference_path;
  if (reference_path_ptr) {
    reference_path_ptr_ = reference_path_ptr;
    fix_lane_ = reference_path_ptr->get_frenet_coord();
    ego_s_ = reference_path_ptr->get_frenet_ego_state().s();
    ego_l_ = reference_path_ptr->get_frenet_ego_state().l();
  } else {
    std::cout << "Error!!! fix_lane_ is nullptr" << std::endl;
    return false;
  }

  double x_dist = reference_path_ptr_->get_points().front().path_point.x() -
                  ref_init_point_x_;
  double y_dist = reference_path_ptr_->get_points().front().path_point.y() -
                  ref_init_point_y_;
  if (ego_s_ < last_ego_s_) {
    compensation_s_ = std::hypot(x_dist, y_dist);
  } else {
    compensation_s_ = -1 * std::hypot(x_dist, y_dist);
  }
  ref_init_point_x_ = reference_path_ptr_->get_points().front().path_point.x();
  ref_init_point_y_ = reference_path_ptr_->get_points().front().path_point.y();
  last_ego_s_ = ego_s_;

  const double target_vel = session_->mutable_environmental_model()
                                ->get_ego_state_manager()
                                ->ego_v_cruise();
  const PlanningInitPoint& planning_init_point = session_->environmental_model()
                                                     .get_ego_state_manager()
                                                     ->planning_init_point();
  init_v_ = planning_init_point.v;
  const auto& last_traj_points =
      session_->planning_context().last_planning_result().traj_points;

  // 最小是1，速度越大，这个值越大，则允许的前轮转角越小，魔数
  std::array<double, 6> xp{0.0, 3, 10.0, 20.0, 30.0, 40.0};
  std::array<double, 6> fp{max_front_wheel_angle_, 0.8, 0.3, 0.27, 0.24, 0.21};
  max_front_wheel_angle_ = interp(init_v_, xp, fp);
  // double steering_factor = std::max(1.0, init_v_ / 10.0);
  // max_front_wheel_angle_ = max_front_wheel_angle_ / steering_factor;

  /*
  // calculate RB buffer
  lane_width_ = session_->planning_context()
                    .lateral_behavior_planner_output()
                    .flane_width;
  hard_safe_distance_ = 0.15;
  // soft bound比hard bound最小多了0.2，速度越大，bound越窄
  soft_safe_distance_ =
      hard_safe_distance_ + std::max(0.01 * std::pow(init_v_ * 3.6, 0.75),
                                     boundary_soft_extra_buffer_);
  // 还是有可能比半车道留出的空间宽
  if (soft_safe_distance_ + ego_half_width_ > lane_width_ / 2.0) {
    soft_safe_distance_ =
        std::max(lane_width_ / 2.0 - ego_half_width_,
                 hard_safe_distance_ + 0.05);
  }
  */

  // 弯道判断
  in_bend_ = DetectBend();

  // 范围更小的弯道判断，for calcu cost
  if (LeftOrRightTurn()) {
    heuristic_factor_ *= 3;
  };

  // 设置搜索的起止点，以及搜索的x，y的范围
  auto time1 = (uint64_t)IflyTime::Now_ms();
  if (!SetStartAndEndPose(planning_init_point, fix_lane_, last_traj_points,
                          target_vel)) {
    return false;
  }
  auto time2 = (uint64_t)IflyTime::Now_ms();
  std::cout << "SetStartAndEndPose time: " << time2 - time1 << " ms"
            << std::endl;

  // prepare for edt
  edt_ = session_->environmental_model()
             .get_edt_manager()
             ->GetEulerDistanceTransform();
  edt_->Init(lateral_extend_, longitudinal_extend_, lateral_extend_);
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  ego_base_.SetBasePose(Pose2D(ego_state_manager->ego_pose().x,
                               ego_state_manager->ego_pose().y,
                               ego_state_manager->ego_pose().theta));

  return true;
}
bool HybridARAStar::Plan(ara_star::HybridARAStarResult& result,
                         const SearchResult search_result) {
  // HARA* begins
  std::cout << std::endl;
  std::cout << "==== start hybrid ara star search ==== " << std::endl;

  auto time1 = (uint64_t)IflyTime::Now_ms();
  Reset();
  auto time2 = (uint64_t)IflyTime::Now_ms();
  std::cout << "ARA star Reset time: " << time2 - time1 << " ms" << std::endl;

  if (!Init(search_result)) {
    return false;
  }
  auto time3 = (uint64_t)IflyTime::Now_ms();
  std::cout << "ARA star Init time: " << time3 - time2 << " ms" << std::endl;

  // 如果静止障碍物是需要nudge的，那将其加入到 bounding_box_vec_ ,
  // virtual_lineseg_tree_ , agent_box_tree_
  if (!ProcessStaticAgents()) {
    return false;
  }
  auto time4 = (uint64_t)IflyTime::Now_ms();
  std::cout << "ProcessStaticAgents time: " << time4 - time3 << " ms"
            << std::endl;

  if (!in_bend_) {
    ChooseDirection();
  }
  auto time5 = (uint64_t)IflyTime::Now_ms();
  std::cout << "ChooseDirection time: " << time5 - time4 << " ms" << std::endl;

  if (!no_right_ && !no_left_) {
    // MergeCloseAgent();
  }
  auto time6 = (uint64_t)IflyTime::Now_ms();
  std::cout << "MergeCloseAgent time: " << time6 - time5 << " ms" << std::endl;

  LogAgent();

  FindClosestUncoveredInterval();
  auto time7 = (uint64_t)IflyTime::Now_ms();
  std::cout << "FindClosestUncoveredInterval time: " << time7 - time6 << " ms"
            << std::endl;

  // load open set, pq
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(),
                   start_node_->GetCost(heuristic_factor_));
  observed_set_.emplace(start_node_->GetIndex(), start_node_);
  start_search_time_ = (uint64_t)IflyTime::Now_ms();
  std::vector<double> expand_num_vec;
  std::cout << "======== ImprovePath start =========" << std::endl;
  if (!ImprovePath()) {
    std::cout << "init search fail no result,return middle search result"
              << std::endl;
    // 如果第一次没找到路径，就找到close_set_中代价最小的点，将它作为终点
    if (enable_middle_final_node_) {
      SetMiddleFinalNode();
      if (!GetResult(result)) {
        std::cout << "GetResult failed, middle search is empty" << std::endl;
        return false;
      }
      return true;
    }
  }
  std::cout << "======== ImprovePath end =========" << std::endl;
  std::cout << "init search num_node_expand_: " << num_node_expand_
            << std::endl;
  std::cout << "init search expand_num_: " << expand_num_ << std::endl;
  expand_num_vec.emplace_back(expand_num_);

  auto end_improve_time = (uint64_t)IflyTime::Now_ms();
  uint64_t diff = end_improve_time - start_search_time_;
  std::cout << "Init ImprovePath time: " << diff << " ms" << std::endl;

  // HACK(zkxie): 迭代可能造成轨迹抖动，暂时关闭
  if (!hybrid_ara_star_conf_.search_once) {
    // zkxie(TODO): 为什么是2？应该是1？
    while (heuristic_factor_ > kMinHeuristicFactor) {
      if (open_set_.empty()) {
        std::cout << "Exit!!! Open set empty" << std::endl;
        break;
      }
      if (diff > kSkipAppendSearchTimeLimit) {
        std::cout << "Exit!!! large time for init search: " << diff
                  << std::endl;
        break;
      }
      auto time1 = (uint64_t)IflyTime::Now_ms();
      UpdateHeuristicFactor();
      auto time2 = (uint64_t)IflyTime::Now_ms();
      std::cout << "UpdateHeuristicFactor time: " << time2 - time1 << " ms"
                << std::endl;
      std::cout << "heuristic_factor_ after reduce: " << heuristic_factor_
                << std::endl;
      UpdateOpenSetWithHeuristicFactor();
      auto time3 = (uint64_t)IflyTime::Now_ms();
      std::cout << "UpdateOpenSetWithHeuristicFactor time: " << time3 - time2
                << " ms" << std::endl;

      // zkxie(TODO): confirm
      close_set_.clear();
      auto start_t = (uint64_t)IflyTime::Now_ms();
      std::cout << "======== ImprovePath start =========" << std::endl;
      ImprovePath();
      std::cout << "======== ImprovePath end =========" << std::endl;
      auto end_t = (uint64_t)IflyTime::Now_ms();
      std::cout << "ImprovePath time: " << end_t - start_t << " ms"
                << std::endl;
      std::cout << "all num_node_expand_ till now: " << num_node_expand_
                << std::endl;
      std::cout << "expand_num_: " << expand_num_ << std::endl;
      expand_num_vec.emplace_back(expand_num_);
    }
  }
  auto append_search_time = (uint64_t)IflyTime::Now_ms();
  diff = append_search_time - end_improve_time;
  std::cout << "append search time: " << diff << " ms" << std::endl;
  std::cout << "Final heuristic_factor_: " << heuristic_factor_ << std::endl;
  std::cout << "kMinHeuristicFactor: " << kMinHeuristicFactor << std::endl;

  JSON_DEBUG_VECTOR("expand_num_vec", expand_num_vec, 0);
  JSON_DEBUG_VALUE("pass_interval_first", pass_interval_.first);
  JSON_DEBUG_VALUE("pass_interval_second", pass_interval_.second);

  auto result_start_t = (uint64_t)IflyTime::Now_ms();
  if (!GetResult(result)) {
    std::cout << "GetResult failed" << std::endl;
    return false;
  }
  last_result_s_.clear();
  if (result.s.size() > 5) {
    for (size_t i = 0; i < 5; ++i) {
      last_result_s_.push_back(result.s[i]);
    }
  }

  auto result_end_t = (uint64_t)IflyTime::Now_ms();
  std::cout << "GetResult time: " << result_end_t - result_start_t << " ms"
            << std::endl;

  auto end_time = (uint64_t)IflyTime::Now_ms();
  diff = end_time - start_search_time_;

  std::cout << "total search time: " << diff << " ms" << std::endl;
  return true;
}

}  // namespace planning
