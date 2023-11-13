#include "behavior_planners/real_time_lane_change_decider/real_time_lane_change_decider.h"
#include "planning_output_context.h"
#include "virtual_lane_manager.h"

namespace planning {

// RealTimeLaneChangeDecider::RealTimeLaneChangeDecider(const TaskConfig
// &config) : Decider(config) {
//   assert(config.has_lane_change_decider_config());
// }

RealTimeLaneChangeDecider::RealTimeLaneChangeDecider(
    planning::common::LaneChangeInfo lc_info)
    : lc_info_(lc_info) {}

// void RealTimeLaneChangeDecider::init(
//   std::shared_ptr<WorldModel> world_model,
//   std::shared_ptr<BaseLineInfo> baseline_info) {
//   Task::init(world_model, baseline_info);
// }

void RealTimeLaneChangeDecider::feed_config_and_target_cars(
    bool is_merging, RTLaneChangeParams params, double dis_to_change_point,
    std::vector<const planning::common::TrackedObjectInfo *> &target_cars,
    const planning::common::TrackedObjectInfo &lead_one, double v_ego) {
  is_merging_ = is_merging;
  params_ = params;
  dis_to_change_point_ = dis_to_change_point;
  target_cars_ = target_cars;
  lead_one_ = &lead_one;  // 这里有问题，用的是指针
  v_ego_ = v_ego;
}

bool RealTimeLaneChangeDecider::process() {
  LOG_NOTICE("----RealTimeLaneChangeDecider::process---- \n");
  /*  context_.mutable_planning_status()->lane_status.status =
       context.planning_status().lane_status.status;
   context_.mutable_planning_status()->v_limit =
       context.planning_status().v_limit;
  */
  // id = -1 stands for virtual front/back obstacle; id = -10 stands for no
  // available gap
  target_gap_ = std::make_pair(-10, -10);
  target_gap_cost_ = std::numeric_limits<double>::max();
  gap_list_.clear();
  // cur_lane_.clear();
  // target_lane_.clear();

  // only enable in lane change preparation stage
  // if (planning_status.lane_status.status != LaneStatus::Status::LANE_CHANGE
  // ||
  //     planning_status.lane_status.change_lane.status !=
  //         ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION) {
  //   context_.mutable_planning_status()->lane_status.change_lane.target_gap_obs
  //   =
  //       target_gap_;
  //   return TaskStatus::STATUS_SUCCESS;
  // }
  // current_lane_id_ = planning_status.lane_status.target_lane_id;
  // dis_to_change_point_ = std::numeric_limits<double>::max();
  // bool is_in_dash_line = false;
  // if (planning_status.lane_status.change_lane.direction == "left") {
  //   lane_change_direction_ = 1;
  //   target_lane_id_ = current_lane_id_ - 1;
  //   if (map_info.left_boundary_info().size() > 0 &&
  //       map_info.left_boundary_info()[0].type == MSD_LANE_BOUNDARY_TYPE_DASH)
  //       {
  //     dis_to_change_point_ = map_info.left_boundary_info()[0].length;
  //     is_in_dash_line = true;
  //   }
  // } else {
  //   lane_change_direction_ = -1;
  //   target_lane_id_ = current_lane_id_ + 1;
  //   if (map_info.right_boundary_info().size() > 0 &&
  //       map_info.right_boundary_info()[0].type ==
  //       MSD_LANE_BOUNDARY_TYPE_DASH) {
  //     dis_to_change_point_ = map_info.right_boundary_info()[0].length;
  //     is_in_dash_line = true;
  //   }
  // }
  // LOG_NOTICE("gap debug dis %f", dis_to_change_point_);
  // dis_to_change_point_ = map_info.lc_end_dis();
  lc_map_decision_ = lc_info_.lc_map_decision();
  current_lane_type_ = lc_info_.current_lane_type();

  v_limit_ = lc_info_.v_limit();
  // leader_car_id_ =
  // planning_status.lane_status.change_lane.origin_lane_leader_id;

  // if (!world_model_->get_baseline_info(current_lane_id_)) {
  //   LOG_NOTICE("invalid origin_lane_id %d", current_lane_id_);
  // }
  // if (!world_model_->get_baseline_info(target_lane_id_)) {
  //   LOG_NOTICE("invalid target_lane_id %d", target_lane_id_);
  // }
  // auto curr_baseline = world_model_->get_baseline_info(current_lane_id_);
  // auto target_baseline = world_model_->get_baseline_info(target_lane_id_);
  // debug for fov
  // if (target_baseline != nullptr) {
  //   auto blocked_s_sections = target_baseline->fov_in_lon_direction();
  // }
  // if (curr_baseline == nullptr || !curr_baseline->is_valid()) {
  //   return TaskStatus::STATUS_FAILED;
  // }
  // if (target_baseline == nullptr || !target_baseline->is_valid()) {
  //   return TaskStatus::STATUS_FAILED;
  // }
  // double dis_to_stopline =
  // world_model_->get_map_info().distance_to_stop_line(); bool
  // solid_lane_change_mode =
  // config_.lane_change_decider_config().enable_solid_lane_change; int
  // broken_down_car_count =
  // config_.lane_change_decider_config().broken_down_car_count; const double
  // kMinDetectDis = 90.0; bool clear_gap_below_broken_down_car = false; int
  // potential_broken_down_car_id{-100}; if (solid_lane_change_mode &&
  // dis_to_stopline < kMinDetectDis) {
  //   //TODO: update scenario context here
  //   auto& brokendown_car_in_solid_line =
  //   PlanningContext::Instance()->mutable_planning_status()->broken_down_car.broken_down_cars_map;
  //   if (!is_in_dash_line) {
  //     // when ego car in solid line, set lane change point in front of ego
  //     car dis_to_change_point_ = 0.0;
  //   }
  //   if (target_baseline->detect_brokendown_car_in_solidline(dis_to_stopline,
  //   dis_to_change_point_, potential_broken_down_car_id)) {
  //     LOG_NOTICE("detect broken down car[%d] in solidline",
  //     potential_broken_down_car_id); if
  //     (brokendown_car_in_solid_line.find(potential_broken_down_car_id) ==
  //     brokendown_car_in_solid_line.end()) {
  //       brokendown_car_in_solid_line.clear();
  //       brokendown_car_in_solid_line[potential_broken_down_car_id] = 1;
  //       return TaskStatus::STATUS_FAILED;
  //     } else {
  //       if (++brokendown_car_in_solid_line[potential_broken_down_car_id] >
  //       broken_down_car_count) {
  //         dis_to_change_point_ = dis_to_stopline;
  //         clear_gap_below_broken_down_car = true;
  //       } else {
  //         return TaskStatus::STATUS_FAILED;
  //       }
  //     }
  //   } else {
  //     brokendown_car_in_solid_line.clear();
  //   }
  // }
  // auto &obstacles =
  // world_model_->get_baseline_info(current_lane_id_)->obstacle_manager().get_obstacles().Items();
  // auto target_lane_frenet_coord = curr_baseline->get_frenet_coord();
  // if (target_lane_frenet_coord == nullptr) {
  //   return TaskStatus::STATUS_FAILED;
  // }

  // world_model_->trans_lane_points(curr_baseline->get_raw_refline_points(),
  //                                 cur_lane_, target_lane_frenet_coord);
  // world_model_->trans_lane_points(target_baseline->get_raw_refline_points(),
  //                                 target_lane_, target_lane_frenet_coord);

  // double ego_s = curr_baseline->get_ego_state().ego_frenet.x +
  // vehicle_param::length / 2; double v_ego =
  // curr_baseline->get_ego_state().ego_vel;

  // if (lead_car_.id > 0) {
  //   auto lead_obs =
  //     curr_baseline->mutable_obstacle_manager().find_obstacle(lead_car_.id);
  //   if (lead_obs == nullptr) {
  //     lead_car_.id = -1;
  //   }
  //   else {
  //     SLBoundary sl_boundary = lead_obs->PerceptionSLBoundary();
  //     if (sl_boundary.start_s - ego_s > 0) {
  //       lead_car_.d_rel = sl_boundary.start_s - ego_s;
  //     }
  //     else if (sl_boundary.end_s - ego_s < 0){
  //       lead_car_.d_rel = sl_boundary.end_s - ego_s;
  //     }
  //     else {
  //       lead_car_.d_rel = 0.0;
  //     }
  //     lead_car_.v_rel = lead_obs->speed() *
  //     std::cos(lead_obs->Yaw_relative_frenet()) - v_ego_;

  //   }
  // }

  if (lead_one_ != nullptr) {
    lead_car_.id = lead_one_->track_id();
    lead_car_.d_rel = lead_one_->d_rel();
    lead_car_.v_rel = lead_one_->v_rel();
  } else {
    lead_car_.id = -1;
  }

  most_front_car_.d_rel = params_.most_front_car_dist;
  most_rear_car_.d_rel = params_.most_rear_car_dist;
  obstacle_on_target_.clear();
  obstacle_on_target_.push_back(most_front_car_);
  obstacle_on_target_.push_back(most_rear_car_);
  // Obstacle* broken_down_car{nullptr};
  // if (clear_gap_below_broken_down_car) {
  //   broken_down_car =
  //     curr_baseline->mutable_obstacle_manager().find_obstacle(potential_broken_down_car_id);
  // }
  // for (auto &obstacle : obstacles) {
  //   SLBoundary sl_boundary = obstacle->PerceptionSLBoundary();
  //   if (is_on_target(sl_boundary)){
  //     TargetObstacle target_obs;
  //     target_obs.id = obstacle->Id();
  //     if (sl_boundary.start_s - ego_s > 0) {
  //       target_obs.d_rel = sl_boundary.start_s - ego_s;
  //     }
  //     else if (sl_boundary.end_s - ego_s < 0){
  //       target_obs.d_rel = sl_boundary.end_s - ego_s;
  //     }
  //     else {
  //       target_obs.d_rel = 0.0;
  //     }
  //     target_obs.v_rel = obstacle->speed() *
  //     std::cos(obstacle->Yaw_relative_frenet()) - v_ego_;
  //     obstacle_on_target_.push_back(target_obs);
  //     // LOG_NOTICE("gap obstacle[%d] in target_lane", obstacle->Id());
  //   }
  // }

  for (auto &obstacle : target_cars_) {
    TargetObstacle target_obs;
    target_obs.id = obstacle->track_id();
    target_obs.d_rel = obstacle->d_rel();
    target_obs.v_rel = obstacle->v_rel();
    obstacle_on_target_.push_back(target_obs);
  }

  std::sort(obstacle_on_target_.begin(), obstacle_on_target_.end(),
            compare_distance_asc);

  double preview_distance = 20 + std::max((v_ego_ - 10.0) / 15.0 * 20.0, 0.0);
  nearest_rear_car_track_ = obstacle_on_target_[0];
  for (int i = 0; i < obstacle_on_target_.size() - 1; i++) {
    double d_lead = 100.0;
    if (lead_car_.id > 0) {
      d_lead = lead_car_.d_rel;
    }
    if (obstacle_on_target_.at(i).d_rel > preview_distance ||
        obstacle_on_target_.at(i).d_rel > d_lead) {
      break;
    }
    if (is_merging_ && (obstacle_on_target_.at(i).d_rel < -60.0)) {
      continue;
    }

    if (obstacle_on_target_.at(i).d_rel < -5.0) {
      nearest_rear_car_track_ = obstacle_on_target_.at(i);
    }
    LOG_NOTICE("obstacle_on_target[%d]'s d_rel: [%f], v_rel: [%f] \n", i,
               obstacle_on_target_.at(i).d_rel,
               obstacle_on_target_.at(i).v_rel);
    LOG_NOTICE("obstacle_on_target[%d]'s d_rel: [%f], v_rel: [%f] \n", i + 1,
               obstacle_on_target_.at(i + 1).d_rel,
               obstacle_on_target_.at(i + 1).v_rel);
    GapInfo gap_info = check_gap_valid(obstacle_on_target_.at(i),
                                       obstacle_on_target_.at(i + 1));
    std::cout << "TBDEBUG Gap Info: valid: " << gap_info.valid
              << " front_id:" << gap_info.front_id
              << " rear_id: " << gap_info.rear_id << " cost: " << gap_info.cost
              << std::endl;
    if (gap_info.valid) {
      if (gap_info.front_id == lc_info_.target_gap_obs_first() &&
          gap_info.rear_id == lc_info_.target_gap_obs_second()) {
        gap_info.cost -= params_.cost_minus;
      }
      gap_list_.push_back(gap_info);
    }
  }
  if (gap_list_.size() > 0) {
    std::sort(gap_list_.begin(), gap_list_.end(), compare_cost_asc);
    target_gap_ =
        std::make_pair(gap_list_.at(0).front_id, gap_list_.at(0).rear_id);
    target_gap_cost_ = gap_list_.at(0).cost;
  }
  LOG_DEBUG("Target gap: [%d], [%d]\n", target_gap_.first, target_gap_.second);
  // if (clear_gap_below_broken_down_car &&
  // broken_down_car) {
  //   auto target_car =
  //     curr_baseline->mutable_obstacle_manager().find_obstacle(target_gap_.first);
  //   if (target_car == nullptr) {
  //     target_car =
  //     curr_baseline->mutable_obstacle_manager().find_obstacle(target_gap_.second);
  //   }
  //   if (target_car) {
  //     if (target_car->PerceptionSLBoundary().end_s <
  //     broken_down_car->PerceptionSLBoundary().start_s ||
  //         target_car->Id() == potential_broken_down_car_id) {
  //       context_.mutable_planning_status()->lane_status.change_lane.target_gap_obs
  //       = {-1, potential_broken_down_car_id}; target_gap_ = {-1,
  //       potential_broken_down_car_id}; target_gap_cost_ = 0.0; return
  //       TaskStatus::STATUS_SUCCESS;
  //     }
  //   }
  // }
  /*
  frame_->mutable_session()
      ->mutable_planning_output_context()
      ->mutable_planning_status()
      ->lane_status.change_lane.target_gap_obs = target_gap_;
  */
  return true;
}

bool RealTimeLaneChangeDecider::is_on_target(
    const FrenetObstacleBoundary &sl_boundary) {
  double s = (sl_boundary.s_end + sl_boundary.s_start) / 2.0;
  max_l_threshold_ = calc_lane_width(s, cur_lane_) / 2.0 +
                     calc_lane_width(s, target_lane_) / 2.0 + 1.3;
  // ROS_INFO("TBDEBUG: %f %f %d %f %f", s, max_l_threshold_,
  // lane_change_direction_, sl_boundary.start_l, sl_boundary.end_l);
  if (lane_change_direction_ > 0 &&
      sl_boundary.l_start > lane_change_direction_ * min_l_threshold_ &&
      sl_boundary.l_start < lane_change_direction_ * max_l_threshold_) {
    return true;
  } else if (lane_change_direction_ < 0 &&
             sl_boundary.l_end < lane_change_direction_ * min_l_threshold_ &&
             sl_boundary.l_end > lane_change_direction_ * max_l_threshold_) {
    return true;
  }
  return false;
}

GapInfo RealTimeLaneChangeDecider::check_gap_valid(
    const TargetObstacle &rear_car, const TargetObstacle &front_car) {
  double buffer = 2.0;
  double t_gap = 0.1;
  double car_length = 5.0;
  double ego_car_length = car_length + buffer;
  // double v_ego = baseline_info_->get_ego_state().ego_vel;
  GapInfo gap_info;
  TargetObstacle base_car;
  double v_ego_p, v_ego_p_rel;
  double safety_distance;

  gap_info.front_id = front_car.id;
  gap_info.rear_id = rear_car.id;
  gap_info.acc_valid = false;

  gap_info.s_front = front_car.d_rel;
  gap_info.v_front = front_car.v_rel + v_ego_;
  gap_info.s_rear = rear_car.d_rel;
  gap_info.v_rear = rear_car.v_rel + v_ego_;

  if (std::abs(front_car.d_rel) <
      std::abs(std::min(rear_car.d_rel + 5.0, front_car.d_rel - 0.1))) {
    base_car = front_car;
    v_ego_p = std::min(v_ego_ + base_car.v_rel - 1.0, v_limit_);
    v_ego_p_rel = std::min(base_car.v_rel - 1.0, v_limit_ - v_ego_);
  } else {
    base_car = rear_car;
    if (front_car.v_rel > rear_car.v_rel + 3.0 && rear_car.v_rel < 0.0) {
      gap_info.acc_valid = true;
    }
    v_ego_p = std::min(v_ego_ + base_car.v_rel + 1.0, v_limit_);
    v_ego_p_rel = std::min(base_car.v_rel + 1.0, v_limit_ - v_ego_);
  }
  safety_distance = get_lc_safe_dist(buffer, t_gap, v_ego_p);
  gap_info.base_car_id = base_car.id;
  gap_info.base_car_drel = base_car.d_rel;
  gap_info.base_car_vrel = base_car.v_rel;
  if (gap_info.acc_valid) {
    // set front car's vrel when acc_valid
    gap_info.base_car_vrel = front_car.v_rel;
  }

  double mssf =
      std::pow(std::max(-(front_car.v_rel - v_ego_p_rel), 0.0), 2) / 2.0 +
      safety_distance;
  double mssr =
      (rear_car.v_rel - v_ego_p_rel) * 2.0 +
      std::pow(std::max((rear_car.v_rel - v_ego_p_rel), 0.0), 2) / 2.0 +
      safety_distance;
  if (is_merging_) {
    mssf = -(front_car.v_rel - v_ego_p_rel) * 2.0 + safety_distance;
    mssr = (rear_car.v_rel - v_ego_p_rel) * 1.0 +
           std::pow(std::max((rear_car.v_rel - v_ego_p_rel), 0.0), 2) / 2.0 +
           safety_distance;
  }
  double acc_time = calc_time_for_lane_change(
      base_car, front_car, gap_info, safety_distance, v_limit_ - v_ego_ - 0.5);
  gap_info.acc_time = acc_time;
  // ROS_INFO("TBDEBUG: acc_time %f", acc_time);
  double v_target_p = 100.0;
  double d_p = 100.0;
  if (lead_car_.id > 0) {
    double d_des = calc_desired_distance(lead_car_.v_rel + v_ego_, v_ego_p);
    double gap_p = (acc_time + 2.0) *
                       (lead_car_.v_rel + v_ego_ - (v_ego_ + rear_car.v_rel)) +
                   lead_car_.d_rel - rear_car.d_rel - car_length;
    d_p = gap_p - safety_distance;
    v_target_p = calc_desired_speed(d_p, d_des, lead_car_.v_rel + v_ego_);
  }

  if (v_ego_p_rel < rear_car.v_rel + params_.v_rel_bufer &&
      rear_car.d_rel >
          -std::max(mssr + ego_car_length, safety_distance + ego_car_length)) {
    d_p = -1;
  }

  double mss = mssf + mssr + ego_car_length;
  double mss0 = 2 * safety_distance + ego_car_length;
  double gap = front_car.d_rel - rear_car.d_rel +
               acc_time * (front_car.v_rel - rear_car.v_rel);
  double lc_end_dis = dis_to_change_point_;
  double l_end_dis = 100.0;
  if ((lc_end_dis < 200.0) && (!is_merging_)) {
    double gap_end_dis = lc_end_dis -
                         (acc_time + 2.0) * (v_ego_ + rear_car.v_rel) -
                         rear_car.d_rel - car_length - safety_distance -
                         10 * v_ego_p * std::max(lc_map_decision_ - 1, 0);
    double ego_end_dis = lc_end_dis - acc_time * v_ego_ -
                         10 * v_ego_p * std::max(lc_map_decision_ - 1, 0);
    if ((lc_map_decision_ == -1) &&
        (current_lane_type_ == FusionRoad::LANETYPE_NORMAL)) {
      ego_end_dis = lc_end_dis - acc_time * v_ego_ - 3.0 * v_ego_p - 20.0;
    }
    l_end_dis = std::min(gap_end_dis, ego_end_dis);
  }
  if (is_merging_) {
    double gap_end_dis =
        lc_end_dis - acc_time * (v_ego_ + rear_car.v_rel) - rear_car.d_rel;
    d_p = std::min(gap_end_dis, d_p);
  }
  // ROS_INFO("TBDEBUG lc_debug: %f %f %f %f %f %f", gap, mss0, mss, d_p,
  // v_target_p, v_ego_p);
  if (gap > mss0 && gap > mss && d_p > 0 && v_target_p >= v_ego_p) {
    gap_info.valid = true;
    if (gap_info.acc_valid) {
      v_ego_p =
          std::min(std::max(0.0, v_ego_ + front_car.v_rel - 1.0), v_limit_);
      v_ego_p_rel = v_ego_p - v_ego_;
    }
    double dis_offset =
        (gap_info.rear_id == base_car.id) ? car_length : -car_length;
    double gap_cost =
        ((10.0 > rear_car.d_rel && rear_car.d_rel > -20.0) or
         (20.0 > front_car.d_rel && front_car.d_rel > -10.0))
            ? -std::min(std::max(front_car.d_rel - rear_car.d_rel - 20.0, 0.0),
                        20.0)
            : 0.0;
    double cost =
        std::pow(v_ego_p_rel, 2) +
        std::abs(base_car.d_rel + base_car.v_rel * 1.0 + dis_offset) * 1.5 +
        acc_time + std::abs(v_limit_ - v_ego_p) + gap_cost;
    if (l_end_dis < 0.0) {
      cost += 100.0;
      if (rear_car.d_rel > 5.0) {
        cost += 900.0;
      }
    }
    if (is_merging_) {
      double cost_minus = ((-40.0 < rear_car.d_rel) && (rear_car.d_rel) < 20.0)
                              ? -std::min(std::max(gap - 20.0, 0.0), 20.0)
                              : 0.0;
      cost = std::pow(v_ego_p_rel, 2) +
             std::abs(base_car.d_rel + base_car.v_rel * 1.0) * 1.5 + acc_time +
             abs(v_limit_ - v_ego_p) + cost_minus;
    }
    gap_info.cost = cost;
  } else {
    gap_info.valid = false;
    gap_info.cost = 0;
  }

  return gap_info;
}

double RealTimeLaneChangeDecider::calc_time_for_lane_change(
    TargetObstacle base_car, TargetObstacle front_car, GapInfo gap_info,
    const double safety_distance, const double max_v) {
  bool is_base_car_rear = gap_info.rear_id == base_car.id;
  double v_diff_max =
      (is_base_car_rear) ? std::min(3.0, max_v - base_car.v_rel) : 3.0;
  double v_diff_end = std::min(1.0, v_diff_max);
  if (gap_info.acc_valid) {
    v_diff_max = std::max(
        std::min(front_car.v_rel - 1.0, max_v) - base_car.v_rel, v_diff_max);
    v_diff_end = v_diff_max;
  }
  double a = 1.0;
  double d, d_offset, vrel;
  double acc_time = 0.0;
  if (is_base_car_rear) {
    d_offset = clip(base_car.d_rel + 5.0, 0, 5.0) + 5.0;
    d = safety_distance + base_car.d_rel + d_offset;
    vrel = -1 * base_car.v_rel;
  } else {
    d_offset = clip(base_car.d_rel, -5.0, 0.0);
    d = safety_distance - base_car.d_rel - d_offset;
    vrel = base_car.v_rel;
  }

  if (d <= 0) {
    return acc_time;
  }
  if (v_diff_max <= 0.0) {
    acc_time = 10000.0;
    return acc_time;
  }
  if (vrel > v_diff_max) {
    double d_dec_max = (std::pow(vrel, 2) - std::pow(v_diff_end, 2)) / (2 * a);
    if (d > d_dec_max) {
      acc_time = (d - d_dec_max) / v_diff_max + (vrel - v_diff_end) / a;
    } else {
      acc_time = (vrel - std::sqrt(std::pow(vrel, 2) - 2 * d * a)) / a;
    }
  } else {
    d += std::pow(std::min(vrel, 0.0), 2) / (2 * a);
    double d_dec = std::pow(v_diff_max - v_diff_end, 2) / (2 * a);
    if (d <
        d_dec + (std::pow(v_diff_max, 2) - std::pow(std::max(vrel, 0.0), 2)) /
                    (2 * a)) {
      double v_m = std::sqrt(d + std::pow(std::max(vrel, 0.0), 2) / (2 * a) +
                             std::pow(v_diff_end, 2) / (2 * a));
      if (v_m >= vrel && v_m >= v_diff_end) {
        acc_time = (v_m - vrel + v_m - v_diff_end) / a;
      } else if (vrel > v_diff_end) {
        acc_time = (vrel - std::sqrt(std::pow(vrel, 2) - 2 * d * a)) / a;
      } else {
        acc_time = (std::sqrt(std::pow(vrel, 2) + 2 * d * a) - vrel) / a;
      }
    } else {
      acc_time = (d + std::pow(v_diff_max - std::max(vrel, 0.0), 2) / (2 * a) +
                  d_dec) /
                     v_diff_max +
                 std::abs(std::min(vrel / a, 0.0));
    }
  }
  return acc_time;
}

double RealTimeLaneChangeDecider::clip(const double x, const double lo,
                                       const double hi) {
  return std::max(lo, std::min(hi, x));
}

double RealTimeLaneChangeDecider::calc_lane_width(
    const double &s, const std::vector<RefPointFrenet> &ref_line) {
  double lane_width = 3.8;
  if (ref_line.size() == 0) {
    return lane_width;
  } else if (s < ref_line.front().s) {
    lane_width = ref_line.front().lane_width;
    return clip(lane_width, 3.0, 4.0);
  } else if (s > ref_line.back().s) {
    lane_width = ref_line.back().lane_width;
    return clip(lane_width, 3.0, 4.0);
  }
  for (int i = 0; i < ref_line.size() - 1; i++) {
    if (s >= ref_line.at(i).s && s <= ref_line.at(i + 1).s) {
      double k =
          (s - ref_line.at(i).s) / (ref_line.at(i + 1).s - ref_line.at(i).s);
      lane_width =
          ref_line.at(i).lane_width +
          (ref_line.at(i + 1).lane_width - ref_line.at(i).lane_width) * k;
    }
  }
  return clip(lane_width, 3.0, 4.0);
}

double RealTimeLaneChangeDecider::calc_desired_distance(const double v_lead,
                                                        const double v_ego) {
  double t_gap = interp(v_ego, _T_GAP_VEGO_BP, _T_GAP_VEGO_V);
  // if (lc_request != "none") {
  //   t_gap = t_gap * (0.6 + v_ego * 0.01);
  // }
  if (!is_merging_) {
    t_gap = t_gap * (0.6 + v_ego * 0.01);
  }
  // if (is_temp_lead) {
  //   t_gap = t_gap * 0.3;
  // }

  // distance when at zero speed
  double d_offset = 3.5;
  // if (is_accident_car) {
  //   d_offset = 6;
  // }
  return d_offset + v_lead * t_gap;
}

double RealTimeLaneChangeDecider::calc_desired_speed(const double d_lead,
                                                     const double d_des,
                                                     const double v_lead) {
  // *** compute desired speed ***
  // the desired speed curve is divided in 4 portions:
  // 1-constant
  // 2-linear to regain distance
  // 3-linear to shorten distance
  // 4-parabolic (constant decel)
  const double max_runaway_speed = -2.;  // no slower than 2m/s over the lead
  //  interpolate the lookups to find the slopes for a give lead speed
  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);
  // this is where parabola && linear curves are tangents
  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  // parabola offset to have the parabola being tangent to the linear curve
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));

  double v_rel_des = 0.0;
  if (d_lead < d_des) {
    // calculate v_rel_des on the line that connects 0m at max_runaway_speed
    // to d_des
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_lead - d_des);
    // calculate v_rel_des on one third of the linear slope
    double v_rel_des_2 = (d_lead - d_des) * l_slope / 3.0;
    // take the min of the 2 above
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else if (d_lead < d_des + x_linear_to_parabola) {
    v_rel_des = (d_lead - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else {
    v_rel_des = std::sqrt(2 * (d_lead - d_des - x_parabola_offset) * p_slope);
  }
  // compute desired speed
  double v_target = v_rel_des + v_lead;

  return v_target;
}

double RealTimeLaneChangeDecider::interp(double x,
                                         const std::vector<double> &xp,
                                         const std::vector<double> &fp) {
  const int N = xp.size() - 1;

  if (x < xp[0]) {
    return fp[0];
  }
  for (int i = 0; i <= N; ++i) {
    if (x < xp[i]) {
      return ((x - xp[i - 1]) * (fp[i] - fp[i - 1]) / (xp[i] - xp[i - 1]) +
              fp[i - 1]);
    }
  }

  return fp[N];
}

bool RealTimeLaneChangeDecider::compare_distance_asc(
    const TargetObstacle &obs1, const TargetObstacle &obs2) {
  return obs1.d_rel < obs2.d_rel;
}
bool RealTimeLaneChangeDecider::compare_cost_asc(const GapInfo &gap1,
                                                 const GapInfo &gap2) {
  return gap1.cost < gap2.cost;
}
}  // namespace planning
