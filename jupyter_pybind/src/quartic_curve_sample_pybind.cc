
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <sys/param.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "behavior_planners/sample_poly_speed_adjust_decider/sample_space_base.h"
#include "jupyter_pybind/src/serialize_utils.h"
#include "src/modules/common/trajectory1d/quartic_poly_trajectory1d.h"
#include "src/modules/tasks/behavior_planners/sample_poly_speed_adjust_decider/sample_poly_curve.h"
#include "src/modules/tasks/behavior_planners/sample_poly_speed_adjust_decider/sample_poly_speed_adjust_decider.h"
#include "src/modules/tasks/behavior_planners/sample_poly_speed_adjust_decider/sample_speed_adjust_cost.h"
#include "st_graph/st_point.h"
#include "st_search_decider.pb.h"
namespace py = pybind11;
using namespace planning;

static SamplePolySpeedAdjustDecider* pSamplePolySpeedAdjustDecider;

int Init() {
  pSamplePolySpeedAdjustDecider = new SamplePolySpeedAdjustDecider();
  return 1;
}

int Execute() {
  bool ok = true;
  if (ok) {
    ok = pSamplePolySpeedAdjustDecider->SamplePolys();
  }

  if (ok) {
    ok = pSamplePolySpeedAdjustDecider->Evaluate();
  }

  if (ok) {
    if (pSamplePolySpeedAdjustDecider->min_cost_traj_ptr() != nullptr) {
      ok = pSamplePolySpeedAdjustDecider->BestTrajCheck();
    }
  }

  if (!ok) {
    pSamplePolySpeedAdjustDecider->set_min_cost_traj_ptr(nullptr);
  }
  return 1;
}

void ClearDeciderInfo() {
  pSamplePolySpeedAdjustDecider->mutable_agent_info().clear();
  pSamplePolySpeedAdjustDecider->mutable_st_sample_space_base().Clear();
  pSamplePolySpeedAdjustDecider->mutable_sample_trajs().clear();
  pSamplePolySpeedAdjustDecider->set_min_cost_traj_ptr(nullptr);
  pSamplePolySpeedAdjustDecider->mutable_leading_veh() = AgentInfo();
}

int ProcessInput(py::bytes& sample_poly_input_bytes) {
  ClearDeciderInfo();

  planning::common::SamplePolySpeedInfo sample_poly_input =
      BytesToProto<planning::common::SamplePolySpeedInfo>(
          sample_poly_input_bytes);

  const auto& agent_info_input = sample_poly_input.agent_infos();
  if (agent_info_input.size() == 0) {  // No speed adjust info
    return 0;
  }

  // agent info
  std::vector<AgentInfo> agent_infos;
  agent_infos.reserve((agent_info_input.size()));
  for (auto& agent_input : agent_info_input) {
    AgentInfo agent{agent_input.id(), agent_input.center_s(),
                    agent_input.half_length(), agent_input.v()};
    agent_infos.emplace_back(std::move(agent));
  }
  std::sort(agent_infos.begin(), agent_infos.end(),
            [](AgentInfo& a, AgentInfo& b) -> bool {
              return a.center_s < b.center_s;
            });
  // leading info
  AgentInfo leading_veh{sample_poly_input.leading_veh_info().id(),
                        sample_poly_input.leading_veh_info().center_s(),
                        sample_poly_input.leading_veh_info().half_length(),
                        sample_poly_input.leading_veh_info().v()};

  pSamplePolySpeedAdjustDecider->mutable_agent_info() = agent_infos;
  pSamplePolySpeedAdjustDecider->mutable_leading_veh() = leading_veh;
  pSamplePolySpeedAdjustDecider->set_ego_s(
      sample_poly_input.sample_print_table_info().ego_s());
  pSamplePolySpeedAdjustDecider->set_ego_v(
      sample_poly_input.sample_print_table_info().ego_v());
  pSamplePolySpeedAdjustDecider->set_ego_a(
      sample_poly_input.sample_print_table_info().ego_acc());
  pSamplePolySpeedAdjustDecider->set_v_suggestted(
      sample_poly_input.sample_print_table_info().v_suggestted());

  pSamplePolySpeedAdjustDecider->mutable_st_sample_space_base().Init(
      agent_infos, sample_poly_input.sample_print_table_info().ego_s());

  pSamplePolySpeedAdjustDecider->set_delta_t(
      sample_poly_input.sample_param().sample_delta_t());
  pSamplePolySpeedAdjustDecider->set_delta_v(
      sample_poly_input.sample_param().sample_delta_v());
  pSamplePolySpeedAdjustDecider->set_speed_adjust_range(
      std::make_pair(sample_poly_input.sample_param().sample_upper_v(),
                     sample_poly_input.sample_param().sample_lower_v()));

  pSamplePolySpeedAdjustDecider->set_weight_follow_vel(
      sample_poly_input.sample_param().weight_follow_vel());
  pSamplePolySpeedAdjustDecider->set_weight_match_gap_s(
      sample_poly_input.sample_param().weight_match_gap_s());
  pSamplePolySpeedAdjustDecider->set_weight_match_gap_vel(
      sample_poly_input.sample_param().weight_match_gap_vel());
  pSamplePolySpeedAdjustDecider->set_weight_stop_line(
      sample_poly_input.sample_param().weight_stop_line());
  pSamplePolySpeedAdjustDecider->set_weight_leading_safe_s(
      sample_poly_input.sample_param().weight_leading_safe_s());
  pSamplePolySpeedAdjustDecider->set_weight_leading_safe_v(
      sample_poly_input.sample_param().weight_leading_safe_v());
  pSamplePolySpeedAdjustDecider->set_weight_vel_variable(
      sample_poly_input.sample_param().weight_vel_variable());
  pSamplePolySpeedAdjustDecider->set_weight_gap_avaliable(
      sample_poly_input.sample_param().weight_gap_avaliable());
  pSamplePolySpeedAdjustDecider->set_weight_acc_limit(
      sample_poly_input.sample_param().weight_acc_limit());
  pSamplePolySpeedAdjustDecider->set_weight_stop_penalty(
      sample_poly_input.sample_param().weight_stop_line());

  pSamplePolySpeedAdjustDecider->CalcTargetLaneObjsFlowVel();
  pSamplePolySpeedAdjustDecider->CalcTargetLaneVehDensity();

  switch (sample_poly_input.sample_print_table_info().sample_scene()) {
    case 0:
      pSamplePolySpeedAdjustDecider->set_sample_scene(
          SampleScene::NormalSampleScene);
      break;
    case 1:
      pSamplePolySpeedAdjustDecider->set_sample_scene(
          SampleScene::PurseFlowVelScene);
      break;
    case 2:
      pSamplePolySpeedAdjustDecider->set_sample_scene(
          SampleScene::DecelerationPriorityScene);
      break;
  }

  pSamplePolySpeedAdjustDecider->set_count_normal_to_hover_state(
      sample_poly_input.sample_print_table_info()
          .count_normal_to_hover_state());

  pSamplePolySpeedAdjustDecider->set_count_hover_to_normal_state(
      sample_poly_input.sample_print_table_info()
          .count_hover_to_normal_state());

  pSamplePolySpeedAdjustDecider->set_distance_to_ramp(
      sample_poly_input.sample_print_table_info().dist_to_ramp());
  pSamplePolySpeedAdjustDecider->set_is_nearing_ramp(
      sample_poly_input.sample_print_table_info().is_nearing_ramp());
  pSamplePolySpeedAdjustDecider->set_is_in_merge_region(
      sample_poly_input.sample_print_table_info().is_in_merge_region());
  pSamplePolySpeedAdjustDecider->set_merge_stop_line_distance(
      sample_poly_input.sample_print_table_info().merge_emegency_distance());
  pSamplePolySpeedAdjustDecider->set_distance_to_road_merge(
      sample_poly_input.sample_print_table_info().distance_to_road_merge());
  pSamplePolySpeedAdjustDecider->set_distance_to_road_split(
      sample_poly_input.sample_print_table_info().distance_to_road_split());

  return 1;
}

int UpdateParam(double weight_follow_vel, double weight_match_gap_s,
                double weight_match_gap_vel, double weight_stop_line,
                double weight_leading_safe_s, double weight_leading_safe_v,
                double weight_vel_variable, double weight_gap_avaliable) {
  pSamplePolySpeedAdjustDecider->set_weight_follow_vel(weight_follow_vel);
  pSamplePolySpeedAdjustDecider->set_weight_match_gap_s(weight_match_gap_s);
  pSamplePolySpeedAdjustDecider->set_weight_match_gap_vel(weight_match_gap_vel);
  pSamplePolySpeedAdjustDecider->set_weight_stop_line(weight_stop_line);
  pSamplePolySpeedAdjustDecider->set_weight_leading_safe_s(
      weight_leading_safe_s);
  pSamplePolySpeedAdjustDecider->set_weight_leading_safe_v(
      weight_leading_safe_v);
  pSamplePolySpeedAdjustDecider->set_weight_vel_variable(weight_vel_variable);
  pSamplePolySpeedAdjustDecider->set_weight_gap_avaliable(weight_gap_avaliable);

  return 1;
}

std::vector<Eigen::Vector3d> GetMinCostTraj() {
  std::vector<Eigen::Vector3d> min_cost_points;
  auto* min_cost_traj_ptr = pSamplePolySpeedAdjustDecider->min_cost_traj_ptr();
  if (min_cost_traj_ptr != nullptr) {
    const double adjust_t_end = kPlanningDuration;
    for (double t = 0.0; t < adjust_t_end + 0.21; t += 0.2) {
      Eigen::Vector3d min_cost_point(t, min_cost_traj_ptr->CalcS(t),
                                     min_cost_traj_ptr->CalcV(t));
      min_cost_points.emplace_back(std::move(min_cost_point));
    }
  } else {
    std::cout << "min cost traj ptr! " << std::endl;
    return min_cost_points;
  }
  return min_cost_points;
}

std::vector<Eigen::Vector2d> GetMinCostAccJerk() {
  std::vector<Eigen::Vector2d> min_cost_acc_jerk;
  const auto* min_cost_traj_ptr =
      pSamplePolySpeedAdjustDecider->min_cost_traj_ptr();
  if (min_cost_traj_ptr != nullptr) {
    const double adjust_t_end = kPlanningDuration;
    for (double t = 0.0; t < adjust_t_end + 0.21; t += 0.2) {
      Eigen::Vector2d min_cost_acc_jerk_point(min_cost_traj_ptr->CalcAcc(t),
                                              min_cost_traj_ptr->CalcJerk(t));
      min_cost_acc_jerk.emplace_back(std::move(min_cost_acc_jerk_point));
    }
  } else {
    std::cout << "min cost traj ptr! " << std::endl;
    return min_cost_acc_jerk;
  }
  return min_cost_acc_jerk;
}

std::pair<size_t, size_t> get_min_cost_index() {
  return pSamplePolySpeedAdjustDecider->min_cost_traj_index();
}

py::bytes get_print_table_string() {
  planning::common::SamplePrintTableInfo sample_print_table_info;

  // 使用接口函数替代直接访问私有成员
  sample_print_table_info.set_dist_to_ramp(
      pSamplePolySpeedAdjustDecider->distance_to_ramp());
  sample_print_table_info.set_is_nearing_ramp(
      pSamplePolySpeedAdjustDecider->is_nearing_ramp());
  sample_print_table_info.set_is_in_merge_region(
      pSamplePolySpeedAdjustDecider->is_in_merge_region());

  sample_print_table_info.set_merge_emegency_distance(
      pSamplePolySpeedAdjustDecider->merge_stop_line_distance());
  sample_print_table_info.set_distance_to_road_merge(
      pSamplePolySpeedAdjustDecider->distance_to_road_merge());
  sample_print_table_info.set_distance_to_road_split(
      pSamplePolySpeedAdjustDecider->distance_to_road_split());

  sample_print_table_info.set_ego_s(pSamplePolySpeedAdjustDecider->ego_s());
  sample_print_table_info.set_ego_v(pSamplePolySpeedAdjustDecider->ego_v());
  sample_print_table_info.set_ego_acc(pSamplePolySpeedAdjustDecider->ego_a());
  sample_print_table_info.set_v_suggestted(
      pSamplePolySpeedAdjustDecider->v_suggestted());

  sample_print_table_info.set_flow_vel(
      pSamplePolySpeedAdjustDecider->target_lane_objs_flow_vel());
  sample_print_table_info.set_sample_status(
      static_cast<int32_t>(pSamplePolySpeedAdjustDecider->sample_status()));
  sample_print_table_info.set_sample_scene(
      static_cast<int32_t>(pSamplePolySpeedAdjustDecider->sample_scene()));
  sample_print_table_info.set_count_normal_to_hover_state(
      pSamplePolySpeedAdjustDecider->count_normal_to_hover_state());
  sample_print_table_info.set_count_hover_to_normal_state(
      pSamplePolySpeedAdjustDecider->count_hover_to_normal_state());
  sample_print_table_info.set_traffic_density(
      pSamplePolySpeedAdjustDecider->traffic_density());
  sample_print_table_info.set_traffic_density_status(
      pSamplePolySpeedAdjustDecider->traffic_density_status());

  // 使用接口函数访问 min_cost_traj_ptr_
  if (pSamplePolySpeedAdjustDecider->min_cost_traj_ptr() != nullptr) {
    auto* traj_ptr = pSamplePolySpeedAdjustDecider->min_cost_traj_ptr();
    sample_print_table_info.mutable_current_sample_match_gap_id()
        ->set_gap_front_id(traj_ptr->end_point_matched_gap_front_id());
    sample_print_table_info.mutable_current_sample_match_gap_id()
        ->set_gap_back_id(traj_ptr->end_point_matched_gap_back_id());

    sample_print_table_info.mutable_best_poly_cost_info()->set_follow_vel_cost(
        traj_ptr->follow_vel_cost().cost());
    sample_print_table_info.mutable_best_poly_cost_info()->set_stop_line_cost(
        traj_ptr->stop_line_cost().cost());
    sample_print_table_info.mutable_best_poly_cost_info()
        ->set_leading_veh_safe_cost(traj_ptr->leading_veh_safe_cost().cost());

    sample_print_table_info.mutable_best_poly_cost_info()
        ->set_vel_variable_cost(traj_ptr->speed_variable_cost().cost());
    sample_print_table_info.mutable_best_poly_cost_info()
        ->set_gap_avaliable_cost(traj_ptr->gap_avaliable_cost().cost());
    sample_print_table_info.mutable_best_poly_cost_info()->set_acc_limit_cost(
        traj_ptr->acc_limit_cost().cost());
    sample_print_table_info.mutable_best_poly_cost_info()->set_stop_line_cost(
        traj_ptr->stop_line_cost().cost());
    const auto& anchor_points_match_gap_vec =
        pSamplePolySpeedAdjustDecider->min_cost_traj_ptr()
            ->anchor_points_match_gap_cost_vec();
    double cost_anchor_points_match_gap_s_sum = 0.0;
    double cost_anchor_points_match_gap_v_sum = 0.0;
    double cost_anchor_points_match_gap_center_sum = 0.0;
    for (size_t i = 0; i < anchor_points_match_gap_vec.size(); i++) {
      cost_anchor_points_match_gap_s_sum +=
          anchor_points_match_gap_vec[i].match_s_cost();
      cost_anchor_points_match_gap_v_sum +=
          anchor_points_match_gap_vec[i].match_v_cost();
      cost_anchor_points_match_gap_center_sum +=
          anchor_points_match_gap_vec[i].match_gap_center_cost();
    }
    sample_print_table_info.mutable_best_poly_cost_info()
        ->set_match_gap_cost_s_sum(cost_anchor_points_match_gap_s_sum);
    sample_print_table_info.mutable_best_poly_cost_info()
        ->set_match_gap_cost_v_sum(cost_anchor_points_match_gap_v_sum);
    sample_print_table_info.mutable_best_poly_cost_info()
        ->set_match_gap_cost_center_sum(
            cost_anchor_points_match_gap_center_sum);
  }

  // // 使用接口函数访问 stitched_last_best_quartic_poly_ptr_
  // if (pSamplePolySpeedAdjustDecider->stitched_last_best_quartic_poly_ptr() !=
  //     nullptr) {
  //   auto stitched_ptr =
  //       pSamplePolySpeedAdjustDecider->stitched_last_best_quartic_poly_ptr();
  //   sample_print_table_info.mutable_stitched_sample_match_gap_id()
  //       ->set_gap_front_id(stitched_ptr->end_point_matched_gap_front_id());
  //   sample_print_table_info.mutable_stitched_sample_match_gap_id()
  //       ->set_gap_back_id(stitched_ptr->end_point_matched_gap_back_id());
  //   sample_print_table_info.mutable_stitched_poly_cost_info()
  //       ->set_follow_vel_cost(stitched_ptr->follow_vel_cost().cost());
  //   sample_print_table_info.mutable_stitched_poly_cost_info()
  //       ->set_stop_line_cost(stitched_ptr->stop_line_cost().cost());
  //   sample_print_table_info.mutable_stitched_poly_cost_info()
  //       ->set_leading_veh_safe_cost(
  //           stitched_ptr->leading_veh_safe_cost().cost());
  //   sample_print_table_info.mutable_stitched_poly_cost_info()
  //       ->set_vel_variable_cost(stitched_ptr->speed_variable_cost().cost());
  //   sample_print_table_info.mutable_stitched_poly_cost_info()
  //       ->set_gap_avaliable_cost(stitched_ptr->gap_avaliable_cost().cost());
  // }

  std::string serialized_print_table_info;
  sample_print_table_info.SerializeToString(&serialized_print_table_info);
  return serialized_print_table_info;
}
// --------------- Get Info for Plot
PYBIND11_MODULE(sample_poly_speed_adjust_decider_py, m) {
  m.doc() = "m";
  m.def("Init", &Init)
      .def("Execute", &Execute)
      .def("ProcessInput", &ProcessInput)
      .def("GetMinCostTraj", &GetMinCostTraj)
      .def("UpdateParam", UpdateParam)
      .def("GetMinCostAccJerk", GetMinCostAccJerk)
      .def("get_print_table_string", &get_print_table_string)
      .def("get_min_cost_index", &get_min_cost_index);
}