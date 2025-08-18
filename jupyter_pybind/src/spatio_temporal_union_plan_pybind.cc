#include <pybind11/attr.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "basic_types.pb.h"
#include "build/proto/spatio_temporal_union_dp_input.pb.h"
#include "environmental_model.h"
#include "planning_debug_info.pb.h"
#include "ros/init.h"
#include "src/common/vec2d.h"
#include "src/modules/common/config/basic_type.h"
#include "src/modules/tasks/behavior_planners/spatio_temporal_planner/spatio_temporal_union_dp.h"
#include "src/modules/tasks/behavior_planners/spatio_temporal_planner/grid_map.h"
#include "trajectory1d/trajectory1d.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"

#include "serialize_utils.h"
#include "reference_path.h"

namespace py = pybind11;
using namespace planning;
using namespace planning::planning_math;
static SpatioTemporalUnionDp *pBase = nullptr;

int Init() {
  FilePath::SetName("spatio_temporal_union_planning_py");
  InitGlog(FilePath::GetName().c_str());

  pBase = new SpatioTemporalUnionDp();
  return 0;
}

int UpdateByParams(py::bytes &spatio_temporal_union_input_bytes, double unit_t,
                   double dense_unit_s, double sparse_unit_s, double unit_l,
                   double dense_dimension_s, double dimension_l,
                   double max_acceleration, double max_deceleration,
                   double path_l_cost_param_l0, double path_l_cost_param_b, double path_l_cost_param_k,
                   double path_l_cost, double path_dl_cost, double path_ddl_cost, double path_end_l_cost,
                   double path_l_stitching_cost_param, double stitching_cost_time_decay_factor,
                   double obstacle_ignore_distance,
                   double obstacle_collision_cost,
                   double obstacle_longit_collision_distance,
                   double obstacle_longit_risk_distance,
                   double dynamic_obstacle_weight,
                   double default_obstacle_cost_weight,
                   double obstacle_lateral_risk_distance,
                   double obstacle_lateral_collision_distance,
                   double obstacle_collision_cost_without_lateral_overlap,
                   double spatial_potential_penalty,
                   double keep_clear_low_speed_penalty, double default_speed_cost,
                   double exceed_speed_penalty, double low_speed_penalty,
                   double reference_speed_penalty,
                   double accel_penalty, double decel_penalty,
                   double positive_jerk_coeff, double negative_jerk_coeff) {
  planning::common::SpationTemporalUnionDpInput spatio_temporal_union_input =
  BytesToProto<planning::common::SpationTemporalUnionDpInput> (
  spatio_temporal_union_input_bytes);
    planning::common::SpationTemporalUnionDpInput origin_planning_input = spatio_temporal_union_input;

  spatio_temporal_union_input.mutable_dp_search_paramms()->set_unit_t(unit_t);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_dense_unit_s(dense_unit_s);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_sparse_unit_s(sparse_unit_s);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_unit_l(unit_l);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_dense_dimension_s(dense_dimension_s);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_dimension_l(dimension_l);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_max_acceleration(max_acceleration);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_max_deceleration(max_deceleration);
  spatio_temporal_union_input.mutable_dp_search_paramms()->set_enable_use_parallel_calculate_cost(false);

  spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost_param_l0(path_l_cost_param_l0);
  spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost_param_b(path_l_cost_param_b);
  spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost_param_k(path_l_cost_param_k);
  spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost(path_l_cost);
  spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_dl_cost(path_dl_cost);
  spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_ddl_cost(path_ddl_cost);
  spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_end_l_cost(path_end_l_cost);

  spatio_temporal_union_input.mutable_stitching_cost_params()->set_path_l_stitching_cost_param(path_l_stitching_cost_param);
  spatio_temporal_union_input.mutable_stitching_cost_params()->set_stitching_cost_time_decay_factor(stitching_cost_time_decay_factor);

  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_ignore_distance(obstacle_ignore_distance);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_collision_cost(obstacle_collision_cost);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_longit_collision_distance(obstacle_longit_collision_distance);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_longit_risk_distance(obstacle_longit_risk_distance);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_dynamic_obstacle_weight(dynamic_obstacle_weight);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_default_obstacle_cost_weight(default_obstacle_cost_weight);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_lateral_risk_distance(obstacle_lateral_risk_distance);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_lateral_collision_distance(obstacle_lateral_collision_distance);
  spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_collision_cost_without_lateral_overlap(obstacle_collision_cost_without_lateral_overlap);

  spatio_temporal_union_input.mutable_long_weight_params()->set_spatial_potential_penalty(spatial_potential_penalty);
  spatio_temporal_union_input.mutable_long_weight_params()->set_keep_clear_low_speed_penalty(keep_clear_low_speed_penalty);
  spatio_temporal_union_input.mutable_long_weight_params()->set_default_speed_cost(default_speed_cost);
  spatio_temporal_union_input.mutable_long_weight_params()->set_exceed_speed_penalty(exceed_speed_penalty);
  spatio_temporal_union_input.mutable_long_weight_params()->set_low_speed_penalty(low_speed_penalty);
  spatio_temporal_union_input.mutable_long_weight_params()->set_reference_speed_penalty(reference_speed_penalty);
  spatio_temporal_union_input.mutable_long_weight_params()->set_accel_penalty(accel_penalty);
  spatio_temporal_union_input.mutable_long_weight_params()->set_decel_penalty(decel_penalty);
  spatio_temporal_union_input.mutable_long_weight_params()->set_positive_jerk_coeff(positive_jerk_coeff);
  spatio_temporal_union_input.mutable_long_weight_params()->set_negative_jerk_coeff(negative_jerk_coeff);
  // std::cout << "111 \n" << std::endl;
  // ILOG_INFO << "\n path_l_stitching_cost_param: " << path_l_stitching_cost_param;
  // ILOG_INFO << "\n stitching_cost_time_decay_factor: " << stitching_cost_time_decay_factor;

  std::vector<planning::planning_math::PathPoint> path_points;
  auto ref_points_vec = spatio_temporal_union_input.ref_points_vec();
  for (int c = 0; c < ref_points_vec.size(); c++) {
    planning::planning_math::PathPoint temp_path_point;
    temp_path_point.set_x(ref_points_vec[c].x());
    temp_path_point.set_y(ref_points_vec[c].y());
    // ILOG_INFO << "\n temp_path_point.x " << ref_points_vec[c].x()
    //           << "  temp_path_point.y " << ref_points_vec[c].y();
    path_points.emplace_back(temp_path_point);
  }
  // std::cout << "path_points: \n" << path_points.size() << std::endl;
  if (path_points.size() < 3) {
    return 0;
  }
  planning_math::KDPath target_lane_coord(std::move(path_points));
  // ILOG_INFO << "\n 333 " ;

  const double target_s = 200.0;
  const double default_points_num = 26;
  const double delta_time = 0.2;
  planning::TrajectoryPoints traj_points;
  traj_points.resize(default_points_num);
  for (size_t t = 0; t < default_points_num; ++t) {
    traj_points[t].t = delta_time * t;
  }
  // ILOG_INFO << "\n 44 " ;
  // std::cout << "222 \n" << std::endl;
  const int num_agent = 8;
  std::vector<AgentFrenetSpatioTemporalInFo> surround_agents_trajs;
  auto agents_time_and_corners = spatio_temporal_union_input.agent_time_corners();
  // ILOG_INFO << "\n agents_time_and_corners: " << agents_time_and_corners.size() ;

  surround_agents_trajs.clear();
  for (size_t i = 0; i < agents_time_and_corners.size(); ++i) {
    // std::vector<AgentFrenetSpatioTemporalInFo> sur_trajs;
    // std::cout << "i \n" << i << std::endl;
    const auto& agent_iter = agents_time_and_corners[i];
    int agent_id = agent_iter.agent_id();
    int agent_type = agent_iter.agent_type();
    AgentFrenetSpatioTemporalInFo agent_state;
    agent_state.agent_id = agent_id;
    agent_state.agent_type = agent_type;

    const auto& times_and_corners = agent_iter.time_and_corners();
    const auto& max_box_corners = agent_iter.max_box_corners();
    for (int j = 0; j < times_and_corners.size(); ++j) {
      if (!times_and_corners[j].enable_use()) {
        continue;
      }

      const auto& agent_corners = times_and_corners[j].agent_corner();
      // std::vector<Vec2d> box_corners;
      std::array<planning_math::Vec2d, 8> box_corners;
      for (int k = 0; k < agent_corners.size(); ++k) {
        Vec2d corner;
        corner.set_x(agent_corners[k].x());
        corner.set_y(agent_corners[k].y());
        box_corners[k] = corner;
        // ILOG_INFO << "\n time " << j * delta_time
        //           << "  k = " << k
        //           << "  corner.x = " << corner.x()
        //           << "  corner.y = " << corner.y();
      }
      if (!box_corners.empty()) {
        AABox2d agent_box(box_corners, num_agent);
        agent_state.agent_boxs_set.insert(std::make_pair(j, agent_box));
      }
    }
    std::vector<Vec2d> max_agent_box_corners;
    for (int c = 0; c < max_box_corners.size(); ++c) {
      Vec2d corner;
      corner.set_x(max_box_corners[c].x());
      corner.set_y(max_box_corners[c].y());
      // ILOG_INFO << "  max_box_corners.x = " << corner.x()
      //           << "  max_box_corners.y = " << corner.y();
      max_agent_box_corners.emplace_back(corner);
    }
    if (!max_agent_box_corners.empty()) {
    AABox2d max_agent_box(max_agent_box_corners);
    agent_state.max_agent_box = max_agent_box;
    }
    surround_agents_trajs.emplace_back(agent_state);
  }

  const int half_lateral_sample_nums = 4;
  bool last_enable_using_st_plan = true;

  // ILOG_INFO << " target_lane_coord.min_x " << target_lane_coord.min_x()
  //           << " target_lane_coord.min_y " << target_lane_coord.min_y()
  //           << " target_lane_coord.max_x " << target_lane_coord.max_x()
  //           << " target_lane_coord.max_y" << target_lane_coord.max_y();
  pBase->Update(
      traj_points, surround_agents_trajs, spatio_temporal_union_input, target_s, target_lane_coord, half_lateral_sample_nums, last_enable_using_st_plan);
  return 0;
}

// int UpdateByParams2(py::bytes &road_msg_input_bytes, double unit_t,
//                     double dense_unit_s, double sparse_unit_s, double unit_l,
//                     double dense_dimension_s, double dimension_l,
//                     double max_acceleration, double max_deceleration,
//                     double path_l_cost_param_l0, double path_l_cost_param_b, double path_l_cost_param_k,
//                     double path_l_cost, double path_dl_cost, double path_ddl_cost, double path_end_l_cost,
//                     double obstacle_ignore_distance,
//                     double obstacle_collision_cost,
//                     double obstacle_collision_distance,
//                     double obstacle_risk_distance,
//                     double spatial_potential_penalty,
//                     double keep_clear_low_speed_penalty, double default_speed_cost,
//                     double exceed_speed_penalty, double low_speed_penalty,
//                     double reference_speed_penalty,
//                     double accel_penalty, double decel_penalty,
//                     double positive_jerk_coeff, double negative_jerk_coeff) {
//   iflyauto::RoadInfo road_info =
//       BytesToStruct<iflyauto::RoadInfo, struct_msgs::RoadInfo>(
//           road_msg_input_bytes);

//   planning::common::SpationTemporalUnionDpInput spatio_temporal_union_input;

//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_unit_t(unit_t);
//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_dense_unit_s(dense_unit_s);
//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_sparse_unit_s(sparse_unit_s);
//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_unit_l(unit_l);
//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_dense_dimension_s(dense_dimension_s);
//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_dimension_l(dimension_l);
//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_max_acceleration(max_acceleration);
//   spatio_temporal_union_input.mutable_dp_search_paramms()->set_max_deceleration(max_deceleration);

//   spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost_param_l0(path_l_cost_param_l0);
//   spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost_param_b(path_l_cost_param_b);
//   spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost_param_k(path_l_cost_param_k);
//   spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_l_cost(path_l_cost);
//   spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_dl_cost(path_dl_cost);
//   spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_ddl_cost(path_ddl_cost);
//   spatio_temporal_union_input.mutable_lat_path_weight_params()->set_path_end_l_cost(path_end_l_cost);

//   spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_ignore_distance(obstacle_ignore_distance);
//   spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_collision_cost(obstacle_collision_cost);
//   spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_collision_distance(obstacle_collision_distance);
//   spatio_temporal_union_input.mutable_dp_dynamic_agent_weight_params()->set_obstacle_risk_distance(obstacle_risk_distance);

//   spatio_temporal_union_input.mutable_long_weight_params()->set_spatial_potential_penalty(spatial_potential_penalty);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_keep_clear_low_speed_penalty(keep_clear_low_speed_penalty);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_default_speed_cost(default_speed_cost);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_exceed_speed_penalty(exceed_speed_penalty);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_low_speed_penalty(low_speed_penalty);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_reference_speed_penalty(reference_speed_penalty);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_accel_penalty(accel_penalty);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_decel_penalty(decel_penalty);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_positive_jerk_coeff(positive_jerk_coeff);
//   spatio_temporal_union_input.mutable_long_weight_params()->set_negative_jerk_coeff(negative_jerk_coeff);

//   std::vector<planning::planning_math::PathPoint> path_points;
//   auto ref_points_vec = spatio_temporal_union_input.ref_points_vec();
//   for (int c = 0; c < ref_points_vec.size(); c++) {
//     planning::planning_math::PathPoint temp_path_point;
//     temp_path_point.set_x(ref_points_vec[c].x());
//     temp_path_point.set_y(ref_points_vec[c].y());
//     path_points.emplace_back(temp_path_point);
//   }

//   planning_math::KDPath target_lane_coord(std::move(path_points), true);

//   const double target_s = 200.0;
//   const double default_points_num = 26;
//   const double delta_time = 0.2;
//   planning::TrajectoryPoints traj_points;
//   traj_points.resize(default_points_num);
//   for (size_t t = 0; t < default_points_num; ++t) {
//     traj_points[t].t = delta_time * t;

//   }
//   traj_points.resize(default_points_num);

//   std::vector<AgentFrenetSpatioTemporalInFo> surround_agents_trajs;
//   auto agents_time_and_corners = spatio_temporal_union_input.agent_time_corners();
//   surround_agents_trajs.clear();
//   for (size_t i = 0; i < agents_time_and_corners.size(); ++i) {
//     // std::vector<AgentFrenetSpatioTemporalInFo> sur_trajs;
//     const auto& agent_iter = agents_time_and_corners[i];
//     int agent_id = agent_iter.agent_id();
//     AgentFrenetSpatioTemporalInFo agent_state;
//     agent_state.agent_id = agent_id;

//     int offset = 0;
//     const auto& times_and_corners = agent_iter.time_and_corners();
//     for (size_t j = 0; j < times_and_corners.size(); ++j) {
//       const auto& agent_corners = times_and_corners[j].agent_corner();
//       std::vector<Vec2d> box_corners;
//       for (size_t k = 0; k < agent_corners.size(); ++k) {
//         Vec2d corner;
//         corner.set_x(agent_corners[k].x());
//         corner.set_y(agent_corners[k].y());
//         box_corners.emplace_back(corner);
//       }
//       AABox2d agent_box(box_corners);
//       agent_state.agent_time_box.insert(std::make_pair(times_and_corners[j].time(), agent_box));
//     }
//     surround_agents_trajs.emplace_back(agent_state);
//   }


//   pBase->Update(traj_points, surround_agents_trajs, spatio_temporal_union_input, target_s, target_lane_coord);
//   return 0;
// }

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

PYBIND11_MODULE(spatio_temporal_union_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetOutputBytes", &GetOutputBytes);
}