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

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>

#include "basic_types.pb.h"
#include "build/proto/spatio_temporal_union_dp_input.pb.h"
#include "environmental_model.h"
#include "planning_debug_info.pb.h"
#include "ros/init.h"
#include "src/common/vec2d.h"
#include "src/modules/common/config/basic_type.h"
#include "src/modules/tasks/behavior_planners/spatio_temporal_planner/spatio_temporal_union_dp.h"
#include "spatio_temporal_union_plan.pb.h"
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
static planning_math::KDPath *pTargetLaneCoord = nullptr;
static planning::common::SpatioTemporalUnionPlan g_plan_output;

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
  if (path_points.size() < planning_math::KDPath::kKDPathMinPathPointSize + 1) {
    return 0;
  }
  planning_math::KDPath target_lane_coord(std::move(path_points));
  // Save for GetSamplePointsXY
  if (pTargetLaneCoord != nullptr) {
    delete pTargetLaneCoord;
  }
  pTargetLaneCoord = new planning_math::KDPath(target_lane_coord);
  // ILOG_INFO << "\n 333 " ;

  double target_s = spatio_temporal_union_input.target_s();
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
    agent_state.is_static = agent_iter.is_static();

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
      traj_points, surround_agents_trajs, spatio_temporal_union_input, target_s, target_lane_coord, half_lateral_sample_nums, last_enable_using_st_plan, &g_plan_output);
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

py::bytes GetDebugPlanBytes() {
  const auto& debug_points = pBase->GetDebugPoints();
  std::cout << "debug_points size:" << debug_points.size() << std::endl;
  std::string serialized_message;
  for (const auto& point : debug_points) {
    std::string msg;
    point.SerializeToString(&msg);
    google::protobuf::io::StringOutputStream sos(&serialized_message);
    google::protobuf::io::CodedOutputStream cos(&sos);
    cos.WriteVarint32(msg.size());
    cos.WriteRaw(msg.data(), msg.size());
  }
  return serialized_message;
}

py::dict GetSamplePointsXY() {
  const auto& cost_table = pBase->GetCostTable();
  std::vector<double> x_coords, y_coords;
  std::vector<double> s_vals, l_vals, t_vals, total_costs;
  std::vector<int> t_indices, s_indices, l_indices;

  if (pTargetLaneCoord == nullptr) {
    py::dict result;
    result["x"] = x_coords; result["y"] = y_coords;
    result["s"] = s_vals; result["l"] = l_vals; result["t"] = t_vals;
    result["total_cost"] = total_costs;
    result["t_index"] = t_indices; result["s_index"] = s_indices; result["l_index"] = l_indices;
    return result;
  }

  // Only iterate t=0 layer to avoid duplicate spatial positions across time
  if (!cost_table.empty()) {
    for (const auto& s_layer : cost_table[0]) {
      for (const auto& point : s_layer) {
        const auto& slt_point = point.point();
        Point2D frenet_point(slt_point.s(), slt_point.l());
        Point2D cart_point;
        if (pTargetLaneCoord->SLToXY(frenet_point, cart_point)) {
          x_coords.push_back(cart_point.x);
          y_coords.push_back(cart_point.y);
          s_vals.push_back(slt_point.s());
          l_vals.push_back(slt_point.l());
          t_vals.push_back(slt_point.t());
          total_costs.push_back(point.total_cost());
          t_indices.push_back(static_cast<int>(point.index_t()));
          s_indices.push_back(static_cast<int>(point.index_s()));
          l_indices.push_back(static_cast<int>(point.index_l()));
        }
      }
    }
  }

  py::dict result;
  result["x"] = x_coords; result["y"] = y_coords;
  result["s"] = s_vals; result["l"] = l_vals; result["t"] = t_vals;
  result["total_cost"] = total_costs;
  result["t_index"] = t_indices; result["s_index"] = s_indices; result["l_index"] = l_indices;
  return result;
}

py::dict GetDebugPathXY() {
  const auto& debug_points = pBase->GetDebugPoints();
  std::vector<double> x_coords, y_coords;

  if (pTargetLaneCoord == nullptr) {
    py::dict result;
    result["x"] = x_coords;
    result["y"] = y_coords;
    return result;
  }

  for (const auto& point : debug_points) {
    Point2D frenet_point(point.s(), point.l());
    Point2D cart_point;
    if (pTargetLaneCoord->SLToXY(frenet_point, cart_point)) {
      x_coords.push_back(cart_point.x);
      y_coords.push_back(cart_point.y);
    }
  }

  py::dict result;
  result["x"] = x_coords;
  result["y"] = y_coords;
  return result;
}

// Return all visited nodes in cost_table_ with costs and predecessor indices.
// Useful for visualizing the full DP cost landscape (not just the optimal path).
py::dict GetAllNodesXY() {
  const auto& cost_table = pBase->GetCostTable();
  std::vector<double> x_coords, y_coords;
  std::vector<double> s_vals, l_vals, t_vals;
  std::vector<double> total_costs, obstacle_costs, path_costs;
  std::vector<double> path_l_costs, path_dl_costs, path_ddl_costs;
  std::vector<double> stitching_costs, long_costs;
  std::vector<double> speeds, accs;
  std::vector<int> t_indices, s_indices, l_indices;
  std::vector<int> pre_t_indices, pre_s_indices, pre_l_indices;

  if (pTargetLaneCoord == nullptr) {
    py::dict result;
    result["x"] = x_coords; result["y"] = y_coords;
    result["s"] = s_vals; result["l"] = l_vals; result["t"] = t_vals;
    result["total_cost"] = total_costs; result["obstacle_cost"] = obstacle_costs;
    result["path_cost"] = path_costs; result["path_l_cost"] = path_l_costs;
    result["path_dl_cost"] = path_dl_costs; result["path_ddl_cost"] = path_ddl_costs;
    result["stitching_cost"] = stitching_costs; result["long_cost"] = long_costs;
    result["speed"] = speeds; result["acc"] = accs;
    result["t_index"] = t_indices; result["s_index"] = s_indices; result["l_index"] = l_indices;
    result["pre_t_index"] = pre_t_indices; result["pre_s_index"] = pre_s_indices;
    result["pre_l_index"] = pre_l_indices;
    return result;
  }

  for (size_t t = 0; t < cost_table.size(); ++t) {
    for (size_t s = 0; s < cost_table[t].size(); ++s) {
      for (size_t l = 0; l < cost_table[t][s].size(); ++l) {
        const auto& point = cost_table[t][s][l];
        if (std::isinf(point.total_cost())) continue;

        Point2D frenet_point(point.point().s(), point.point().l());
        Point2D cart_point;
        if (!pTargetLaneCoord->SLToXY(frenet_point, cart_point)) continue;

        x_coords.push_back(cart_point.x);
        y_coords.push_back(cart_point.y);
        s_vals.push_back(point.point().s());
        l_vals.push_back(point.point().l());
        t_vals.push_back(point.point().t());
        total_costs.push_back(point.total_cost());
        obstacle_costs.push_back(point.obstacle_cost());
        path_costs.push_back(point.path_cost());
        path_l_costs.push_back(point.path_l_cost());
        path_dl_costs.push_back(point.path_dl_cost());
        path_ddl_costs.push_back(point.path_ddl_cost());
        stitching_costs.push_back(point.stitching_cost());
        long_costs.push_back(point.longitinal_cost());
        speeds.push_back(point.GetOptimalSpeed());
        accs.push_back(point.GetAcc());
        t_indices.push_back(static_cast<int>(t));
        s_indices.push_back(static_cast<int>(s));
        l_indices.push_back(static_cast<int>(l));

        if (point.pre_point() != nullptr) {
          pre_t_indices.push_back(static_cast<int>(point.pre_point()->index_t()));
          pre_s_indices.push_back(static_cast<int>(point.pre_point()->index_s()));
          pre_l_indices.push_back(static_cast<int>(point.pre_point()->index_l()));
        } else {
          pre_t_indices.push_back(-1);
          pre_s_indices.push_back(-1);
          pre_l_indices.push_back(-1);
        }
      }
    }
  }

  py::dict result;
  result["x"] = x_coords; result["y"] = y_coords;
  result["s"] = s_vals; result["l"] = l_vals; result["t"] = t_vals;
  result["total_cost"] = total_costs; result["obstacle_cost"] = obstacle_costs;
  result["path_cost"] = path_costs; result["path_l_cost"] = path_l_costs;
  result["path_dl_cost"] = path_dl_costs; result["path_ddl_cost"] = path_ddl_costs;
  result["stitching_cost"] = stitching_costs; result["long_cost"] = long_costs;
  result["speed"] = speeds; result["acc"] = accs;
  result["t_index"] = t_indices; result["s_index"] = s_indices; result["l_index"] = l_indices;
  result["pre_t_index"] = pre_t_indices; result["pre_s_index"] = pre_s_indices;
  result["pre_l_index"] = pre_l_indices;
  return result;
}

// Set last frame trajectory from bag data so stitching cost uses correct previous frame.
// Accepts serialized basic_types_pb2.TrajectoryPoints bytes.
int SetLastFrameTrajectory(py::bytes& traj_bytes) {
  auto traj_points_proto = BytesToProto<planning::common::TrajectoryPoints>(traj_bytes);

  std::vector<planning::planning_math::PathPoint> path_points;
  path_points.reserve(traj_points_proto.point_size());
  for (int i = 0; i < traj_points_proto.point_size(); ++i) {
    const auto& pt = traj_points_proto.point(i);
    planning::planning_math::PathPoint path_pt;
    path_pt.set_x(pt.x());
    path_pt.set_y(pt.y());
    path_points.emplace_back(path_pt);
  }

  if (path_points.size() < planning_math::KDPath::kKDPathMinPathPointSize + 1) {
    return 0;
  }

  auto kd_path = std::make_shared<planning_math::KDPath>(std::move(path_points));
  pBase->SetLastFrameTrajectory(kd_path);
  return 0;
}

PYBIND11_MODULE(spatio_temporal_union_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetOutputBytes", &GetOutputBytes)
      .def("GetDebugPlanBytes", &GetDebugPlanBytes)
      .def("GetSamplePointsXY", &GetSamplePointsXY)
      .def("GetDebugPathXY", &GetDebugPathXY)
      .def("GetAllNodesXY", &GetAllNodesXY)
      .def("SetLastFrameTrajectory", &SetLastFrameTrajectory);
}