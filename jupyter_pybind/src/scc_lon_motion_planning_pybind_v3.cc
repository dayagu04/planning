#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>
#include <vector>

#include "longitudinal_motion_planner.pb.h"
#include "planning_debug_info.pb.h"
#include "tasks/behavior_planners/long_ref_path_decider/target_marker/target.h"
#include "tasks/motion_planners/scc_lon_motion_planner_v3/src/scc_longitudinal_motion_planning_problem_v3.h"

namespace py = pybind11;
using namespace pnc::scc_longitudinal_planning_v3;

static SccLongitudinalMotionPlanningProblemV3 *pBase = nullptr;
constexpr double plan_points_num = 26;
constexpr double plan_time = 5.0;
constexpr double dt = 0.2;
constexpr double max_s_weight_time = 3.0;
constexpr double front_lower_weight = 0.5;
constexpr double max_s_weight = 10.0;
constexpr double back_upper_weight = 1.5;
constexpr double kUrgentWeightStartTime = 0.0;
constexpr double kAddUrgentWeightEndTime = 1.5;
constexpr double s_speed_upper_weight_v = 8.33;
constexpr double s_speed_lower_weight_v = 2.78;
constexpr double s_speed_upper_weight = 5.0;
constexpr double s_speed_lower_weight = 2.5;

int Init() {
  pBase = new SccLongitudinalMotionPlanningProblemV3();
  pBase->Init();
  return 0;
}

template <class T>
inline T BytesToProto(py::bytes &bytes) {
  T proto_obj;
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char *input_ptr = static_cast<char *>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

int UpdateBytes(py::bytes &planning_input_bytes) {
  planning::common::LongitudinalPlanningInput planning_input =
      BytesToProto<planning::common::LongitudinalPlanningInput>(
          planning_input_bytes);

  pBase->Update(planning_input);

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

std::vector<double> MakeSWeight(double default_s_weight, double follow_s_weight,
                                double overtake_s_weight,
                                double neighbor_s_weight,
                                std::vector<int32_t> target_type_vec,
                                bool is_urgent, double lon_state_v,
                                double urgent_scale) {
  auto s_weights = std::vector<double>(plan_points_num, default_s_weight);
  for (size_t i = 0; i < plan_points_num; ++i) {
    double relative_t = i * dt;
    auto type = static_cast<TargetType>(target_type_vec[i]);
    if (type == TargetType::kCruiseSpeed) {
      s_weights[i] = default_s_weight;
    } else if (type == TargetType::kFollow) {
      s_weights[i] = follow_s_weight;
    } else if (type == TargetType::kOvertake) {
      s_weights[i] = overtake_s_weight;
    } else if (type == TargetType::kNeighbor ||
               type == TargetType::kNeighborYeild ||
               type == TargetType::kNeighborOvertake) {
      s_weights[i] = neighbor_s_weight;
    }

    const double speed_scale = planning_math::LerpWithLimit(
        s_speed_lower_weight, s_speed_lower_weight_v, s_speed_upper_weight,
        s_speed_upper_weight_v, lon_state_v);

    const double mid_time =
        std::fmin(std::fmax(0.0, max_s_weight_time), plan_time);
    if (relative_t <= mid_time) {
      const double front_time_scale = planning_math::LerpWithLimit(
          front_lower_weight, 0.0, max_s_weight, mid_time, relative_t);
      s_weights[i] = s_weights[i] * front_time_scale;
    } else {
      const double back_time_scale = planning_math::LerpWithLimit(
          max_s_weight, mid_time, back_upper_weight, plan_time, relative_t);
      s_weights[i] = s_weights[i] * back_time_scale;
    }
    s_weights[i] = s_weights[i] * speed_scale;
    if (is_urgent && relative_t >= kUrgentWeightStartTime &&
        relative_t <= kAddUrgentWeightEndTime) {
      s_weights[i] = s_weights[i] * urgent_scale;
    }
  }
  return s_weights;
}

std::vector<double> MakeVWeight(double default_v_weight, double cruise_v_weight,
                                std::vector<int32_t> target_type_vec) {
  auto v_weights = std::vector<double>(plan_points_num, default_v_weight);
  for (size_t i = 0; i < plan_points_num; ++i) {
    auto type = static_cast<TargetType>(target_type_vec[i]);
    if (type == TargetType::kCruiseSpeed) {
      v_weights[i] = cruise_v_weight;
    }
  }
  return v_weights;
}

int UpdateByParams(py::bytes &planning_input_bytes, double q_acc, double q_jerk,
                   double q_hard_pos_bound, double q_vel_bound,
                   double q_acc_bound, double q_jerk_bound, double q_stop_s,
                   double default_s_weight, double follow_s_weight,
                   double overtake_s_weight, double neighbor_s_weight,
                   std::vector<int32_t> target_type_vec, bool is_urgent,
                   double lon_state_v, double urgent_scale,
                   double default_v_weight, double cruise_v_weight, double const_s) {
  planning::common::LongitudinalPlanningInput planning_input =
      BytesToProto<planning::common::LongitudinalPlanningInput>(
          planning_input_bytes);
  planning_input.set_q_ref_pos(const_s);
  planning_input.set_q_acc(q_acc);
  planning_input.set_q_jerk(q_jerk);
  planning_input.set_q_hard_pos_bound(q_hard_pos_bound);
  planning_input.set_q_vel_bound(q_vel_bound);
  planning_input.set_q_acc_bound(q_acc_bound);
  planning_input.set_q_jerk_bound(q_jerk_bound);
  planning_input.set_q_stop_s(q_stop_s);

  auto s_weights = MakeSWeight(
      default_s_weight, follow_s_weight, overtake_s_weight, neighbor_s_weight,
      target_type_vec, is_urgent, lon_state_v, urgent_scale);

  auto v_weights =
      MakeVWeight(default_v_weight, cruise_v_weight, target_type_vec);

  for (size_t i = 0; i < s_weights.size(); ++i) {
    // planning_input.mutable_s_weights()->Set(i, const_s);
    planning_input.mutable_v_weights()->Set(i, v_weights[i]);
  }

  pBase->Update(planning_input);

  return 0;
}

PYBIND11_MODULE(scc_lon_motion_planning_v3_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &UpdateBytes)
      .def("UpdateByParams", &UpdateByParams)
      .def("GetOutputBytes", &GetOutputBytes);
}
