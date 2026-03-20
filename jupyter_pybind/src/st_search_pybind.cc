#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "basic_types.pb.h"
// #include "behavior_planners/speed_search_decider/st_graph_base.h"
#include "behavior_planners/speed_search_decider/new_st_point.h"
#include "behavior_planners/speed_search_decider/st_graph_base.h"
#include "behavior_planners/speed_search_decider/st_search_node.h"
#include "behavior_planners/speed_search_decider/st_search_path.h"
// #include "behavior_planners/speed_search_decider/st_search_utils.h"
#include "config/basic_type.h"
#include "environmental_model.h"
#include "log.h"
#include "new_st_point.h"
#include "planning_debug_info.pb.h"
#include "serialize_utils.h"
#include "st_boundary.h"
#include "st_search_decider.pb.h"
// #include "tasks/behavior_planners/speed_search_decider/st_search_decider.h"
#include "trajectory1d/trajectory1d.h"
// #include "utils/frenet_coordinate_system.h"
// #include "utils/kd_path.h"
// #include "utils/path_point.h"

namespace py = pybind11;
// using namespace planning::speed_decider;
// using namespace speed_decider;
using namespace planning;
using namespace speed_decider;
static StSearchPath *search_path = nullptr;
static STGraphBase *st_graph_base = nullptr;

static StSearchConfig *st_search_config = nullptr;
static CostConfig *cost_config = nullptr;
static std::pair<double, double> *init_sl =
    new std::pair<double, double>(0., 0.);
static std::pair<double, double> *init_va =
    new std::pair<double, double>(0., 0.);
static double *heuristic_s = new double(0.);

static Json json;

int Init() {
  search_path = new StSearchPath();
  st_graph_base = new STGraphBase();
  // st_graph_base = new STGraphBase();

  st_search_config = new StSearchConfig();
  cost_config = new CostConfig();

  // st_graph_base->ego_sl_.first = 0.0;
  // st_graph_base->ego_sl_.second = 0.0;
  st_graph_base->time_range_.first = 0.0;
  st_graph_base->time_range_.second = kPlanningDuration;
  st_graph_base->path_range_.first = 0.0;
  st_graph_base->path_range_.second = 150.0;
  st_graph_base->reserve_num_ = (kPlanningDuration) / kTimeResolution + 1;

  // // // add default st boundary
  //  LaneChangeVehInfo default_agent_one;
  // default_agent_one.id = 11;
  // default_agent_one.center_s = 20.0;
  // default_agent_one.v = 20.0;
  // default_agent_one.half_length = 2.55;
  // st_graph_base->LinearExtendAgentStBoundary(
  //     default_agent_one,  StBoundaryType::NORMAL);
  //  LaneChangeVehInfo default_agent_two;
  // default_agent_two.id = 12;
  // default_agent_two.center_s = -15.0;
  // default_agent_two.v = 20.0;
  // default_agent_two.half_length = 2.55;
  // st_graph_base->LinearExtendAgentStBoundary(
  //     default_agent_two,  StBoundaryType::NORMAL);

  st_search_config->max_acc_limit = 4.0;
  st_search_config->min_acc_limit = -4.0;
  st_search_config->max_jerk_limit = 6.0;
  st_search_config->min_jerk_limit = -6.0;
  st_search_config->speed_limit_scale = 1.1;
  st_search_config->max_count = 5000;
  st_search_config->s_step = 0.2;
  st_search_config->t_step = 0.2;
  st_search_config->t_step_inverse = 1 / st_search_config->t_step;
  st_search_config->t_step_square =
      st_search_config->t_step * st_search_config->t_step;
  st_search_config->vel_step = 0.2;
  st_search_config->max_search_time = 30.0;
  st_search_config->use_stop_line = false;

  std::vector<double> acc_steps;
  for (auto acc = st_search_config->min_acc_limit;
       acc < st_search_config->max_acc_limit; acc += 1.0) {
    acc_steps.emplace_back(acc);
  }
  st_search_config->acc_step = acc_steps;
  cost_config->vel_weight = 0.0;
  cost_config->accel_weight = 0.2;
  cost_config->jerk_weight = 0.1;

  cost_config->hcost_s_weight = 5.0;
  cost_config->hcost_v_weight = 15.0;
  cost_config->comfort_gap_length = 15.0;
  cost_config->violation_weight = 0.0;
  cost_config->s_dis_weight = 0.4;

  return 0;
}

int UpdateBytes(py::bytes &st_search_decider_bytes) {
  st_graph_base->lane_change_veh_info_id_map_.clear();
  st_graph_base->lane_change_veh_vec_.clear();
  planning::common::StSearchDeciderInfo st_search_decider_info =
      BytesToProto<planning::common::StSearchDeciderInfo>(
          st_search_decider_bytes);

  // lane_change_veh_info_id_map
  const auto &lane_change_veh_info =
      st_search_decider_info.lane_change_veh_info();
  std::unordered_map<int32_t, LaneChangeVehInfo> lane_change_veh_info_id_map;
  std::vector<LaneChangeVehInfo> lane_change_veh_vec;
  std::cout << "proto input, lane_change_veh_info size: "
            << lane_change_veh_info.size() << std::endl;
  for (auto &lane_change_veh : lane_change_veh_info) {
    int32_t id = lane_change_veh.id();

    LaneChangeVehInfo veh_info;
    veh_info.id = id;
    veh_info.center_s = lane_change_veh.center_s();
    veh_info.half_length = lane_change_veh.half_length();
    veh_info.v = lane_change_veh.v();
    lane_change_veh_vec.emplace_back(veh_info);
    lane_change_veh_info_id_map.insert(
        std::make_pair(veh_info.id, std::move(veh_info)));
  }

  LaneChangeVehInfo leading_veh_info;
  if (st_search_decider_info.leading_veh_info().id() > -1) {
    leading_veh_info.half_length =
        st_search_decider_info.leading_veh_info().half_length();
    leading_veh_info.center_s =
        st_search_decider_info.leading_veh_info().center_s();
    leading_veh_info.v = st_search_decider_info.leading_veh_info().v();
    leading_veh_info.id = st_search_decider_info.leading_veh_info().id();
  }

  init_sl->first = st_search_decider_info.ego_s();
  init_sl->second = st_search_decider_info.ego_l();
  init_va->first = st_search_decider_info.ego_v();
  init_va->second = st_search_decider_info.ego_a();

  std::cout << "lane_change_veh_info size: " << lane_change_veh_vec.size()
            << std::endl;
  std::cout << "leading veh info id: " << leading_veh_info.id << std::endl;
  // LOG_ERROR("lane_change_veh_info size: ", lane_change_veh_vec.size());
  st_graph_base->Init(leading_veh_info, lane_change_veh_vec,
                      lane_change_veh_info_id_map, *init_sl);
}

int UpdateParamsNew(double max_acc_limit, double min_acc_limit,
                    double max_jerk_limit, double min_jerk_limit,
                    double vel_weight, double accel_weight, double jerk_weight,
                    double violation_weight, double s_dis_weight,
                    double hcost_s_weight, double hcost_v_weight) {
  st_search_config->max_acc_limit = max_acc_limit;
  st_search_config->min_acc_limit = min_acc_limit;
  st_search_config->max_jerk_limit = max_jerk_limit;
  st_search_config->min_jerk_limit = min_jerk_limit;
  cost_config->vel_weight = vel_weight;
  cost_config->accel_weight = accel_weight;
  cost_config->jerk_weight = jerk_weight;
  cost_config->violation_weight = violation_weight;
  cost_config->s_dis_weight = s_dis_weight;
  cost_config->hcost_s_weight = hcost_s_weight;
  cost_config->hcost_v_weight = hcost_v_weight;

  return 1;
}

int UpdateParams(double max_acc_limit, double min_acc_limit,
                 double max_jerk_limit, double min_jerk_limit,
                 double speed_limit, double speed_limit_scale, double v_cruise,
                 double v_min, double collision_ttc, double min_collision_dist,
                 double max_collision_dist, double s_step, double t_step,
                 double vel_step, double acc_search_step,
                 double max_search_time, double acc_search_max,
                 double acc_search_min, double vel_tolerance,
                 double propoper_accel_value, double planning_time_horizon,
                 double rel_ego_stop_s, bool use_stop_line, int max_count) {
  st_search_config->max_acc_limit = max_acc_limit;
  st_search_config->min_acc_limit = min_acc_limit;
  st_search_config->max_jerk_limit = max_jerk_limit;
  st_search_config->min_jerk_limit = min_jerk_limit;
  st_search_config->speed_limit = v_cruise * speed_limit_scale;
  st_search_config->speed_limit_scale = speed_limit_scale;
  // st_search_config->speed_limit_inverse = 1 / speed_limit;
  st_search_config->v_cruise = v_cruise;
  // st_search_config->collision_ttc = collision_ttc;
  // st_search_config->min_collision_dist = min_collision_dist;
  // st_search_config->max_collision_dist = max_collision_dist;
  st_search_config->v_min = v_min;
  st_search_config->max_count = max_count;
  st_search_config->s_step = s_step;
  st_search_config->t_step = t_step;
  st_search_config->t_step_inverse = 1 / t_step;
  st_search_config->t_step_square = t_step * t_step;
  st_search_config->vel_step = vel_step;
  st_search_config->max_search_time = max_search_time;
  st_search_config->rel_ego_stop_s = rel_ego_stop_s;
  st_search_config->stop_line_s_inverse = 1 / rel_ego_stop_s;
  st_search_config->use_stop_line = use_stop_line;
  // st_search_config->stop_line_first_s = stop_line_first_s;
  // st_search_config->stop_line_second_s = stop_line_second_s;
  std::vector<double> acc_steps;
  for (auto acc = acc_search_min; acc < acc_search_max;
       acc += acc_search_step) {
    acc_steps.emplace_back(acc);
  }

  st_search_config->acc_step = acc_steps;
  // st_search_config->accel_sample_num = acc_steps.size();
  // st_search_config->vel_tolerance = vel_tolerance;
  // st_search_config->propoper_accel_value = propoper_accel_value;
  st_search_config->planning_time_horizon = 5.0;
  st_search_config->planning_time_horizon_inverse = 1 / 5.0;
  return 1;
}

int UpdateWeight(double yield_weight, double overtake_weight, double vel_weight,
                 double accel_weight, double accel_sign_weight,
                 double jerk_weight, double virtual_yield_weight,
                 double length_t_weight, double hcost_t_weight,
                 double hcost_s_weight, double hcost_v_weight,
                 double upper_trancation_time_buffer,
                 double lower_trancation_time_buffer,
                 double min_upper_distance_buffer,
                 double min_lower_distance_buffer, double comfort_gap_length,
                 double violation_weight, double s_dis_weight) {
  // cost_config->yield_weight = yield_weight;
  // cost_config->overtake_weight = overtake_weight;
  cost_config->vel_weight = vel_weight;
  cost_config->accel_weight = accel_weight;
  cost_config->jerk_weight = jerk_weight;
  // cost_config->virtual_yield_weight = virtual_yield_weight;
  // cost_config->length_t_weight = length_t_weight;
  // cost_config->hcost_t_weight = hcost_t_weight;
  cost_config->hcost_s_weight = hcost_s_weight;
  cost_config->hcost_v_weight = hcost_v_weight;
  // cost_config->upper_trancation_time_buffer = upper_trancation_time_buffer;
  // cost_config->lower_trancation_time_buffer = lower_trancation_time_buffer;
  // cost_config->min_upper_distance_buffer = min_upper_distance_buffer;
  // cost_config->min_lower_distance_buffer = min_lower_distance_buffer;
  cost_config->comfort_gap_length = comfort_gap_length;
  cost_config->violation_weight = violation_weight;
  cost_config->s_dis_weight = s_dis_weight;
  return 1;
}

int UpdateEgoState(double init_s, double init_v, double init_a,
                   double updated_heuristic_s) {
  init_sl->first = init_s;
  init_va->first = init_v;
  init_va->second = init_a;
  *heuristic_s = updated_heuristic_s;

  st_graph_base->ego_sl_.first = init_s;
  return 1;
}

int UpdateSearch() {
  search_path->UpdatePybind(*st_search_config, *cost_config, *st_graph_base,
                            *init_sl, *init_va);
  return 1;
}

std::vector<std::pair<double, double>> nodes() {
  std::vector<std::pair<double, double>> node_s;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    std::pair<double, double> s_t(item.second.s(), item.second.t());
    node_s.emplace_back(s_t);
  }
  return node_s;
}

std::vector<double> vel_cost_list() {
  std::vector<double> vel_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    vel_cost_list.emplace_back(item.second.vel_cost());
  }
  return vel_cost_list;
}

std::vector<double> acc_cost_list() {
  std::vector<double> acc_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    acc_cost_list.emplace_back(item.second.acc_cost());
  }
  return acc_cost_list;
}

std::vector<double> jerk_cost_list() {
  std::vector<double> jerk_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    jerk_cost_list.emplace_back(item.second.jerk_cost());
  }
  return jerk_cost_list;
}

std::vector<double> h_s_cost_list() {
  std::vector<double> h_s_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    h_s_cost_list.emplace_back(item.second.h_s_cost());
  }
  return h_s_cost_list;
}

std::vector<double> h_v_cost_list() {
  std::vector<double> h_v_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    h_v_cost_list.emplace_back(item.second.h_v_cost());
  }
  return h_v_cost_list;
}

std::vector<double> h_cost_list() {
  std::vector<double> h_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    h_cost_list.emplace_back(item.second.h_cost());
  }
  return h_cost_list;
}

std::vector<double> g_cost_list() {
  std::vector<double> g_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    g_cost_list.emplace_back(item.second.g_cost());
  }
  return g_cost_list;
}

std::vector<int> match_slot_list() {
  std::vector<int> match_slot_list;
  const auto &nodes = search_path->nodes();
  for (auto &item : nodes) {
    match_slot_list.emplace_back(item.second.aligned_slot_id());
  }
  return match_slot_list;
}

std::vector<double> v_list() {
  std::vector<double> v_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    v_list.emplace_back(item.second.vel());
  }
  return v_list;
}

std::vector<double> s_dis_cost_list() {
  std::vector<double> s_dis_cost_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    s_dis_cost_list.emplace_back(item.second.s_dis_cost());
  }
  return s_dis_cost_list;
}

std::vector<double> aligned_v_list() {
  std::vector<double> aligned_v_list;
  const auto &nodes = search_path->nodes();

  for (auto &item : nodes) {
    aligned_v_list.emplace_back(
        search_path->slot_point_info_[item.second.aligned_slot_id()].aligned_v);
  }
  return aligned_v_list;
}

std::vector<std::vector<Eigen::Vector2d>> GetAllSTBoundary() {
  const auto &boundary_id_st_boundaries_map =
      st_graph_base->boundary_id_st_boundaries_map();
  std::vector<std::vector<Eigen::Vector2d>> all_st_boundary;

  for (auto &item : boundary_id_st_boundaries_map) {
    auto &boundary = item.second;
    std::vector<Eigen::Vector2d> st_boundary;
    // std::cout << "boundary id:\n " << item.first << std::endl;
    // std::cout << "boundary lower_points " <<
    // boundary->lower_points().back().s()
    //           << std::endl;
    // std::cout << "boundary lower_points_size "
    // << boundary->lower_points().size() << std::endl;
    Eigen::Vector2d left_bottom_point(boundary->lower_points()[0].t(),
                                      boundary->lower_points()[0].s());
    Eigen::Vector2d right_bottom_point(boundary->lower_points().back().t(),
                                       boundary->lower_points().back().s());
    Eigen::Vector2d left_upper_point(boundary->upper_points()[0].t(),
                                     boundary->upper_points()[0].s());
    Eigen::Vector2d right_upper_point(boundary->upper_points().back().t(),
                                      boundary->upper_points().back().s());
    st_boundary.emplace_back(left_bottom_point);
    st_boundary.emplace_back(right_bottom_point);
    st_boundary.emplace_back(right_upper_point);
    st_boundary.emplace_back(left_upper_point);

    all_st_boundary.emplace_back(st_boundary);
  }
  return all_st_boundary;
}

std::vector<std::vector<Eigen::Vector2d>> GetSTIntervals() {
  const auto &st_points_table = st_graph_base->st_points_table();
  std::vector<std::vector<Eigen::Vector2d>> res;
  for (auto i = 0; i < st_points_table.size(); i++) {
    const auto intervals = st_points_table[i];
    std::vector<Eigen::Vector2d> t_points;
    for (auto j = 0; j < intervals.size(); j++) {
      Eigen::Vector2d lower_point(intervals[j].first.t(),
                                  intervals[j].first.s());
      Eigen::Vector2d upper_point(intervals[j].second.t(),
                                  intervals[j].second.s());

      t_points.emplace_back(lower_point);
      t_points.emplace_back(upper_point);
    }
    res.emplace_back(t_points);
  }
  return res;
}

std::vector<Eigen::Vector2d> GetSearchPath() {
  const auto &st_search_path = search_path->searched_path();
  std::vector<Eigen::Vector2d> st_search_path_res;

  for (auto &st_search_node : st_search_path) {
    Eigen::Vector2d search_point(st_search_node.t(), st_search_node.s());
    st_search_path_res.emplace_back(search_point);
  }
  return st_search_path_res;
}

std::vector<double> GetSearchPathS() {
  const auto &st_search_path = search_path->searched_path();
  std::vector<double> st_search_path_s_res;

  for (auto &st_search_node : st_search_path) {
    if (st_search_node.aligned_type() != STITCHED) {
      st_search_path_s_res.emplace_back(st_search_node.s());
    }
  }
  return st_search_path_s_res;
}

std::vector<double> GetSearchPathT() {
  const auto &st_search_path = search_path->searched_path();
  std::vector<double> st_search_path_t_res;

  for (auto &st_search_node : st_search_path) {
    if (st_search_node.aligned_type() != STITCHED) {
      st_search_path_t_res.emplace_back(st_search_node.t());
    }
  }
  return st_search_path_t_res;
}

std::vector<double> GetStitchPathS() {
  const auto &st_stitch_path = search_path->searched_path();
  std::vector<double> st_stitch_path_s_res;

  for (auto &st_search_node : st_stitch_path) {
    if (st_search_node.aligned_type() == STITCHED) {
      st_stitch_path_s_res.emplace_back(st_search_node.s());
    }
  }
  return st_stitch_path_s_res;
}

std::vector<double> GetStitchPathT() {
  const auto &st_stitch_path = search_path->searched_path();
  std::vector<double> st_stitch_path_t_res;

  for (auto &st_search_node : st_stitch_path) {
    if (st_search_node.aligned_type() == STITCHED) {
      st_stitch_path_t_res.emplace_back(st_search_node.t());
    }
  }
  return st_stitch_path_t_res;
}
double GetStopLineS() { return search_path->stop_line_s_; }

void UpdateObjInfo() {
  search_path->Preprocess(*st_search_config, *cost_config, *st_graph_base);
  return;
}
void ClearObjs() {
  st_graph_base->neighbor_agent_id_st_boundaries_map_.clear();
  st_graph_base->boundary_id_st_boundaries_map_.clear();
  st_graph_base->st_points_table_.clear();
  st_graph_base->lane_change_veh_vec_.clear();
  search_path->slot_point_info_.clear();
  search_path->nodes_.clear();
  return;
}

void AddObj(int obj_id, double obj_center_s, double obj_v) {
  if (obj_id < 0) return;

  // step0: clear virtual obj
  auto front_virtual_obj =
      std::find_if(st_graph_base->lane_change_veh_vec_.begin(),
                   st_graph_base->lane_change_veh_vec_.end(),
                   [obj_id](const LaneChangeVehInfo &veh) {
                     return veh.id == kNoAgentId - 1;
                   });

  if (front_virtual_obj != st_graph_base->lane_change_veh_vec_.end()) {
    st_graph_base->lane_change_veh_vec_.erase(front_virtual_obj);
  }

  auto rear_virtual_obj =
      std::find_if(st_graph_base->lane_change_veh_vec_.begin(),
                   st_graph_base->lane_change_veh_vec_.end(),
                   [obj_id](const LaneChangeVehInfo &veh) {
                     return veh.id == kNoAgentId - 2;
                   });

  if (rear_virtual_obj != st_graph_base->lane_change_veh_vec_.end()) {
    st_graph_base->lane_change_veh_vec_.erase(rear_virtual_obj);
  }

  // step1: check origin lane change veh vec exists obj
  auto iter = std::find_if(
      st_graph_base->lane_change_veh_vec_.begin(),
      st_graph_base->lane_change_veh_vec_.end(),
      [obj_id](const LaneChangeVehInfo &veh) { return veh.id == obj_id; });

  if (iter != st_graph_base->lane_change_veh_vec_.end()) {
    st_graph_base->lane_change_veh_vec_.erase(iter);
  }
  LaneChangeVehInfo veh_obj;
  veh_obj.center_s = obj_center_s;
  veh_obj.id = obj_id;
  veh_obj.v = obj_v;
  veh_obj.half_length = 2.6;
  st_graph_base->lane_change_veh_vec_.emplace_back(veh_obj);

  // step2:
  st_graph_base->ObjInfoProcess();
}

std::vector<double> GenerateHeruisticVList() {
  std::vector<double> v_list;
  for (auto slot : search_path->slot_point_info()) {
    v_list.emplace_back(slot.aligned_v);
    std::cout << "aligned v: " << slot.aligned_v << std::endl;
  }
  return v_list;
}

std::vector<std::vector<double>> GenerateHeruisticSList() {
  std::vector<std::vector<double>> aligned_s_list;
  for (auto slot : search_path->slot_point_info()) {
    std::vector<double> s_vec;
    s_vec = slot.aligned_s_vec;
    aligned_s_list.emplace_back(s_vec);
  }
  return aligned_s_list;
}

void MakeStPoints() {
  // add run slot elem generate

  st_graph_base->MakeStPointsTable();
  return;
}

PYBIND11_MODULE(st_search_py, m) {
  m.doc() = "m";
  m.def("Init", &Init)
      .def("UpdateBytes", &UpdateBytes)
      .def("UpdateParams", &UpdateParams)
      .def("UpdateWeight", &UpdateWeight)
      .def("UpdateEgoState", &UpdateEgoState)
      .def("UpdateSearch", &UpdateSearch)
      .def("GetAllSTBoundary", &GetAllSTBoundary)
      .def("GetSearchPath", &GetSearchPath)
      .def("ClearObjs", &ClearObjs)
      .def("AddObj", &AddObj)
      .def("MakeStPoints", &MakeStPoints)
      .def("GetSearchPathS", &GetSearchPathS)
      .def("GetSearchPathT", &GetSearchPathT)
      .def("GenerateHeruisticVList", &GenerateHeruisticVList)
      .def("nodes", nodes)
      .def("UpdateObjInfo", UpdateObjInfo)
      .def("GetSTIntervals", GetSTIntervals)
      .def("GetStopLineS", GetStopLineS)
      .def("GenerateHeruisticSList", GenerateHeruisticSList)
      .def("vel_cost_list", vel_cost_list)
      .def("acc_cost_list", acc_cost_list)
      .def("jerk_cost_list", jerk_cost_list)
      .def("h_s_cost_list", h_s_cost_list)
      .def("h_v_cost_list", h_v_cost_list)
      .def("h_cost_list", h_cost_list)
      .def("g_cost_list", g_cost_list)
      .def("match_slot_list", match_slot_list)
      .def("v_list", v_list)
      .def("s_dis_cost_list", s_dis_cost_list)
      .def("aligned_v_list", aligned_v_list)
      .def("GetStitchPathS", GetStitchPathS)
      .def("GetStitchPathT", GetStitchPathT)
      .def("UpdateParamsNew", UpdateParamsNew);
}