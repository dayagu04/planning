#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <sys/param.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>
#include <unordered_map>
#include <utility>
#include <vector>
#include "behavior_planners/dp_path_decider/dp_road_graph.h"
#include "behavior_planners/dp_path_decider/dp_base.h"
#include "jupyter_pybind/src/serialize_utils.h"
#include "src/modules/common/config/message_type.h"
#include "src/modules/common/math/curve1d/quintic_polynomial_curve1d.h"

namespace py = pybind11;
using namespace planning;

static DPRoadGraph* pDPRoadGraph;
int Init() {
  pDPRoadGraph = new DPRoadGraph();
  return 1;
}

void ClearInfo(){
  pDPRoadGraph->mutable_sampled_points().clear();
  pDPRoadGraph->mutable_dp_selected_points().clear();
  pDPRoadGraph->muteble_refined_paths().clear();
  pDPRoadGraph->muteble_min_cost_path().clear();
  return;// no clear function
}
// bag data processed to set class members to call functions
bool InputProcess(py::bytes& dp_path_input_bytes){
  ClearInfo();
  planning::common::DPRoadInfo dp_road_input =
      BytesToProto<planning::common::DPRoadInfo>(dp_path_input_bytes);
  // sampled sl
    const auto& sample_points_input = dp_road_input.sample_lanes_info().sampled_points();
  if (sample_points_input.size() == 0) {  // No speed adjust info
    return 0;
  }
  std::vector<std::vector<planning::SLPoint>> sampled_points;
  for (auto & level_points : sample_points_input) {
    std::vector<SLPoint> level_sampled_points;  // 临时存储这一层的点
    for (auto & point : level_points.level_points()) {
        level_sampled_points.emplace_back(point.s(), point.l());
    }
    sampled_points.emplace_back(std::move(level_sampled_points));
  }
  pDPRoadGraph->mutable_sampled_points() = sampled_points;

// sample param
  pDPRoadGraph->set_v_cruise(dp_road_input.v_cruise());
  pDPRoadGraph->set_ego_s(dp_road_input.print_info().ego_s());
  pDPRoadGraph->set_ego_l(dp_road_input.print_info().ego_l());
  pDPRoadGraph->set_s_range(dp_road_input.s_range());
  pDPRoadGraph->set_l_range(dp_road_input.l_range());
  pDPRoadGraph->set_total_length(dp_road_input.total_length());
  pDPRoadGraph->set_left_boundary(dp_road_input.sample_lanes_info().left_boundary());// important
  pDPRoadGraph->set_right_boundary(dp_road_input.sample_lanes_info().right_boundary());
  //dp cost init
  const auto& dp_param_input = dp_road_input.dp_param();
  pDPRoadGraph->set_coeff_l_cost(dp_param_input.coeff_l_cost());
  pDPRoadGraph->set_coeff_dl_cost(dp_param_input.coeff_dl_cost());
  pDPRoadGraph->set_coeff_ddl_cost(dp_param_input.coeff_ddl_cost());
  pDPRoadGraph->set_coeff_end_l_cost(dp_param_input.coeff_end_l_cost());
  pDPRoadGraph->set_path_resolution(dp_param_input.path_resolution());
  pDPRoadGraph->set_coeff_collision_cost(dp_param_input.coeff_collision_cost());
  pDPRoadGraph->set_collision_distance(dp_param_input.collision_distance());

   const auto& obstacles_input = dp_road_input.obstacles_info();
  std::vector<StaticObstacleInfo> obstacles_infos;

  obstacles_infos.reserve((obstacles_input.size()));
  for (auto& obs_input : obstacles_input) {
    StaticObstacleInfo obs{obs_input.id(), obs_input.s_start(),
                    obs_input.s_end(), obs_input.l_start(),obs_input.l_end()};
    obstacles_infos.emplace_back(std::move(obs));
  }
  std::sort(obstacles_infos.begin(), obstacles_infos.end(),
            [](StaticObstacleInfo& a, StaticObstacleInfo& b) -> bool {
              return a.s_start < b.s_start;
            });
  pDPRoadGraph->mutable_obstacles_info() = obstacles_infos;
  // pSamplePolySpeedAdjustDecider->mutable_agent_info() = agent_infos;
  return true;
}
bool UpdateParams(double s_range,double l_range,\
                  double coeff_l_cost,double coeff_dl_cost,double coeff_ddl_cost,\
                  double coeff_end_l_cost, double coeff_collision_cost,double path_resolution,double collision_distance){
  // pDPRoadGraph->set_s_range(s_range); hacked not update  s range
  pDPRoadGraph->set_l_range(l_range);
  //dp params
  pDPRoadGraph->set_coeff_l_cost(coeff_l_cost);
  pDPRoadGraph->set_coeff_dl_cost(coeff_dl_cost);
  pDPRoadGraph->set_coeff_ddl_cost(coeff_ddl_cost);
  pDPRoadGraph->set_coeff_end_l_cost(coeff_end_l_cost);
  // pDPRoadGraph->set_path_resolution(path_resolution);
  pDPRoadGraph->set_coeff_collision_cost(coeff_collision_cost);
  pDPRoadGraph->set_collision_distance(collision_distance);
  return true;
}
int Execute(){
  bool ok = true;
  // if(ok){
  //   ok = pDPRoadGraph->SampleLanes();
  // }
  // if(ok){
  //   ok = pDPRoadGraph->DPSearchPath();
  // }
  // if(ok){
  //   ok = pDPRoadGraph->FinedReferencePath();
  // }

  return 1;
}
std::vector<Eigen::Vector2d> GetSamplePoints() {
  std::vector<Eigen::Vector2d> sample_vec;
  auto& sample_points = pDPRoadGraph->sampled_points();
  if (!sample_points.empty()) {
    for (auto &level_points:sample_points){
      for(auto &point:level_points){
        sample_vec.emplace_back(Eigen::Vector2d(point.s,point.l));
      }
    }
  } else {
    std::cout << "empty " << std::endl;
    return sample_vec;
  }
  return sample_vec;

}
std::vector<Eigen::Vector2d> GetFinedDPPath() {
  std::vector<Eigen::Vector2d> path_vec;
  auto& fined_path = pDPRoadGraph->refined_paths();
  if (!fined_path.empty()) {
    for (auto &path_point:fined_path){
        path_vec.emplace_back(Eigen::Vector2d(path_point.s(),path_point.l()));
    }
  } else {
    std::cout << "empty " << std::endl;
    return path_vec;
  }
  return path_vec;
}

py::bytes get_print_table_string() {
  planning::common::PrintInfo dp_path_print_info;
  // 使用接口函数替代直接访问私有成员
  // dp_path_print_info.set_ego_s(pDPRoadGraph->ego_s());
  // 使用接口函数访问 stitched_last_best_quartic_poly_ptr_
  std::string serialized_print_table_info;
  dp_path_print_info.SerializeToString(&serialized_print_table_info);
  return serialized_print_table_info;
}

PYBIND11_MODULE(dp_path_decider_py,m){
  m.doc() = "m";
  m.def("Init", &Init)  // 绑定 C++ 函数 Init 到 Python
     .def("Execute", &Execute)  // 绑定 Execute
     .def("InputProcess", &InputProcess)  // 绑定 ProcessInput
     .def("UpdateParams", &UpdateParams)  // 绑定 UpdateParam (少了 & 可能是宏定义或静态函数)
     .def("GetSamplePoints",&GetSamplePoints)
     .def("GetFinedDPPath",&GetFinedDPPath)
     .def("get_print_table_string", &get_print_table_string);  // 绑定 get_print_table_string
}

