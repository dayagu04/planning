#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include "Eigen/Core"
#include "geometry_math.h"
#include "math_lib.h"
#include "transform_lib.h"
#include "uss_obstacle_avoidance.h"
#include "apa_plan_interface.h"

namespace py = pybind11;
using namespace planning;

static UssObstacleAvoidance* pBaseUssOaAir = nullptr;

static planning::apa_planner::ApaPlanInterface *pApaPlanInterface = nullptr;

int Init() {
  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();

  pApaPlanInterface->Init();


  pBaseUssOaAir = new UssObstacleAvoidance();
  return 0;
}

template <class T>
inline T BytesToProto(py::bytes& bytes) {
  T proto_obj;
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char* input_ptr = static_cast<char*>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

void SetParam(const double& detection_distance, const double& lat_inflation) {
  UssObstacleAvoidance::Paramters param;
  param.detection_distance = detection_distance;
  param.lat_inflation = lat_inflation;
  pBaseUssOaAir->SetParam(param);
}

void SetUssRawDist(const double& uss_raw_dist) {
  pBaseUssOaAir->SetUssRawDist(uss_raw_dist);
}

void SetCarMotionInfo(const double& steer_angle, const int& forward_back) {
  UssObstacleAvoidance::CarMotionInfo car_motion_info;
  car_motion_info.steer_angle = steer_angle;
  car_motion_info.reverse_flag = (forward_back == 0) ? false : true;
  pBaseUssOaAir->SetCarMotionInfo(car_motion_info);
}

void UpdateUssDis() { pBaseUssOaAir->UpdateByPybind(); }

const double GetRemainDist() {
  return pBaseUssOaAir->GetRemainDistInfo().remain_dist;
}

const bool GetAvailable() {
  return pBaseUssOaAir->GetRemainDistInfo().is_available;
}

const size_t GetUssIndex() {
  return pBaseUssOaAir->GetRemainDistInfo().uss_index;
}

const size_t GetCarIndex() {
  return pBaseUssOaAir->GetRemainDistInfo().car_index;
}

const std::vector<Eigen::Vector2d> GetCarLine() {
  std::vector<Eigen::Vector2d> car_line;
  car_line.clear();
  car_line.reserve(2);
  if (pBaseUssOaAir->GetRemainDistInfo().is_available) {
    car_line.emplace_back(
        pBaseUssOaAir
            ->GetCarLocalLine()[pBaseUssOaAir->GetRemainDistInfo().car_index]
            .pA);
    car_line.emplace_back(
        pBaseUssOaAir
            ->GetCarLocalLine()[pBaseUssOaAir->GetRemainDistInfo().car_index]
            .pB);
  } else {
    car_line.resize(2, Eigen::Vector2d(0.0, 0.0));
  }
  return car_line;
}

const Eigen::Vector3d GetCarArc() {
  Eigen::Vector3d car_arc;
  if (pBaseUssOaAir->GetRemainDistInfo().is_available) {
    pnc::geometry_lib::Arc arc =
        pBaseUssOaAir
            ->GetCarLocalArc()[pBaseUssOaAir->GetRemainDistInfo().car_index];
    car_arc << arc.circle_info.center.x(), arc.circle_info.center.y(),
        arc.circle_info.radius;
  } else {
    car_arc << 0.0, 0.0, 0.0;
  }
  return car_arc;
}

PYBIND11_MODULE(uss_obstacle_avoidance_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("SetUssRawDist", &SetUssRawDist)
      .def("SetCarMotionInfo", &SetCarMotionInfo)
      .def("UpdateUssDis", &UpdateUssDis)
      .def("GetRemainDist", &GetRemainDist)
      .def("GetAvailable", &GetAvailable)
      .def("GetUssIndex", &GetUssIndex)
      .def("GetCarIndex", &GetCarIndex)
      .def("GetCarLine", &GetCarLine)
      .def("GetCarArc", &GetCarArc)
      .def("SetParam", &SetParam);
}
