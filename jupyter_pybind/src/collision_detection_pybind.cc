#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "apa_planner/collision_detection/collision_detection.h"
#include "apa_planner/dubins_lib/dubins_lib.h"

namespace py = pybind11;
using namespace planning;
using namespace pnc::dubins_lib;

// 进行碰撞检测
static CollisionDetector* pBaseColDet = nullptr;

// 获取轨迹
static DubinsLibrary* pBaseDubins = nullptr;

int Init() {
  pBaseColDet = new CollisionDetector();
  pBaseColDet->Init();
  pBaseDubins = new DubinsLibrary();
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

int Update(double x_start, double y_start, double heading_start,
           double x_target, double y_target, double heading_target,
           double radius, uint8_t dubins_type, uint8_t case_type, double ds,
           bool is_complete_path) {
  DubinsLibrary::Input input;
  input.radius = radius;
  input.heading1 = heading_start;
  input.heading2 = heading_target;
  input.p1 << x_start, y_start;
  input.p2 << x_target, y_target;

  pBaseDubins->SetInput(input);
  pBaseDubins->Solve(dubins_type, case_type);
  pBaseDubins->Sampling(ds, is_complete_path);

  return 0;
}

int UpdateLineArc(double x_start, double y_start, double heading_start,
                  double x_target, double y_target, double heading_target,
                  double radius, uint8_t line_arc_type, double ds,
                  bool is_complete_path) {
  DubinsLibrary::Input input;
  input.radius = radius;
  input.heading1 = heading_start;
  input.heading2 = heading_target;
  input.p1 << x_start, y_start;
  input.p2 << x_target, y_target;

  pBaseDubins->SetInput(input);
  pBaseDubins->Solve(line_arc_type);
  pBaseDubins->Sampling(ds, is_complete_path);

  return 0;
}

Eigen::Vector2d GetABCenter() {
  return pBaseDubins->GetOutput().arc_AB.circle_info.center;
}

Eigen::Vector2d GetCDCenter() {
  return pBaseDubins->GetOutput().arc_CD.circle_info.center;
}

std::vector<double> GetPathEle(size_t index) {
  return pBaseDubins->GetPathEle(index);
}

Eigen::Vector2d GetpB() { return pBaseDubins->GetOutput().line_BC.pA; }
Eigen::Vector2d GetpC() { return pBaseDubins->GetOutput().line_BC.pB; }
Eigen::Vector2d GetpD() { return pBaseDubins->GetOutput().arc_CD.pB; }
bool GetPathAvailiable() { return pBaseDubins->GetOutput().path_available; }
double GetLength() { return pBaseDubins->GetOutput().length; }
std::vector<uint8_t> GetGearCmdVec() {
  return pBaseDubins->GetOutput().gear_cmd_vec;
}
uint8_t GetGearChangeCount() {
  return pBaseDubins->GetOutput().gear_change_count;
}
double GetThetaBC() { return pBaseDubins->GetThetaBC(); }
double GetThetaD() { return pBaseDubins->GetThetaD(); }
double GetRadius() {
  return pBaseDubins->GetOutput().arc_AB.circle_info.radius;
}

void GenObstacleLinePb(const std::vector<Eigen::Vector2d>& start_point_vec,
                       const std::vector<Eigen::Vector2d>& end_point_vec) {
  //sstd::cout << "\nGenObstacleLinePb";                    
  pnc::geometry_lib::LineSegment obstacle_line;
  std::vector<pnc::geometry_lib::LineSegment> obstacle_line_vec;
  obstacle_line_vec.clear();
  obstacle_line_vec.reserve(start_point_vec.size());
  for (size_t i = 0; i < start_point_vec.size(); i++) {
    obstacle_line.SetPoints(start_point_vec[i], end_point_vec[i]);
    obstacle_line_vec.emplace_back(std::move(obstacle_line));
  }
  pBaseColDet->GenObstaclesSimulation(obstacle_line_vec);
}

void GenCarCirclePb(const std::vector<double>& path_x,
                    const std::vector<double>& path_y,
                    const std::vector<double>& path_theta) {
  //std::cout << "\nGenCarCirclePb";
  std::vector<pnc::dubins_lib::DubinsLibrary::PathPoint> path_point_vec;
  path_point_vec.clear();
  path_point_vec.reserve(path_x.size());
  pnc::dubins_lib::DubinsLibrary::PathPoint path_point;
  // std::cout << "\npath_point_vec0.size():" << path_point_vec.size();
  for (size_t i = 0; i < path_x.size(); i++) {
    path_point.Set(Eigen::Vector2d(path_x[i], path_y[i]), path_theta[i]);
    path_point_vec.emplace_back(path_point);
  }
  // std::cout << "\npath_point_vec1.size():" << path_point_vec.size();
  pBaseColDet->GenCarCircles(path_point_vec);
  // std::cout << "\npath_point_vec2.size():" << path_point_vec.size();
}

const std::vector<std::vector<Eigen::Vector3d>> GetCarCirclePb() {
  //std::cout << "\nGetCarCirclePb";
  const auto &N = pBaseColDet->GetCarCircle().size();
  const auto &M = pBaseColDet->GetCarCircle()[0].size();
  // std::cout << "\nN:" << N;
  // std::cout << "\nM:" << M;
  std::vector<std::vector<Eigen::Vector3d>> circle_path_vec;
  circle_path_vec.clear();
  circle_path_vec.reserve(pBaseColDet->GetCarCircle().size());
  std::vector<Eigen::Vector3d> circle_vec;
  circle_vec.clear();
  circle_vec.reserve(pBaseColDet->GetCarCircle()[0].size());
  Eigen::Vector3d circle;
  // std::cout << "\ncircle_path_vec.size():" << circle_path_vec.size();
  //std::cout << "\ncircle_vec.size():" << circle_vec.size();
  for (const auto& car_circle_global_vec : pBaseColDet->GetCarCircle()) {
    for (const auto& car_circle_global : car_circle_global_vec) {
      circle << car_circle_global.center.x(), car_circle_global.center.y(),
          car_circle_global.radius;
      circle_vec.emplace_back(circle);
    }
    circle_path_vec.emplace_back(std::move(circle_vec));
  }
  // std::cout << "\ncircle_path_vec.size():" << circle_path_vec.size();
  return circle_path_vec;
}

bool CollisionDetectPb() {
  //std::cout << "\nCollisionDetectPb";
  // printf("\nnCollisionDetectPb");
  // return false;
  return pBaseColDet->CollisionDetect();
}

PYBIND11_MODULE(collision_detection_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("UpdateLineArc", &UpdateLineArc)
      .def("GetPathEle", &GetPathEle)
      .def("GenObstacleLinePb", &GenObstacleLinePb)
      .def("GenCarCirclePb", &GenCarCirclePb)
      .def("GetCarCirclePb", GetCarCirclePb)
      .def("CollisionDetectPb", CollisionDetectPb)
      .def("GetABCenter", &GetABCenter)
      .def("GetCDCenter", &GetCDCenter)
      .def("GetpB", &GetpB)
      .def("GetpC", &GetpC)
      .def("GetpD", &GetpD)
      .def("GetThetaBC", &GetThetaBC)
      .def("GetThetaD", &GetThetaD)
      .def("GetPathAvailiable", &GetPathAvailiable)
      .def("GetLength", &GetLength)
      .def("GetGearCmdVec", &GetGearCmdVec)
      .def("GetGearChangeCount", &GetGearChangeCount)
      .def("GetRadius", &GetRadius);
}
