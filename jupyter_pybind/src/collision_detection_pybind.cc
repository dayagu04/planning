#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "Eigen/Core"
#include "apa_plan_interface.h"
#include "collision_detection.h"
#include "geometry_math.h"
#include "math_lib.h"
#include "transform_lib.h"

namespace py = pybind11;
using namespace planning;

namespace Eigen {
typedef Eigen::Matrix<double, 5, 1> Vector5d;
}

static CollisionDetector* pBaseColDetAir = nullptr;
static CollisionDetector::CollisionResult collision_result;
static std::vector<Eigen::Vector3d> pt_vec;
static planning::apa_planner::ApaPlanInterface* pApaPlanInterface = nullptr;
static bool is_obs_in_car = false;
static Eigen::Vector2d obs;
static double length = 0.0;

int Init() {
  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();

  pApaPlanInterface->Init();
  pBaseColDetAir = new CollisionDetector();
  return 0;
}

void SetObstacle(const double obstacles_x, const double obstacles_y) {
  std::vector<Eigen::Vector2d> obstacle_global_vec;
  obstacle_global_vec.clear();
  obstacle_global_vec.reserve(1);
  Eigen::Vector2d obstacle_global(obstacles_x, obstacles_y);

  obstacle_global_vec.emplace_back(obstacle_global);

  obs = obstacle_global;

  pBaseColDetAir->SetObstacles(obstacle_global_vec,
                               planning::CollisionDetector::TLANE_OBS);
}

void SetObstacleLine(const double obstacles_x1, const double obstacles_y1,
                     const double obstacles_x2, const double obstacles_y2) {
  std::vector<pnc::geometry_lib::LineSegment> obs_line_global_vec;
  obs_line_global_vec.clear();
  obs_line_global_vec.reserve(1);
  pnc::geometry_lib::LineSegment obs_line_global;
  Eigen::Vector2d obstacle_global_1(obstacles_x1, obstacles_y1);
  Eigen::Vector2d obstacle_global_2(obstacles_x2, obstacles_y2);
  obs_line_global.SetPoints(obstacle_global_1, obstacle_global_2);
  obs_line_global_vec.emplace_back(obs_line_global);

  pBaseColDetAir->SetLineObstacles(obs_line_global_vec);
}

void SetParam(const double lat_inflation) {
  CollisionDetector::Paramters param;
  param.lat_inflation = lat_inflation;
  pBaseColDetAir->SetParam(param);
}

void UpdateRefTrajLine(const Eigen::Vector3d ego_pos_start,
                       const Eigen::Vector3d ego_pos_end,
                       const int is_line_obs) {
  pt_vec.clear();
  pnc::geometry_lib::LineSegment line_seg(
      Eigen::Vector2d(ego_pos_start[0], ego_pos_start[1]),
      Eigen::Vector2d(ego_pos_end[0], ego_pos_end[1]), ego_pos_start[2]);
  if (is_line_obs == 0) {
    collision_result =
        pBaseColDetAir->UpdateByObsMap(line_seg, ego_pos_start[2]);
  } else {
    collision_result =
        pBaseColDetAir->UpdateByLineObs(line_seg, ego_pos_start[2]);
  }
  std::vector<pnc::geometry_lib::PathPoint> pose_vec;
  pnc::geometry_lib::SamplePointSetInLineSeg(pose_vec, line_seg, 0.5);
  for (size_t i = 0; i < pose_vec.size(); ++i) {
    pt_vec.emplace_back(pose_vec[i].pos.x(), pose_vec[i].pos.y(),
                        pose_vec[i].heading);
  }

  is_obs_in_car = pBaseColDetAir->IsObstacleInCar(
      obs,
      pnc::geometry_lib::PathPoint(
          Eigen::Vector2d(ego_pos_start[0], ego_pos_start[1]),
          ego_pos_start[2]),
      0.02);
}

void UpdateRefTrajArc(const Eigen::Vector3d ego_pos_start,
                      const Eigen::Vector3d ego_pos_end,
                      const Eigen::Vector5d ego_turn_circle,
                      bool is_anti_clockwise, const int is_line_obs) {
  pt_vec.clear();
  pnc::geometry_lib::Arc arc;
  arc.pA = Eigen::Vector2d(ego_pos_start[0], ego_pos_start[1]);
  arc.pB = Eigen::Vector2d(ego_pos_end[0], ego_pos_end[1]);
  arc.circle_info.center =
      Eigen::Vector2d(ego_turn_circle[0], ego_turn_circle[1]);
  arc.circle_info.radius = ego_turn_circle[2];
  arc.is_anti_clockwise = is_anti_clockwise;
  arc.headingA = ego_pos_start[2];
  arc.headingB = ego_pos_end[2];
  arc.length = length;
  if (!is_line_obs) {
    collision_result = pBaseColDetAir->UpdateByObsMap(arc, ego_pos_start[2]);
  } else {
    collision_result = pBaseColDetAir->UpdateByLineObs(arc, ego_pos_start[2]);
  }
  std::vector<pnc::geometry_lib::PathPoint> pose_vec;
  pnc::geometry_lib::SamplePointSetInArc(pose_vec, arc, 0.5);
  for (size_t i = 0; i < pose_vec.size(); ++i) {
    pt_vec.emplace_back(pose_vec[i].pos.x(), pose_vec[i].pos.y(),
                        pose_vec[i].heading);
  }

  is_obs_in_car = pBaseColDetAir->IsObstacleInCar(
      obs,
      pnc::geometry_lib::PathPoint(
          Eigen::Vector2d(ego_pos_start[0], ego_pos_start[1]),
          ego_pos_start[2]),
      0.02);
}

const double GetRemainDist() { return float(collision_result.remain_dist); }

const double GetRemainCarDist() {
  return float(collision_result.remain_car_dist);
}

const double GetRemainObstacleDist() {
  return float(collision_result.remain_obstacle_dist);
}

const int GetCarLineOrder() { return collision_result.car_line_order; }

const bool GetCollisionFlag() { return collision_result.collision_flag; }

const Eigen::Vector2d GetCollisionPointEgoGlobal() {
  return collision_result.col_pt_ego_global;
}

const std::vector<Eigen::Vector3d> GetSamplePt() { return pt_vec; }

const bool GetObsIsInCar() { return is_obs_in_car; }

const Eigen::Vector2d GetTrunCenterCoord(
    const Eigen::Vector3d ego_pos_start,
    const Eigen::Vector5d ego_turn_circle) {
  const Eigen::Vector2d tangent_unit_vector(cos(ego_pos_start[2]),
                                            sin(ego_pos_start[2]));
  // ego_pos_start: x, y, heading
  // ego_turn_circle: x, y, radius, rotation_angle, rotation_direction
  const double sign = (ego_turn_circle[4] == true ? 1.0 : -1.0);
  const double rot_angle = sign * pnc::mathlib::Deg2Rad(90);
  length = std::fabs(rot_angle * ego_turn_circle[2]);
  const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(rot_angle);
  const Eigen::Vector2d normal_unit_vector = rot_m * tangent_unit_vector;
  const Eigen::Vector2d AO = ego_turn_circle[2] * normal_unit_vector;
  return Eigen::Vector2d(ego_pos_start[0], ego_pos_start[1]) + AO;
}

const Eigen::Vector2d GetEgoPosCoord(const Eigen::Vector3d ego_pos_start,
                                     const Eigen::Vector5d ego_turn_circle) {
  // ego_pos_start: x, y, heading
  // ego_turn_circle: x, y, radius, rotation_angle, rotation_direction
  const Eigen::Vector2d OA =
      Eigen::Vector2d(ego_pos_start[0] - ego_turn_circle[0],
                      ego_pos_start[1] - ego_turn_circle[1]);

  const double sign = (ego_turn_circle[4] == true ? 1.0 : -1.0);
  const double rot_angle = sign * ego_turn_circle[3];
  const auto rot_m = pnc::geometry_lib::GetRotm2dFromTheta(rot_angle);
  Eigen::Vector2d OB = rot_m * OA;
  return OB + Eigen::Vector2d(ego_turn_circle[0], ego_turn_circle[1]);
}

PYBIND11_MODULE(collision_detection_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("SetObstacle", &SetObstacle)
      .def("SetObstacleLine", &SetObstacleLine)
      .def("UpdateRefTrajLine", &UpdateRefTrajLine)
      .def("UpdateRefTrajArc", &UpdateRefTrajArc)
      .def("GetTrunCenterCoord", &GetTrunCenterCoord)
      .def("GetEgoPosCoord", &GetEgoPosCoord)
      .def("GetRemainDist", &GetRemainDist)
      .def("GetCollisionFlag", &GetCollisionFlag)
      .def("GetCollisionPointEgoGlobal", &GetCollisionPointEgoGlobal)
      .def("GetRemainCarDist", &GetRemainCarDist)
      .def("GetRemainObstacleDist", &GetRemainObstacleDist)
      .def("GetCarLineOrder", &GetCarLineOrder)
      .def("SetParam", &SetParam)
      .def("GetSamplePt", &GetSamplePt)
      .def("GetObsIsInCar", &GetObsIsInCar);
}
