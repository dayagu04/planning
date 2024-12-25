#ifndef __USS_OBSTACLE_AVOIDANCE_H__
#define __USS_OBSTACLE_AVOIDANCE_H__

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "apa_data.h"
#include "apa_measure_data_manager.h"
#include "apa_obstacle_manager.h"
#include "apa_param_config.h"
#include "apa_predict_path_manager.h"
#include "collision_detection/collision_detection.h"
#include "geometry_math.h"
#include "local_view.h"
#include "math_lib.h"
#include "planning_plan_c.h"

namespace planning {

namespace apa_planner {
class UssObstacleAvoidance {
 public:
  enum CarPointMode {
    LINE_MODE = 0,
    ARC_MODE = 1,
    COUNT,
  };

  struct Paramters {
    double steer_ratio = apa_param.GetParam().steer_ratio;
    double arc_line_shift_steer_angle_deg =
        apa_param.GetParam().arc_line_shift_steer_angle_deg;
    double c1 = apa_param.GetParam().c1;
    double detection_distance = apa_param.GetParam().detection_distance;
    double lat_inflation = apa_param.GetParam().lat_inflation;
  };

  struct RemainDistInfo {
    double remain_dist = 3.0;
    double obs_pt_remain_dist = 3.0;
    double vel_target = 2.0;
    size_t car_index = 0;
    int uss_index = 0;
    bool is_available = false;

    void Reset() {
      remain_dist = 3.0;
      obs_pt_remain_dist = 3.0;
      vel_target = 2.0;
      car_index = 0;
      uss_index = 0;
      is_available = false;
    }
  };

  struct CarMotionInfo {
    double steer_angle = 0.0;
    bool reverse_flag = false;
    double rear_axle_center_turn_radius = 1.0e4;
    Eigen::Vector2d turning_center = Eigen::Vector2d::Zero();
    uint8_t car_motion_mode = LINE_MODE;

    void Reset() {
      steer_angle = 0.0;
      reverse_flag = false;
      rear_axle_center_turn_radius = 1.0e4;
      turning_center = Eigen::Vector2d::Zero();
      car_motion_mode = LINE_MODE;
    }
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UssObstacleAvoidance() { Init(); }

  void Reset() {
    remain_dist_info_.Reset();
    car_motion_info_.Reset();
  }

  void Init();

  void SetDisable() { remain_dist_info_.is_available = false; }

  const RemainDistInfo& GetRemainDistInfo() const { return remain_dist_info_; }

  void Update(const std::shared_ptr<ApaData> apa_data_ptr,
              const std::shared_ptr<ApaMeasureDataManager> measure_data_ptr,
              const std::shared_ptr<ApaPredictPathManager> predict_path_ptr,
              const std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr);

  void SetParam(const Paramters& param) {
    param_ = param;
    SetLatInflation();
  }

  const bool CheckIsDirectlyBehindUss();
  const bool CheckIsDirectlyFrontUss();

  void SetLatInflation();

  // for simulation by pybind
  void SetUssRawDist(const double& uss_raw_dist);

  void SetCarMotionInfo(const CarMotionInfo& car_motion_info) {
    car_motion_info_ = car_motion_info;
  }

  const std::vector<pnc::geometry_lib::LineSegment>& GetCarLocalLine() {
    return car_local_line_vec_;
  }

  const std::vector<Eigen::Vector2d> GetCarLocalVertex();
  const std::vector<Eigen::Vector2d> GetUssLocalVertex();
  const std::vector<double> GetUssLocalAngle();

  const std::vector<pnc::geometry_lib::Arc>& GetCarLocalArc() {
    return car_local_arc_vec_;
  }

  const std::vector<double>& GetUssDistVec() { return uss_raw_dist_vec_; }

  void UpdateByPybind();

 private:
  const bool Preprocess();

  void GenCarArc();

  void GenCarLine();

  void GenUssArc();

  void CalRemainDist();

  std::vector<Eigen::Vector2d> car_local_vertex_vec_;

  std::vector<Eigen::Vector2d> uss_local_vertex_vec_;
  std::vector<double> uss_local_normal_angle_vec_;
  std::vector<double>
      uss_raw_dist_vec_;  // front, rear uss in clockwise direction

  std::vector<pnc::geometry_lib::LineSegment> car_local_line_vec_;
  std::vector<pnc::geometry_lib::Arc> car_local_arc_vec_;

  std::vector<pnc::geometry_lib::Arc> uss_local_arc_vec_;

  RemainDistInfo remain_dist_info_;
  Paramters param_;
  CarMotionInfo car_motion_info_;

  std::shared_ptr<ApaPredictPathManager> predict_path_ptr_ = nullptr;
  std::shared_ptr<ApaMeasureDataManager> measure_data_ptr_ = nullptr;
  std::shared_ptr<ApaObstacleManager> obstacle_manager_ptr_ = nullptr;
  std::shared_ptr<ApaData> apa_data_ptr_ = nullptr;

  CollisionDetector col_det;
};
}  // namespace apa_planner
}  // namespace planning

#endif