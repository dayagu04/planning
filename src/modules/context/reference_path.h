#pragma once

#include <memory>
#include <vector>

#include "common_c.h"
#include "config/basic_type.h"
#include "frenet_ego_state.h"
#include "frenet_obstacle.h"
#include "math/discretized_points_smoothing/fem_pos_deviation_smoother.h"
#include "session.h"
#include "utils/kd_path.h"
#include "utils/path_point.h"

// class ObstacleManager;
namespace planning {

enum class ReferencePathSource {
  FUSION_ROAD,
  FUNCTIONAL_STATE_MACHINE,
  CONSTRUCTION_SCENE,
  NARROW_SPACE_SCENE
};

enum class ReferencePathPointType { MAP, TRAJ, INTERPOLATE };

struct ReferencePathPoint {
  planning_math::PathPoint path_point;
  double distance_to_left_road_border;
  double distance_to_right_road_border;
  double distance_to_left_lane_border;
  double distance_to_right_lane_border;
  iflyauto::LaneBoundaryType left_road_border_type;
  iflyauto::LaneBoundaryType right_road_border_type;
  iflyauto::LaneBoundaryType left_lane_border_type;
  iflyauto::LaneBoundaryType right_lane_border_type;
  double lane_width;
  double max_velocity;
  double min_velocity;
  ReferencePathPointType type;
  bool is_in_intersection;
};
using ReferencePathPoints = std::vector<ReferencePathPoint>;

struct ReferencePathCurveInfo {
  // 弯道类型
  enum class CurveType {
    STRAIGHT = 0,      // 直道
    NORMAL_CURVE = 1,  // 弯道
    BIG_CURVE = 2,     // 大曲率弯道
    S_CURVE = 3,       // S弯
    SHARP_CURVE = 4,   // 急弯
  };

  CurveType curve_type = CurveType::STRAIGHT;
  bool is_left = false;                                  // 是否左弯
  bool is_right = false;                                 // 是否右弯
  double start_s = 0.0;                                  // 大曲率起始纵向位置
  double end_s = 0.0;                                    // 大曲率结束纵向位置
  double max_curve = 1e-4;                               // 最大曲率值
  double max_curve_s = 0.0;                              // 最大曲率对应的纵向位置
  double min_radius = 1e4;                               // 最小曲率半径
  std::pair<double, double> left_s_range{0.0, 0.0};      // 左弯<起始 - 结束>纵向位置
  std::pair<double, double> right_s_range{0.0, 0.0};     // 右弯<起始 - 结束>纵向位置
  std::pair<double, double> left_max_curve{1e-4, 0.0};   // 左弯<最大曲率 - 对应纵向位置>
  std::pair<double, double> right_max_curve{1e-4, 0.0};  // 右弯<最大曲率 - 对应纵向位置>
  std::vector<double> curve_vec;                         // 曲率序列
  std::vector<double> s_vec;                             // 对应的纵向距离序列

  void Clear() {
    curve_type = CurveType::STRAIGHT;
    is_left = false;
    is_right = false;
    start_s = 0.0;
    end_s = 0.0;
    min_radius = 1e4;
    max_curve = 1e-4;
    max_curve_s = 0.0;
    left_s_range = std::make_pair(0.0, 0.0);
    right_s_range = std::make_pair(0.0, 0.0);
    left_max_curve = std::make_pair(1e-4, 0.0);
    right_max_curve = std::make_pair(1e-4, 0.0);
    curve_vec.clear();
    s_vec.clear();
  }
};

struct SmootherData {
  bool is_available = false;
  std::vector<double> points_x;
  std::vector<double> points_y;
  std::vector<double> points_s;
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;

  void Clear() {
    is_available = false;
    points_x.clear();
    points_y.clear();
    points_s.clear();
  }
};

class ReferencePath {
 public:
  ReferencePath();

  virtual ~ReferencePath() = default;

  bool valid() { return valid_; }

  virtual void update(planning::framework::Session *session);

  // virtual void Update(planning::framework::Session *session, ReferencePathPoints &raw_reference_path_points);

  virtual void update_obstacles();

  const ReferencePathPoint& GetRawStartRefPathPoint() const {
    return raw_start_point_;
  }

  const ReferencePathPoint& GetRawEndRefPathPoint() const {
    return raw_end_point_;
  }

  const std::vector<ReferencePathPoint> &get_points() const {
    return refined_ref_path_points_;
  }

  const std::shared_ptr<planning_math::KDPath> &get_frenet_coord() const {
    return frenet_coord_;
  }

  const FrenetEgoState &get_frenet_ego_state() const {
    return frenet_ego_state_;
  }

  const FrenetBoundary &get_ego_frenet_boundary() const {
    return frenet_ego_state_.boundary();
  }

  const std::vector<std::shared_ptr<FrenetObstacle>> &get_obstacles() const {
    return frenet_obstacles_;
  }

  std::vector<std::shared_ptr<FrenetObstacle>> &mutable_obstacles() {
    return frenet_obstacles_;
  }

  std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
      &mutable_obstacles_map() {
    return frenet_obstacles_map_;
  }

  const std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
      &get_obstacles_map() const {
    return frenet_obstacles_map_;
  }

  // std::vector<int> &mutable_obstacles_in_lane_map() {
  //   return obstacles_in_lane_map_;
  // }

  // const std::vector<int> &get_obstacles_in_lane_map() const {
  //   return obstacles_in_lane_map_;
  // }

  virtual bool is_obstacle_ignorable(
      const std::shared_ptr<FrenetObstacle> obstacle);

  const std::vector<const Obstacle *> &get_parking_space() const {
    return parking_spaces_;
  }

  const std::vector<const Obstacle *> &get_free_space_ground_lines() const {
    return free_space_ground_lines_;
  }

  const std::vector<const Obstacle *> &get_road_edges() const {
    return road_edges_;
  }

  bool get_reference_point_by_lon(
      double s, ReferencePathPoint &reference_path_point) const;
  bool transform_trajectory_points(TrajectoryPoints &trajectory_points) const;
  bool transform_trajectory_point(TrajectoryPoint &trajectory_point) const;

  bool get_polygon_at_time(
      const int id, bool is_use_recurrence, const int relative_time,
      planning_math::Polygon2d
          &obstacle_polygon);  // relative_time 为时间相对时间 * 10

  const ReferencePathCurveInfo& GetReferencePathCurveInfo() const {
    return ref_path_curve_info_;
  }

  const pnc::mathlib::spline& GetRawCurveSpline() const {
    return raw_k_s_spline_;
  }

  const bool GetIsSmoothed() const { return is_smoothed_; }

  const bool GetIsConstructionScene() const { return is_construction_scene_ref_path_; }

  const ReferencePathSource& GetReferencePathSource() const {
    return ref_path_source_;
  }

 public:
  // 用在sort函数中，应使用全局量或Lambda函数
  inline static bool compare_obstacle_s_descend(
      const std::shared_ptr<FrenetObstacle> o1,
      const std::shared_ptr<FrenetObstacle> o2) {
    return (o1->frenet_s() > o2->frenet_s());
  }

  inline static bool compare_obstacle_s_ascend(
      const std::shared_ptr<FrenetObstacle> o1,
      const std::shared_ptr<FrenetObstacle> o2) {
    return (o1->frenet_s() < o2->frenet_s());
  }

 protected:
  void init();
  void update_refpath_points(
      const ReferencePathPoints &raw_reference_path_points,
      const bool is_need_smooth);

  void update_refpath_points_in_hpp(
      const double ego_projection_length_in_reference_path,
      const ReferencePathPoints &raw_reference_path_points);

  bool get_reference_point_by_lon_from_raw_ref_path_points(
      double s, const ReferencePathPoints &raw_reference_path_point,
      ReferencePathPoint &reference_path_point);
  void discrete(double start, double end, double gap,
                std::vector<double> &output) {
    output.clear();
    for (double value = start; value < end; value += gap) {
      output.push_back(value);
    }
  }

  // smooth
  void InitReferencePathSmoother();

  bool UpdateReferencePath(const bool is_need_smooth);

  void CalculateValidLaneLineLength();

  bool CalculateRoadCurvature(
      std::vector<double> &x_vec,
      std::vector<double> &y_vec,
      std::vector<double> &s_vec);

  bool HandleRoadCurvature(const double init_s);

  bool SmoothReferencePath(
      const double init_s,
      const pnc::mathlib::spline &x_s_spline,
      const pnc::mathlib::spline &y_s_spline,
      std::vector<planning_math::PathPoint> &smoothed_path_points);

  bool PartionedRefPoints(
      const double init_s,
      const pnc::mathlib::spline &x_s_spline,
      const pnc::mathlib::spline &y_s_spline,
      double &behind_partition_length,
      double &ahead_partition_length,
      std::vector<double> &refined_x_vec,
      std::vector<double> &refined_y_vec);

  bool SamplingRefPoints(
      const double init_s,
      const pnc::mathlib::spline &x_s_spline,
      const pnc::mathlib::spline &y_s_spline,
      double &behind_partition_length,
      double &ahead_partition_length,
      std::vector<double> &refined_x_vec,
      std::vector<double> &refined_y_vec);

  bool HandleInputData(
      const std::vector<double> &refined_x_vec,
      const std::vector<double> &refined_y_vec,
      const std::pair<double, double> &init_point,
      std::vector<double> &bounds,
      std::vector<std::pair<double, double>> &raw_points_vec);

  void SetSmoothBounds(
      const double bound_val, std::vector<double> &bounds);

  bool HandleOutputData(
      const double ahead_partition_length,
      const std::pair<double, double> &init_point);

  bool ForwardExtendedRefPoints(const double ahead_partition_length);

  void StraightExtendedRefPoints(const double extend_length);

  void ClothoidExtendedRefPoints(const double extend_length,
                                 const double target_curv);

  bool GenerateDenseRefPathPoints(
      const double behind_partition_length,
      const pnc::mathlib::spline &x_s_spline,
      const pnc::mathlib::spline &y_s_spline,
      std::vector<planning_math::PathPoint> &smoothed_path_points);

  bool BackwardExtendedRefPoints(
      const double behind_partition_length,
      const pnc::mathlib::spline &x_s_spline,
      const pnc::mathlib::spline &y_s_spline,
      std::vector<planning_math::PathPoint> &smoothed_path_points);

  bool UpdateReferencePathInfo(
      std::vector<planning_math::PathPoint> &smoothed_path_points);

  void SaveSmootherDebugInfo();

 protected:
  bool valid_;
  bool is_construction_scene_ref_path_ = false;
  ReferencePathPoint raw_start_point_;
  ReferencePathPoint raw_end_point_;
  ReferencePathPoints refined_ref_path_points_;
  // frenet coord system
  // FrenetCoordinateSystemParameters frenet_parameters_;
  // std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  // kd_path
  std::shared_ptr<planning_math::KDPath> frenet_coord_;

  // ego_state
  FrenetEgoState frenet_ego_state_;

  // obstacles
  // std::shared_ptr<ObstacleManager> obstacle_manager_;
  std::vector<std::shared_ptr<FrenetObstacle>> frenet_obstacles_;

  std::unordered_map<int, std::shared_ptr<FrenetObstacle>>
      frenet_obstacles_map_;

  std::vector<int> obstacles_in_lane_map_;

  std::vector<const Obstacle *> parking_spaces_;
  std::vector<const Obstacle *> free_space_ground_lines_;
  std::vector<const Obstacle *> road_edges_;

  // 第一个int为障碍物id;  第二个int为预测时间（t*10， Polygon2d为障碍物轮廓)
  std::unordered_map<int, std::unordered_map<int, planning_math::Polygon2d>>
      obstacles_frenet_infos_;
  std::unordered_map<int, std::unordered_map<int, planning_math::Polygon2d>>
      obstacles_frenet_infos_recurrence_;

  // session
  planning::framework::Session *session_;

  //  DISALLOW_COPY_AND_ASSIGN(ReferencePath);

  // smooth
  bool is_smoothed_ = false;
  bool is_enable_clothoid_extend_ = false;
  double valid_lane_line_length_ = 0.0;
  double sampling_gap_ = 2.0;
  double sampling_step_ = 2.0;
  std::vector<double> map_bound_val_{0.03, 0.05, 0.07};
  std::vector<double> map_weight_fem_pos_deviation_{1000.0, 10000.0, 100000.0};
  pnc::mathlib::spline raw_k_s_spline_;
  ReferencePathCurveInfo ref_path_curve_info_;
  SmootherData smooth_input_;
  SmootherData smooth_output_;
  planning::planning_math::FemPosDeviationSmoother ref_path_smoother_;
  planning::common::ReferencePathSmoothInfo ref_path_smoother_info_;

  ReferencePathSource ref_path_source_ = ReferencePathSource::FUSION_ROAD;
};

}  // namespace planning
