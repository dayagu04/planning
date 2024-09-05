#pragma once
#include "reference_path.h"

namespace planning {

class LaneReferencePath
    : public ReferencePath,
      public std::enable_shared_from_this<LaneReferencePath> {
 public:
  LaneReferencePath(int target_lane_virtual_id);
  virtual ~LaneReferencePath() = default;

  int get_lane_id() { return lane_virtual_id_; }

  virtual void update(planning::framework::Session *session);

  virtual bool is_obstacle_ignorable(
      const std::shared_ptr<FrenetObstacle> obstacle) override;

  // 分配、排序车道内障碍物，仅保留lead one，lead two
  void assign_obstacles_to_lane();
  void cal_current_leadone_leadtwo_to_ego();
  bool is_potential_current_leadone_leadtwo_to_ego(
      const std::shared_ptr<FrenetObstacle> frenet_obstacle);

  /**
   * @brief 获取车道内障碍物：lead one & lead two以及后方obstacles的id
   */
  const std::vector<int> &get_lane_obstacles_ids() const {
    return lane_obstacles_id_;
  }

  int get_lane_leadone_obstacle() const { return lane_leadone_obstacle_; }

  int get_lane_leadtwo_obstacle() const { return lane_leadtwo_obstacle_; }

  int get_current_leadone_obstacle_to_ego() const {
    return current_leadone_obstacle_to_ego_;
  }

  int get_current_leadtwo_obstacle_to_ego() const {
    return current_leadtwo_obstacle_to_ego_;
  }
  bool IsObstacleOn(std::shared_ptr<FrenetObstacle> frenet_obstacle);
  double get_origin_reference_path_length() const {
    return origin_reference_path_length_;
  }

 private:
  void update_refined_lane_points();
  virtual void update_obstacles() override;
  ReferencePathPoint CalculateExtendedReferencePathPoint(
      const ReferencePathPoint &p1, const ReferencePathPoint &p2,
      const double length) const;
  double CalculateEgoProjectionDistanceInReferencePath(
      const ReferencePathPoints &ref_path_points) const;
  bool get_ref_points(ReferencePathPoints &points);

  int lane_virtual_id_ = 0;
  std::vector<int> lane_obstacles_id_;
  int lane_leadone_obstacle_;
  int lane_leadtwo_obstacle_;
  int current_leadone_obstacle_to_ego_{-1};
  int current_leadtwo_obstacle_to_ego_{-1};
  double origin_reference_path_length_ = -1;
};

}  // namespace planning