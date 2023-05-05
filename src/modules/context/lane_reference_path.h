#pragma once
#include "context/reference_path.h"

namespace planning {

class LaneReferencePath : public ReferencePath, public std::enable_shared_from_this<LaneReferencePath> {
 public:
  LaneReferencePath(int target_lane_virtual_id);
  virtual ~LaneReferencePath() = default;

  int get_lane_id() { return lane_virtual_id_; }

  virtual void update(planning::framework::Session *session);

  virtual bool is_obstacle_ignorable(const std::shared_ptr<FrenetObstacle> obstacle) override;
  void assign_obstacles_to_lane();
  void cal_current_leadone_leadtwo_to_ego();
  bool is_potential_current_leadone_leadtwo_to_ego(const std::shared_ptr<FrenetObstacle> frenet_obstacle);

  const std::vector<int> &get_lane_obstacles() const {
    return lane_obstacles_;
  }

  int get_lane_leadone_obstacle() const {
    return lane_leadone_obstacle_;
  }

  int get_lane_leadtwo_obstacle() const {
    return lane_leadtwo_obstacle_;
  }

  int get_current_leadone_obstacle_to_ego() const {
    return current_leadone_obstacle_to_ego_;
  }

  int get_current_leadtwo_obstacle_to_ego() const {
    return current_leadtwo_obstacle_to_ego_;
  }
 private:
  virtual void update_obstacles();
  bool get_points_by_lane_id(int target_lane_virtual_id, ReferencePathPoints &points);

  int lane_virtual_id_ = 0;
  std::vector<int> lane_obstacles_;
  int lane_leadone_obstacle_;
  int lane_leadtwo_obstacle_;
  int current_leadone_obstacle_to_ego_{-1};
  int current_leadtwo_obstacle_to_ego_{-1};
};

}  // namespace planning