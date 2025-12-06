#pragma once

#include "tasks/task.h"

namespace planning {

enum class NarrowSpaceState {
  NO_NARROW_SPACE = 0,
  APPROACH_NARROW_SPACE = 1,
  ENTERING_NARROW_SPACE = 2,
  IN_NARROW_SPACE = 3,
  EXITING_NARROW_SPACE = 4,
};

enum class NarrowSpaceStatus {
  UNKNOWN = 0,
  NORMAL = 1,
  NARROW = 2,
  WIDE = 3,
  OBLIQUE = 4,
};

class NarrowSpaceDecider : public Task {
 public:
  NarrowSpaceDecider(const EgoPlanningConfigBuilder *config_builder,
                       framework::Session *session);
  virtual ~NarrowSpaceDecider() = default;

  bool Execute() override;

 private:
  void ResetNarrowSpace();

  void InitInfo();

  bool HandleNarrowSpaceData();

  bool ExtractNarrowSpaceOutline(
      std::map<LatObstacleDecisionType, std::map<double, double>>& outline);

  bool GenerateNarrowSpaceBoundary(
      const std::map<LatObstacleDecisionType, std::map<double, double>>& outline);

  bool DenseBoundary(
      std::vector<double>& s_vec, std::vector<double>& l_vec);

  bool CalculateNarrowSpaceInfo(
      const std::vector<double>& left_s_vec, const std::vector<double>& left_l_vec,
      const std::vector<double>& right_s_vec, const std::vector<double>& right_l_vec);

  void IsExistObstacleInNarrowSpace(
      const std::map<LatObstacleDecisionType, std::map<double, double>>& outline);

  void UpdateNarrowSpaceState();

  void UpdateNarrowSpaceStatus();

  void CalculateDrivingDistance();

  void GenerateNarrowSpaceOutput();

  void LogNarrowSpaceCorners();

 private:
  NarrowSpaceDeciderConfig config_;
  NarrowSpaceState narrow_space_state_;
  NarrowSpaceStatus narrow_space_status_;
  NarrowSpaceStatus last_narrow_space_status_;
  bool is_exist_narrow_space_;
  bool is_passable_;
  bool is_in_function_;
  double distance_to_narrow_space_;
  double rotate_narrow_space_width_;
  double narrow_space_width_;
  double narrow_space_length_;
  // double narrow_space_height_;
  double narrow_space_direction_angle_;
  double accumulated_driving_distance_;
  double max_narrow_space_width_;
  double min_narrow_space_width_;
  double width_slack_factor_;
  double angle_slack_factor_;
  std::pair<double, double> vehicle_s_range_;
  std::pair<double, double> narrow_space_s_range_;
  pnc::mathlib::spline left_boundary_spline_;   // s, l
  pnc::mathlib::spline right_boundary_spline_;  // s, l
  Point2D last_ego_cart_;
};

}  // namespace planning