#pragma once

#include <Eigen/Core>

#include "pose2d.h"

namespace planning {

// todo, unify all transform related implements for parking and driving, or
// other modules.
class Transform2d {
 public:
  Transform2d() = default;
  Transform2d(const Transform2d &) = default;
  Transform2d &operator=(const Transform2d &) = default;

  Transform2d(const Pose2D &pose) : base_pose_(pose) {}

  /**
   *
   * base: east-north frame
   *
   * local: right up frame,  frame heading is y axis, and right side is x axis
   */
  static void RUFLocalPoseToGlobal(Pose2D *global_pose,
                                   const Pose2D &local_pose,
                                   const Pose2D &base_pose);

  void RUFLocalPoseToGlobal(Pose2D *global_pose, const Pose2D &local_pose);

  static void GlobalPoseToRUFLocal(Pose2D *local_pose,
                                   const Pose2D &global_pose,
                                   const Pose2D &base_pose);

  void GlobalPoseToRUFLocal(Pose2D *local_pose, const Pose2D &global_pose);

  /**
   *
   * base: east-north frame
   *
   * local: up left frame,  frame heading is x axis, and left side is y axis
   */
  static void ULFLocalPoseToGlobal(Pose2D *global_pose,
                                   const Pose2D &local_pose,
                                   const Pose2D &base_pose);

  void ULFLocalPoseToGlobal(Pose2D *global_pose, const Pose2D &local_pose);

  void ULFLocalPointToGlobal(Position2D *global_pose,
                             const Position2D &local_pose) const;

  void ULFLocalPointToGlobal(Eigen::Vector2d &global_pose,
                             const Eigen::Vector2d &local_pose);

  static void GlobalPoseToULFLocal(Pose2D *local_pose,
                                   const Pose2D &global_pose,
                                   const Pose2D &base_pose);

  void GlobalPoseToULFLocal(Pose2D *local_pose, const Pose2D &global_pose);

  void GlobalPointToULFLocal(Pose2D *local_pose, const Pose2D &global_pose);

  void SetBasePose(const Pose2D &base_pose);

  void SetBasePose(const Pose2D &base_pose, const double sin_theta,
                   const double cos_theta);

  const Pose2D &GetConstBasePose() const { return base_pose_; }

  Pose2D *GetMutableBasePose() { return &base_pose_; }

  const double GetSinTheta() const { return sin_theta_; }

  const double GetCosTheta() const { return cos_theta_; }

 private:
  Pose2D base_pose_;
  double sin_theta_;
  double cos_theta_;
};

}  // namespace planning