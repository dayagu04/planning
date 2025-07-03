#include "transform2d.h"

#include "pose2d.h"

namespace planning {

void Transform2d::RUFLocalPoseToGlobal(Position2D *global_pose,
                                       const Position2D &local_pose) const {
  if (global_pose == nullptr) {
    return;
  }

  double lx, ly;
  lx = local_pose.x;
  ly = local_pose.y;

  global_pose->x = base_pose_.x;
  global_pose->x += sin_theta_ * lx + cos_theta_ * ly;

  global_pose->y = base_pose_.y;
  global_pose->y += sin_theta_ * ly - cos_theta_ * lx;

  return;
}

void Transform2d::RUFLocalPoseToGlobal(Pose2D *global_pose,
                                       const Pose2D &local_pose,
                                       const Pose2D &base_pose) {
  if (global_pose == nullptr) {
    return;
  }

  double lx, ly, theta;

  lx = local_pose.x;
  ly = local_pose.y;
  theta = base_pose.theta;

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  global_pose->x = base_pose.x;
  global_pose->x += sin_theta * lx + cos_theta * ly;

  global_pose->y = base_pose.y;
  global_pose->y += sin_theta * ly - cos_theta * lx;

  global_pose->theta = local_pose.theta + theta - M_PI / 2.0;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PI);

  return;
}

void Transform2d::RUFLocalPoseToGlobal(Pose2D *global_pose,
                                       const Pose2D &local_pose) {
  if (global_pose == nullptr) {
    return;
  }

  double lx, ly;
  lx = local_pose.x;
  ly = local_pose.y;

  global_pose->x = base_pose_.x;
  global_pose->x += sin_theta_ * lx + cos_theta_ * ly;

  global_pose->y = base_pose_.y;
  global_pose->y += sin_theta_ * ly - cos_theta_ * lx;

  global_pose->theta = local_pose.theta + base_pose_.theta - M_PI / 2.0;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PI);

  return;
}

void Transform2d::GlobalPoseToRUFLocal(Pose2D *local_pose,
                                       const Pose2D &global_pose,
                                       const Pose2D &base_pose) {
  if (local_pose == nullptr) {
    return;
  }

  double dx, dy, theta;

  dx = global_pose.x - base_pose.x;
  dy = global_pose.y - base_pose.y;
  theta = base_pose.theta;

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  local_pose->x = sin_theta * dx - cos_theta * dy;
  local_pose->y = cos_theta * dx + sin_theta * dy;

  local_pose->theta = M_PI / 2.0 + global_pose.theta - theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PI);

  return;
}

void Transform2d::GlobalPoseToRUFLocal(Pose2D *local_pose,
                                       const Pose2D &global_pose) {
  if (local_pose == nullptr) {
    return;
  }

  double dx, dy, theta;

  dx = global_pose.x - base_pose_.x;
  dy = global_pose.y - base_pose_.y;
  theta = base_pose_.theta;

  local_pose->x = sin_theta_ * dx - cos_theta_ * dy;
  local_pose->y = cos_theta_ * dx + sin_theta_ * dy;

  local_pose->theta = M_PI / 2.0 + global_pose.theta - theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PI);

  return;
}

/**
 *
 * base: east-north frame
 *
 * local: up left frame,  frame heading is x axis, and left side is y axis
 */
void Transform2d::ULFLocalPoseToGlobal(Pose2D *global_pose,
                                       const Pose2D &local_pose,
                                       const Pose2D &base_pose) {
  if (global_pose == nullptr) {
    return;
  }

  double sin_theta = std::sin(base_pose.theta);
  double cos_theta = std::cos(base_pose.theta);

  double tmp_x = local_pose.x * cos_theta - local_pose.y * sin_theta;
  double tmp_y = local_pose.x * sin_theta + local_pose.y * cos_theta;

  global_pose->x = tmp_x + base_pose.x;
  global_pose->y = tmp_y + base_pose.y;

  global_pose->theta = local_pose.theta + base_pose.theta;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PI);

  return;
}

void Transform2d::ULFLocalPoseToGlobal(Pose2D *global_pose,
                                       const Pose2D &local_pose) {
  if (global_pose == nullptr) {
    return;
  }

  double tmp_x = local_pose.x * cos_theta_ - local_pose.y * sin_theta_;
  double tmp_y = local_pose.x * sin_theta_ + local_pose.y * cos_theta_;

  global_pose->x = tmp_x + base_pose_.x;
  global_pose->y = tmp_y + base_pose_.y;

  global_pose->theta = local_pose.theta + base_pose_.theta;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PI);

  return;
}

void Transform2d::ULFLocalPointToGlobal(Position2D *global_pose,
                                        const Position2D &local_pose) const {
  if (global_pose == nullptr) {
    return;
  }

  double tmp_x = local_pose.x * cos_theta_ - local_pose.y * sin_theta_;
  double tmp_y = local_pose.x * sin_theta_ + local_pose.y * cos_theta_;

  global_pose->x = tmp_x + base_pose_.x;
  global_pose->y = tmp_y + base_pose_.y;

  return;
}

void Transform2d::ULFLocalPointToGlobal(Eigen::Vector2d &global_pose,
                                        const Eigen::Vector2d &local_pose) {
  double tmp_x = local_pose.x() * cos_theta_ - local_pose.y() * sin_theta_;
  double tmp_y = local_pose.x() * sin_theta_ + local_pose.y() * cos_theta_;

  global_pose[0] = tmp_x + base_pose_.x;
  global_pose[1] = tmp_y + base_pose_.y;

  return;
}

void Transform2d::GlobalPoseToULFLocal(Pose2D *local_pose,
                                       const Pose2D &global_pose,
                                       const Pose2D &base_pose) {
  if (local_pose == nullptr) {
    return;
  }

  double sin_theta = std::sin(base_pose.theta);
  double cos_theta = std::cos(base_pose.theta);

  double tmp_x = global_pose.x - base_pose.x;
  double tmp_y = global_pose.y - base_pose.y;

  local_pose->x = tmp_x * cos_theta + tmp_y * sin_theta;
  local_pose->y = tmp_y * cos_theta - tmp_x * sin_theta;

  local_pose->theta = global_pose.theta - base_pose.theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PI);

  return;
};

void Transform2d::GlobalPoseToULFLocal(Pose2D *local_pose,
                                       const Pose2D &global_pose) {
  if (local_pose == nullptr) {
    return;
  }

  double tmp_x = global_pose.x - base_pose_.x;
  double tmp_y = global_pose.y - base_pose_.y;

  local_pose->x = tmp_x * cos_theta_ + tmp_y * sin_theta_;
  local_pose->y = tmp_y * cos_theta_ - tmp_x * sin_theta_;

  local_pose->theta = global_pose.theta - base_pose_.theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PI);

  return;
};

void Transform2d::GlobalPointToULFLocal(Pose2D *local_pose,
                                        const Pose2D &global_pose) {
  if (local_pose == nullptr) {
    return;
  }

  double tmp_x = global_pose.x - base_pose_.x;
  double tmp_y = global_pose.y - base_pose_.y;

  local_pose->x = tmp_x * cos_theta_ + tmp_y * sin_theta_;
  local_pose->y = tmp_y * cos_theta_ - tmp_x * sin_theta_;

  return;
};

void Transform2d::SetBasePose(const Pose2D &base_pose) {
  sin_theta_ = std::sin(base_pose.theta);
  cos_theta_ = std::cos(base_pose.theta);

  base_pose_ = base_pose;

  return;
}

void Transform2d::SetBasePose(const Pose2D &base_pose, const double sin_theta,
                              const double cos_theta) {
  base_pose_ = base_pose;
  sin_theta_ = sin_theta;
  cos_theta_ = cos_theta;
  return;
}

void Transform2d::GlobalPointToULFLocal(const Position2D &global_pos,
                                        Position2D *local_pos) const {
  if (local_pos == nullptr) {
    return;
  }

  double tmp_x = global_pos.x - base_pose_.x;
  double tmp_y = global_pos.y - base_pose_.y;

  local_pos->x = tmp_x * cos_theta_ + tmp_y * sin_theta_;
  local_pos->y = tmp_y * cos_theta_ - tmp_x * sin_theta_;
  return;
}

void Transform2f::RUFLocalPoseToGlobal(Pose2f *global_pose,
                                       const Pose2f &local_pose,
                                       const Pose2f &base_pose) {
  if (global_pose == nullptr) {
    return;
  }

  float lx, ly, theta;

  lx = local_pose.x;
  ly = local_pose.y;
  theta = base_pose.theta;

  float sin_theta = std::sin(theta);
  float cos_theta = std::cos(theta);

  global_pose->x = base_pose.x;
  global_pose->x += sin_theta * lx + cos_theta * ly;

  global_pose->y = base_pose.y;
  global_pose->y += sin_theta * ly - cos_theta * lx;

  global_pose->theta = local_pose.theta + theta - M_PI / 2.0;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PIf32);

  return;
}

void Transform2f::RUFLocalPoseToGlobal(Pose2f *global_pose,
                                       const Pose2f &local_pose) {
  if (global_pose == nullptr) {
    return;
  }

  float lx, ly;
  lx = local_pose.x;
  ly = local_pose.y;

  global_pose->x = base_pose_.x;
  global_pose->x += sin_theta_ * lx + cos_theta_ * ly;

  global_pose->y = base_pose_.y;
  global_pose->y += sin_theta_ * ly - cos_theta_ * lx;

  global_pose->theta = local_pose.theta + base_pose_.theta - M_PI / 2.0;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PIf32);

  return;
}

void Transform2f::GlobalPoseToRUFLocal(Pose2f *local_pose,
                                       const Pose2f &global_pose,
                                       const Pose2f &base_pose) {
  if (local_pose == nullptr) {
    return;
  }

  float dx, dy, theta;

  dx = global_pose.x - base_pose.x;
  dy = global_pose.y - base_pose.y;
  theta = base_pose.theta;

  float sin_theta = std::sin(theta);
  float cos_theta = std::cos(theta);

  local_pose->x = sin_theta * dx - cos_theta * dy;
  local_pose->y = cos_theta * dx + sin_theta * dy;

  local_pose->theta = M_PI / 2.0 + global_pose.theta - theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PIf32);

  return;
}

void Transform2f::GlobalPoseToRUFLocal(Pose2f *local_pose,
                                       const Pose2f &global_pose) {
  if (local_pose == nullptr) {
    return;
  }

  float dx, dy, theta;

  dx = global_pose.x - base_pose_.x;
  dy = global_pose.y - base_pose_.y;
  theta = base_pose_.theta;

  local_pose->x = sin_theta_ * dx - cos_theta_ * dy;
  local_pose->y = cos_theta_ * dx + sin_theta_ * dy;

  local_pose->theta = M_PI / 2.0 + global_pose.theta - theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PIf32);

  return;
}

/**
 *
 * base: east-north frame
 *
 * local: up left frame,  frame heading is x axis, and left side is y axis
 */
void Transform2f::ULFLocalPoseToGlobal(Pose2f *global_pose,
                                       const Pose2f &local_pose,
                                       const Pose2f &base_pose) {
  if (global_pose == nullptr) {
    return;
  }

  float sin_theta = std::sin(base_pose.theta);
  float cos_theta = std::cos(base_pose.theta);

  float tmp_x = local_pose.x * cos_theta - local_pose.y * sin_theta;
  float tmp_y = local_pose.x * sin_theta + local_pose.y * cos_theta;

  global_pose->x = tmp_x + base_pose.x;
  global_pose->y = tmp_y + base_pose.y;

  global_pose->theta = local_pose.theta + base_pose.theta;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PIf32);

  return;
}

void Transform2f::ULFLocalPoseToGlobal(Pose2f *global_pose,
                                       const Pose2f &local_pose) {
  if (global_pose == nullptr) {
    return;
  }

  float tmp_x = local_pose.x * cos_theta_ - local_pose.y * sin_theta_;
  float tmp_y = local_pose.x * sin_theta_ + local_pose.y * cos_theta_;

  global_pose->x = tmp_x + base_pose_.x;
  global_pose->y = tmp_y + base_pose_.y;

  global_pose->theta = local_pose.theta + base_pose_.theta;
  global_pose->theta = IflyUnifyTheta(global_pose->theta, M_PIf32);

  return;
}

void Transform2f::ULFLocalPointToGlobal(Position2f *global_pose,
                                        const Position2f &local_pose) const {
  if (global_pose == nullptr) {
    return;
  }

  float tmp_x = local_pose.x * cos_theta_ - local_pose.y * sin_theta_;
  float tmp_y = local_pose.x * sin_theta_ + local_pose.y * cos_theta_;

  global_pose->x = tmp_x + base_pose_.x;
  global_pose->y = tmp_y + base_pose_.y;

  return;
}

void Transform2f::ULFLocalPointToGlobal(Eigen::Vector2f &global_pose,
                                        const Eigen::Vector2f &local_pose) {
  float tmp_x = local_pose.x() * cos_theta_ - local_pose.y() * sin_theta_;
  float tmp_y = local_pose.x() * sin_theta_ + local_pose.y() * cos_theta_;

  global_pose[0] = tmp_x + base_pose_.x;
  global_pose[1] = tmp_y + base_pose_.y;

  return;
}

void Transform2f::GlobalPoseToULFLocal(Pose2f *local_pose,
                                       const Pose2f &global_pose,
                                       const Pose2f &base_pose) {
  if (local_pose == nullptr) {
    return;
  }

  float sin_theta = std::sin(base_pose.theta);
  float cos_theta = std::cos(base_pose.theta);

  float tmp_x = global_pose.x - base_pose.x;
  float tmp_y = global_pose.y - base_pose.y;

  local_pose->x = tmp_x * cos_theta + tmp_y * sin_theta;
  local_pose->y = tmp_y * cos_theta - tmp_x * sin_theta;

  local_pose->theta = global_pose.theta - base_pose.theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PIf32);

  return;
};

void Transform2f::GlobalPoseToULFLocal(Pose2f *local_pose,
                                       const Pose2f &global_pose) {
  if (local_pose == nullptr) {
    return;
  }

  float tmp_x = global_pose.x - base_pose_.x;
  float tmp_y = global_pose.y - base_pose_.y;

  local_pose->x = tmp_x * cos_theta_ + tmp_y * sin_theta_;
  local_pose->y = tmp_y * cos_theta_ - tmp_x * sin_theta_;

  local_pose->theta = global_pose.theta - base_pose_.theta;
  local_pose->theta = IflyUnifyTheta(local_pose->theta, M_PIf32);

  return;
};

void Transform2f::GlobalPointToULFLocal(Pose2f *local_pose,
                                        const Pose2f &global_pose) {
  if (local_pose == nullptr) {
    return;
  }

  float tmp_x = global_pose.x - base_pose_.x;
  float tmp_y = global_pose.y - base_pose_.y;

  local_pose->x = tmp_x * cos_theta_ + tmp_y * sin_theta_;
  local_pose->y = tmp_y * cos_theta_ - tmp_x * sin_theta_;

  return;
};

void Transform2f::SetBasePose(const Pose2f &base_pose) {
  sin_theta_ = std::sin(base_pose.theta);
  cos_theta_ = std::cos(base_pose.theta);

  base_pose_ = base_pose;

  return;
}

void Transform2f::SetBasePose(const Pose2f &base_pose, const float sin_theta,
                              const float cos_theta) {
  base_pose_ = base_pose;
  sin_theta_ = sin_theta;
  cos_theta_ = cos_theta;
  return;
}

void Transform2f::GlobalPointToULFLocal(const Position2f &global_pos,
                                        Position2f *local_pos) const {
  if (local_pos == nullptr) {
    return;
  }

  float tmp_x = global_pos.x - base_pose_.x;
  float tmp_y = global_pos.y - base_pose_.y;

  local_pos->x = tmp_x * cos_theta_ + tmp_y * sin_theta_;
  local_pos->y = tmp_y * cos_theta_ - tmp_x * sin_theta_;
  return;
}

void Transform2f::GetTransform2d(Transform2d *tf) {
  tf->SetBasePose(Pose2D(base_pose_.x, base_pose_.y, base_pose_.theta),
                  sin_theta_, cos_theta_);

  return;
}

}  // namespace planning