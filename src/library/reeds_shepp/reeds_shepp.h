
#pragma once

#include "pose2d.h"
#include "./../hybrid_astar_lib/hybrid_astar_common.h"
#include "rs_path_request.h"

namespace planning {

#define is_veh_move_forward(mov_dir) \
  ((AstarPathGear::drive == mov_dir) ? true : false)

#define is_veh_move_backward(mov_dir) \
  ((AstarPathGear::reverse == mov_dir) ? true : false)

#define get_signed_segment_dir(signed_length)                       \
  ((1 == ifly_sign(signed_length))                                  \
       ? AstarPathGear::drive                                       \
       : ((-1 == ifly_sign(signed_length)) ? AstarPathGear::reverse \
                                           : AstarPathGear::none))

enum RSPathSteer {
  RS_NOP = 0,
  RS_LEFT = 1,
  RS_STRAIGHT = 2,
  RS_RIGHT = 3,
  RS_STEER_MAX
};

struct RSPoint : public Pose2D {
  // left turn is positive
  double kappa;

  AstarPathGear dir;
};

#define MAX_RS_PATH_NUM (5)

struct RSPathParam {
  double t;
  double u;
  double v;
  double w;
  double x;
  // if < 0, gear is reverse
  // if > 0, gear if drive
  // if ==0, dist is 0
  // length is real dist.
  double length[MAX_RS_PATH_NUM];
  double total_length;
  const RSPathSteer *type;

  void Clear() {
    t = 1000.0;
    u = 1000.0;
    v = 1000.0;
    w = 1000.0;
    x = 1000.0;
  }
};

// base coordinate is rs start point. start point local system is up-left.
struct RSEndPoint {
  Pose2D pose;
  double sin_theta;
  double cos_theta;

  double sin_minus_theta;
  double cos_minus_theta;
};

struct RSSegmentKappaParam {
  /* signed arc length */
  double length;
  // left is postive, kappa
  double kappa;

  RSPathSteer steer_type;
};

struct RSAnchorPoints {
  // record points if kappa change or gear change
  RSPoint points[16];
  int size;
};

struct RSPathKappaParam {
  RSSegmentKappaParam path_kappa[MAX_RS_PATH_NUM];
  int size;

  int gear_change_num;
};

struct RSPathInfo {
  Pose2D start;
  Pose2D end;
  double min_radius;
  RSPathParam path;

  bool is_path_valid;
  RSPathKappaParam kappa_param;
};

// todo
class RSPathGenerator {
 public:
  RSPathGenerator() = default;

 private:
};

int GetShortestRSPathParam(RSPathParam *path, const Pose2D *start_pose,
                           const Pose2D *goal_pose, double min_turn_radius,
                           double inverse_radius,
                           const RSPathRequestType request_type);

RSPathInfo *GetRSPathGlobalInfo(void);

/**
 * \brief Get gear switch num of the reeds shepp shortest path.
 *        The unit of start_pose.pos and goal_pose.pos should
 *        be the same (m or cell).
 *
 * \param[out]      gear_switch_num gear switch num of the rs shortest path;
 * \param[in]            start_pose start pose;
 * \param[in]             goal_pose goal pose;
 * \param[in]       min_turn_radius min turn radius of vehicle;
 * \param[in]      initial_pose_dir direction of initial pose;
 * \return 0 or error code.
 **/
int GetRSPathGearSwitchNum(int *gear_switch_num, const Pose2D *start_pose,
                           const Pose2D *goal_pose, double min_turn_radius,
                           AstarPathGear initial_pose_dir);

/**
 * \brief Get the shortest distance of start pose to goal pose by reeds shepp.
 *        The unit of start_pose.pos and goal_pose.pos should be
 *        the same (m or cell). The unit of out param distance
 *        keeps pace with pos.
 *
 * \param[out]          distance distance of start pose to goal pose;
 * \param[in]         start_pose start pose;
 * \param[in]          goal_pose goal pose;
 * \param[in]    min_turn_radius min turn radius of vehicle;
 * \return 0 or error code.
 **/
int GetRSPathDist(double *distance, const Pose2D *start_pose,
                  const Pose2D *goal_pose, double min_turn_radius);

int TestSCS(RSPathParam *path, const Pose2D *start_pose,
            const Pose2D *goal_pose, double min_turn_radius,
            double inverse_radius, const RSPathRequestType request_type);

}  // namespace planning
