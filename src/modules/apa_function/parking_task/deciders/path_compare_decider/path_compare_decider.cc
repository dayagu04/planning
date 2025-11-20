#include "path_compare_decider.h"

#include <cmath>
#include <cstddef>

#include "apa_context.h"
#include "apa_slot.h"
#include "common_math.h"
#include "curve_node.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "pose2d.h"

namespace planning {
namespace apa_planner {

#define DEBUG_CHALLENGING_SUCCESS (0)

const bool PathCompareDecider::Compare(const HybridAStarRequest* request,
                                       const CurveNode* best_node,
                                       const CurveNode* challenging_node) {
  if (request == nullptr || best_node == nullptr ||
      challenging_node == nullptr) {
    return false;
  }
  request_ = request;
  best_node_ = best_node;
  challenging_node_ = challenging_node;

  if (best_node_->GetCurvePath().segment_size < 1) {
#if DEBUG_CHALLENGING_SUCCESS
    ILOG_INFO << "best_node path segment size is less than 1";
#endif
    return true;
  }

  if (challenging_node_->GetGearSwitchNum() == 0 &&
      best_node_->GetGearSwitchNum() == 0) {
    return false;
  }

  if (request->scenario_type ==
      ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN) {
    return CompareForPerpendicularTailIn();
  } else if (request->scenario_type ==
             ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_OUT) {
    return CompareForPerpendicularTailOut();
  } else {
    return false;
  }
}

const bool PathCompareDecider::CompareForPerpendicularTailIn() {
  // if (challenging_node_->GetFCost() > best_node_->GetFCost() + 10.0) {
  //   return false;
  // }

  //   if (challenging_node_->GetGearSwitchNum() <
  //   best_node_->GetGearSwitchNum()) {
  // #if DEBUG_CHALLENGING_SUCCESS
  //     ILOG_INFO << "challenging  node gear change num is less than
  //     best_node";
  // #endif
  //     return true;
  //   }

  const int challenging_gear_change_num = challenging_node_->GetGearSwitchNum();
  const int best_gear_change_num = best_node_->GetGearSwitchNum();
  if (challenging_gear_change_num > best_gear_change_num + 2) {
    return false;
  } else if (challenging_gear_change_num < best_gear_change_num - 2) {
    return true;
  }

  const AstarPathGear challenging_cur_gear = challenging_node_->GetCurGear();
  const AstarPathGear best_cur_gear = best_node_->GetCurGear();

  const double challenging_cur_gear_length =
      challenging_node_->GetCurGearLength();
  const double best_cur_gear_length = best_node_->GetCurGearLength();

  const auto& challenging_gear_switch_pose =
      challenging_node_->GetGearSwitchPose();
  const auto& best_gear_switch_pose = best_node_->GetGearSwitchPose();

  const SlotCoord& slot_coord =
      request_->ego_info_under_slot.slot.processed_corner_coord_local_;

  ObsToPathDistRelativeSlot challenging_obs_info =
      challenging_node_->GetObsDistRelativeSlot();
  ObsToPathDistRelativeSlot best_obs_info =
      best_node_->GetObsDistRelativeSlot();
  if (best_obs_info.obs_dist_out_slot_turn < 0.4 &&
      challenging_obs_info.obs_dist_out_slot_turn >
          best_obs_info.obs_dist_out_slot_turn + 0.05) {
    return true;
  } else if (challenging_obs_info.obs_dist_out_slot_turn < 0.4 &&
             best_obs_info.obs_dist_out_slot_turn >
                 challenging_obs_info.obs_dist_out_slot_turn + 0.05) {
    return false;
  }

  const double challenging_gear_switch_pose_dist_to_slot =
      common_math::CalTwoPosDist(challenging_gear_switch_pose.pos,
                                 slot_coord.pt_center);
  const double best_gear_switch_pose_dist_to_slot = common_math::CalTwoPosDist(
      best_gear_switch_pose.pos, slot_coord.pt_center);
  if (best_gear_switch_pose_dist_to_slot > 8.28 &&
      challenging_gear_switch_pose_dist_to_slot <
          best_gear_switch_pose_dist_to_slot - 0.368) {
    return true;
  } else if (challenging_gear_switch_pose_dist_to_slot > 8.28 &&
             best_gear_switch_pose_dist_to_slot <
                 challenging_gear_switch_pose_dist_to_slot - 0.368) {
    return false;
  }

  const AstarPathGear ref_gear = request_->inital_action_request.ref_gear;
  if (IsGearDifferent(best_cur_gear, ref_gear) &&
      !IsGearDifferent(challenging_cur_gear, ref_gear)) {
#if DEBUG_CHALLENGING_SUCCESS
    ILOG_INFO << "best node is not in ref gear, but challenging node is";
#endif
    return true;
  }

  const ApaSlot& slot = request_->ego_info_under_slot.slot;
  const geometry_lib::PathPoint& ego_pose =
      request_->ego_info_under_slot.cur_pose;
  const bool adjust_pose_inslot =
      (ego_pose.pos.x() < slot.processed_corner_coord_local_.pt_01_mid.x() &&
       request_->inital_action_request.ref_gear == AstarPathGear::DRIVE);
  if (adjust_pose_inslot) {
    if (best_gear_switch_pose.pos.x() <
            slot.processed_corner_coord_local_.pt_01_mid.x() &&
        best_gear_switch_pose.pos.x() < challenging_gear_switch_pose.pos.x()) {
#if DEBUG_CHALLENGING_SUCCESS
      ILOG_INFO
          << "adjust_pose_inslot is true, best_gear_switch_pose x is less than "
             "pt_01_mid x and less than challenging_gear_switch_pose x";
#endif
      return true;
    }
  }

  const double x_upper = 11.68;
  const double x_lower = 2.68;
  if (best_gear_switch_pose.GetX() > x_upper) {
    if (challenging_gear_switch_pose.GetX() < best_gear_switch_pose.GetX() &&
        challenging_gear_switch_pose.GetX() > x_lower) {
#if DEBUG_CHALLENGING_SUCCESS
      ILOG_INFO << "best_gear_switch_pose x is greater than x_upper and "
                   "challenging_gear_switch_pose x is less than "
                   "best_gear_switch_pose x and greater than x_lower";
#endif
      return true;
    }
  }
  if (best_gear_switch_pose.GetX() < x_lower) {
    if (challenging_gear_switch_pose.GetX() > best_gear_switch_pose.GetX() &&
        challenging_gear_switch_pose.GetX() < x_upper) {
#if DEBUG_CHALLENGING_SUCCESS
      ILOG_INFO << "best_gear_switch_pose x is less than x_lower and "
                   "challenging_gear_switch_pose x is greater than "
                   "best_gear_switch_pose x and less than x_upper";
#endif
      return true;
    }
  }

  const double challenging_dist = CalcHeuristicPointDistance(
      challenging_gear_switch_pose.pos, challenging_gear_switch_pose.GetTheta(),
      slot.processed_corner_coord_local_.pt_01_mid);

  const double best_dist = CalcHeuristicPointDistance(
      best_gear_switch_pose.pos, best_gear_switch_pose.GetTheta(),
      slot.processed_corner_coord_local_.pt_01_mid);

  if (challenging_dist < best_dist - 0.05) {
#if DEBUG_CHALLENGING_SUCCESS
    ILOG_INFO
        << "challenging dist is smaller than best dist - 0.05, challenging "
           "dist: "
        << challenging_dist << ", best dist: " << best_dist;
#endif
    return true;
  }

  if (challenging_dist < best_dist + 1.0) {
    const double challenging_heading_err =
        std::fabs(challenging_gear_switch_pose.GetTheta());
    const double best_heading_err = std::fabs(best_gear_switch_pose.GetTheta());
    if (std::fabs(challenging_heading_err - best_heading_err) * kRad2Deg <
        2.68) {
      if (challenging_cur_gear_length < best_cur_gear_length) {
#if DEBUG_CHALLENGING_SUCCESS
        ILOG_INFO << "challenging_heading_err and best heading err are very "
                     "close and "
                     "challenging_cur_gear_length is smaller than best cur "
                     "gear length";
#endif
        return true;
      }
    } else {
      if (challenging_heading_err < best_heading_err) {
#if DEBUG_CHALLENGING_SUCCESS
        ILOG_INFO
            << "challenging_heading_err is much smaller than best_heading_err";
#endif
        return true;
      }
    }
  }

  return false;
}

const bool PathCompareDecider::CompareForPerpendicularTailOut() {
  return false;
}

const float PathCompareDecider::CalcHeuristicPointDistance(
    const Eigen::Vector2d& pos_a, const double heading_a,
    const Eigen::Vector2d& pos_b) {
  const Eigen::Vector2d heading_vec(std::cos(heading_a), std::sin(heading_a));
  const Eigen::Vector2d slot_line(pos_b.x() - pos_a.x(), pos_b.y() - pos_a.y());
  return std::fabs(heading_vec.x() * slot_line.y() -
                   heading_vec.y() * slot_line.x());
}

const float PathCompareDecider::CalcHeuristicPointDistance(
    const Eigen::Vector2f& pos_a, const float heading_a,
    const Eigen::Vector2d& pos_b) {
  const Eigen::Vector2f heading_vec(std::cos(heading_a), std::sin(heading_a));
  const Eigen::Vector2f slot_line(pos_b.x() - pos_a.x(), pos_b.y() - pos_a.y());
  return std::fabs(heading_vec.x() * slot_line.y() -
                   heading_vec.y() * slot_line.x());
}

}  // namespace apa_planner
}  // namespace planning