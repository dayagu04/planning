#include "lane_change_lane_manager.h"
#include "config/basic_type.h"
#include "planning_context.h"

namespace planning {

LaneChangeLaneManager::LaneChangeLaneManager(
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    planning::framework::Session* session) {
  session_ = session;
  virtual_lane_mgr_ = virtual_lane_mgr;
  if (virtual_lane_mgr == nullptr) {
    ILOG_ERROR << "[LaneChangeLaneManager::constructor] empty pointer";
  } else {
    fix_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    origin_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    target_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  }
}

LaneChangeLaneManager::LaneChangeLaneManager(
    std::shared_ptr<LaneChangeLaneManager> source) {
  virtual_lane_mgr_ = source->virtual_lane_mgr_;
  session_ = source->session_;
  copy_lane_change_lanes(*source);
}

void LaneChangeLaneManager::assign_lc_lanes(int lane_virtual_id) {
  target_lane_virtual_id_ = lane_virtual_id;
  origin_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  fix_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
}

void LaneChangeLaneManager::reset_lc_lanes(
    const StateMachineLaneChangeStatus state_machine_lc_state) {
  if (state_machine_lc_state == kLaneChangeExecution) {
    const auto& origin_lane =
        virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
    if (origin_lane) {
      target_lane_virtual_id_ = origin_lane_virtual_id_;
      fix_lane_virtual_id_ = origin_lane_virtual_id_;
    } else {
      std::cout
          << "origin lane disappear ,set cur_lane as origin/target/fix_lane"
          << std::endl;
      target_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
      origin_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
      fix_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    }
  } else if (state_machine_lc_state == kLaneChangeCancel) {
    const auto& origin_lane =
        virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
    if (origin_lane) {
      // target_lane_virtual_id_ = origin_lane_virtual_id_;
      fix_lane_virtual_id_ = origin_lane_virtual_id_;
    } else {
      std::cout
          << "origin lane disappear ,set cur_lane as origin/target/fix_lane"
          << std::endl;
      target_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
      origin_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
      fix_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    }
  } else if (state_machine_lc_state == kLaneChangeHold) {
    const auto& origin_lane =
        virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
    if (origin_lane) {
      fix_lane_virtual_id_ = origin_lane_virtual_id_;
    } else {
      fix_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    }
  } else if (state_machine_lc_state == kLaneChangeComplete) {
    const auto& target_lane =
        virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id_);
    if (target_lane) {
      fix_lane_virtual_id_ = target_lane_virtual_id_;
    } else {
      target_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
      origin_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
      fix_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    }
  } else {
    target_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    origin_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
    fix_lane_virtual_id_ = virtual_lane_mgr_->current_lane_virtual_id();
  }
}

void LaneChangeLaneManager::copy_lane_change_lanes(
    LaneChangeLaneManager& source) {
  target_lane_virtual_id_ = source.target_lane_virtual_id_;
  origin_lane_virtual_id_ = source.origin_lane_virtual_id_;
  fix_lane_virtual_id_ = source.fix_lane_virtual_id_;
}

void LaneChangeLaneManager::save_context(
    VirtualLaneManagerContext& context) const {
  // todo :clren
}
void LaneChangeLaneManager::restore_context(
    const VirtualLaneManagerContext& context) {
  // auto restore_master =
  //     [this](VirtualLane &target_lane, int master_position) {
  //       switch (master_position) {
  //       case LEFT_LEFT_POS:
  //         target_lane.attach(&lllane_);
  //         target_lane.set_neighbours(lllane_.neighbours());
  //         if (target_lane.has_raw_refline() && lllane_.has_raw_refline()) {
  //           auto raw_refline = target_lane.raw_refline();
  //           raw_refline->set_neighbours(lllane_.raw_refline()->neighbours());
  //         }
  //         break;

  //       case LEFT_POS:
  //         target_lane.attach(&llane_);
  //         target_lane.set_neighbours(llane_.neighbours());
  //         if (target_lane.has_raw_refline() && llane_.has_raw_refline()) {
  //           auto raw_refline = target_lane.raw_refline();
  //           raw_refline->set_neighbours(llane_.raw_refline()->neighbours());
  //         }
  //         break;

  //       case CURR_POS:
  //         target_lane.attach(&clane_);
  //         target_lane.set_neighbours(clane_.neighbours());
  //         if (target_lane.has_raw_refline() && clane_.has_raw_refline()) {
  //           auto raw_refline = target_lane.raw_refline();
  //           raw_refline->set_neighbours(clane_.raw_refline()->neighbours());
  //         }
  //         break;

  //       case RIGHT_POS:
  //         target_lane.attach(&rlane_);
  //         target_lane.set_neighbours(rlane_.neighbours());
  //         if (target_lane.has_raw_refline() && rlane_.has_raw_refline()) {
  //           auto raw_refline = target_lane.raw_refline();
  //           raw_refline->set_neighbours(rlane_.raw_refline()->neighbours());
  //         }
  //         break;

  //       case RIGHT_RIGHT_POS:
  //         target_lane.attach(&rrlane_);
  //         target_lane.set_neighbours(rrlane_.neighbours());
  //         if (target_lane.has_raw_refline() && rrlane_.has_raw_refline()) {
  //           auto raw_refline = target_lane.raw_refline();
  //           raw_refline->set_neighbours(rrlane_.raw_refline()->neighbours());
  //         }
  //         break;

  //       default:
  //         target_lane.detach();
  //         target_lane.update();
  //         break;
  //       }
  //     };

  // flane_update_ = context.flane_update;
  // flane_.restore_context(context.flane);
  // olane_.restore_context(context.olane);
  // tlane_.restore_context(context.tlane);

  // restore_master(flane_, context.flane.master_position);
  // restore_master(olane_, context.olane.master_position);
  // restore_master(tlane_, context.tlane.master_position);

  // f_refline_.restore_context(context.f_refline);
  // switch (f_refline_.position()) {
  // case CURR_REFLINE:
  //   f_refline_.set_master(&c_raw_refline_);
  //   break;
  // case LEFT_REFLINE:
  //   f_refline_.set_master(&l_raw_refline_);
  //   break;
  // case RIGHT_REFLINE:
  //   f_refline_.set_master(&r_raw_refline_);
  //   break;
  // case LEFT_LEFT_REFLINE:
  //   f_refline_.set_master(&ll_raw_refline_);
  //   break;
  // case RIGHT_RIGHT_REFLINE:
  //   f_refline_.set_master(&rr_raw_refline_);
  //   break;
  // default:
  //   break;
  // }
}

}  // namespace planning