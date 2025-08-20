#include "map_lane_change_request.h"
#include "common_c.h"
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "planning_context.h"

namespace planning {
// class: MapRequest
MapRequest::MapRequest(
    framework::Session* session, const EgoPlanningConfigBuilder* config_builder,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {}
void MapRequest::Update(int lc_status, double lc_map_tfinish) {
  ILOG_INFO << "MapRequest::update";
  //检查是否有拨杆信息
  const int ego_blinker = session_->mutable_environmental_model()
                              ->get_ego_state_manager()
                              ->ego_blinker();
  const bool is_valid_ego_blinker = ego_blinker == 1 || ego_blinker == 2;
  const bool is_cancel_mlc_for_ego_blinker =
      is_valid_ego_blinker && lc_status <= kLaneChangeExecution &&
      request_type_ != NO_CHANGE &&
      ((ego_blinker == 1 && request_type_ == RIGHT_CHANGE) ||
       (ego_blinker == 2 && request_type_ == LEFT_CHANGE));
  //如果CheckMLCEnable检查没通过，根据当前状态判断是否可以取消已经生成了的request_type_
  const bool allow_cancel =
      (lc_status == kLaneChangePropose || lc_status == kLaneKeeping);
  //检查是否满足变道请求
  const bool is_mlc_enable = CheckMLCEnable(lc_status);
  if (is_mlc_enable && !is_valid_ego_blinker) {
    GenerateMLCRequest();
  } else if ((allow_cancel || is_cancel_mlc_for_ego_blinker) &&
             request_type_ != NO_CHANGE) {
    Finish();
    set_target_lane_virtual_id(lane_change_lane_mgr_->origin_lane_virtual_id());
    ILOG_DEBUG << "[MapRequest::update] cancel map request as allow cancel";
  }
  ILOG_DEBUG << "MapRequest::update: finished";
}

bool MapRequest::CheckMLCEnable(const int lc_status) {
  //检查是否满足变道条件，如果满足，则生成mlc_req
  const int current_lane_virtual_id =
      virtual_lane_mgr_->current_lane_virtual_id();
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  if (current_lane == nullptr) {
    return false;
  }

  // 1、当前车道有变道任务数，否则返回false
  const int lc_map_decision = virtual_lane_mgr_->lc_map_decision(current_lane);
  JSON_DEBUG_VALUE("lc_map_decision", lc_map_decision);
  if (lc_map_decision == 0) {
    return false;
  }
  const RequestType request_type =
      lc_map_decision < 0 ? LEFT_CHANGE : RIGHT_CHANGE;

  // 2、当前状态可以生成变道请求，否则返回false
  const bool allow_generate = lc_status == kLaneChangePropose ||
                              lc_status == kLaneChangeHold ||
                              lc_status == kLaneKeeping;
  if (!allow_generate) {
    return false;
  }

  // 3、判断剩余距离是否触发生成变道请求
  bool is_trigger_mlc = IsTriggerMLCForRemainDistane();
  if (!is_trigger_mlc) {
    return false;
  }

  // 4、判断虚线长度是否满足变道条件
  const RequestType target_direction =
      lc_map_decision < 0 ? LEFT_CHANGE : RIGHT_CHANGE;

  bool is_dash_enough = IsDashEnoughForRepeatSegments(
      target_direction, current_lane->get_virtual_id(),
      static_cast<StateMachineLaneChangeStatus>(lc_status));
  if (!is_dash_enough) {
    return false;
  }

  // 5、判断目标车道是否有汇流箭头与当前变道请求冲突
  if (!CheckTargetLaneLaneMarks(request_type)) {
    return false;
  }

  // 6、判断目标车道与当前车道在前方是否存在merge split_region
  if (!CheckTargetLaneMergeDirection(request_type)) {
    return false;
  }

  return true;
}

bool MapRequest::IsTriggerMLCForRemainDistane() {
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double v_ego = ego_state->ego_v();
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  int lc_map_decision = virtual_lane_mgr_->lc_map_decision(current_lane);

  // TODO(fengwang31):目前自车在最左侧车道上时，无法确认到最右边有几条车道，
  //因此hack一下，判断自车在左侧，提前产生变道任务
  bool is_ego_on_leftmost = false;
  auto right_lane = virtual_lane_mgr_->get_right_lane();
  if (right_lane) {
    bool is_solid_right_lane_left =
        MakesureCurrentBoundaryType(LEFT_CHANGE,
                                    right_lane->get_virtual_id()) ==
        iflyauto::LaneBoundaryType_MARKING_SOLID;
    bool is_solid_right_lane_right =
        MakesureCurrentBoundaryType(RIGHT_CHANGE,
                                    right_lane->get_virtual_id()) ==
        iflyauto::LaneBoundaryType_MARKING_SOLID;
    bool is_right_lane_boundary_both_dash =
        !is_solid_right_lane_left && !is_solid_right_lane_right;
    bool is_solid_cur_lane_left =
        MakesureCurrentBoundaryType(LEFT_CHANGE,
                                    current_lane->get_virtual_id()) ==
        iflyauto::LaneBoundaryType_MARKING_SOLID;
    bool is_solid_cur_lane_right =
        MakesureCurrentBoundaryType(RIGHT_CHANGE,
                                    current_lane->get_virtual_id()) ==
        iflyauto::LaneBoundaryType_MARKING_SOLID;
    bool is_current_lane_is_leftmost =
        is_solid_cur_lane_left && !is_solid_cur_lane_right;
    is_ego_on_leftmost =
        is_current_lane_is_leftmost && is_right_lane_boundary_both_dash;
  }

  //确定地图变道触发距离lc_map_decision
  const double kTmpRampLength = 100.;
  const double kResponseOffset = 300.;
  const double kDefaultMapDelay = 3.;
  if (lc_map_decision == 1) {
    lc_map_decision = is_ego_on_leftmost ? 2 : 1;
  }
  double lc_end_dis;
  if (route_info_output.is_on_ramp ||
      route_info_output.lc_nums_for_split != 0) {
    lc_end_dis =
        route_info_output.distance_to_first_road_split - kTmpRampLength;
  } else {
    lc_end_dis = route_info_output.dis_to_ramp - kTmpRampLength;
  }

  double v_limit =
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise();
  std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
  std::array<double, 3> fp{500.0, 800.0, 1200.0};
  const double adaptor_interval = interp(v_limit, xp, fp);
  const double map_response_dist =
      kResponseOffset + adaptor_interval * std::fabs(lc_map_decision);

  if (lc_map_decision < 0) {
    lc_end_dis = 0.;
  }

  return (lc_end_dis < map_response_dist);
}

void MapRequest::GenerateMLCRequest() {
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  const int lc_map_decision = virtual_lane_mgr_->lc_map_decision(current_lane);
  // lc_map_decision 小于0表示左转，大于0表示右转
  if (lc_map_decision < 0) {
    const auto& target_lane = virtual_lane_mgr_->get_left_lane();
    if (request_type_ != LEFT_CHANGE && target_lane && target_lane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE) {
      GenerateRequest(LEFT_CHANGE);
      set_target_lane_virtual_id(target_lane->get_virtual_id());
      ILOG_DEBUG << "[MapRequest::update] Ask for map changing lane to left";
    }
  } else {
    const auto& target_lane = virtual_lane_mgr_->get_right_lane();
    if (request_type_ != RIGHT_CHANGE && target_lane && target_lane->get_lane_type() != iflyauto::LANETYPE_OPPOSITE) {
      GenerateRequest(RIGHT_CHANGE);
      set_target_lane_virtual_id(target_lane->get_virtual_id());
      ILOG_DEBUG << "[MapRequest::update] Ask for map changing lane to right";
    }
  }
}

bool MapRequest::CheckTargetLaneLaneMarks(RequestType request_type) {
  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<VirtualLane> target_lane = nullptr;
  std::shared_ptr<ReferencePath> target_path = nullptr;
  if (request_type == LEFT_CHANGE) {
    target_lane = virtual_lane_mgr_->get_left_lane();
  } else if (request_type == RIGHT_CHANGE) {
    target_lane = virtual_lane_mgr_->get_right_lane();
  }

  if (!target_lane) {
    return false;
  }

  target_path = reference_path_manager->get_reference_path_by_lane(
      target_lane->get_virtual_id());
  if (!target_path) {
    return false;
  }

  const double ego_s_on_target_lane = target_path->get_frenet_ego_state().s();
  const auto& target_lane_marks = target_lane->lane_marks();
  int seg_index = target_lane_marks.size() + 1;
  // seg_index这样取值是保证如果没有匹配到相应的index，那么后面不会进入到循环中去
  for (int i = 0; i < target_lane_marks.size(); ++i) {
    const auto& mark = target_lane_marks[i];
    if (mark.begin < ego_s_on_target_lane && mark.end > ego_s_on_target_lane) {
      seg_index = i;
      break;
    }
  }
  //遍历判断前方是否有与变道方向冲突的汇流车道
  for (int j = seg_index; j < target_lane_marks.size(); ++j) {
    if (request_type == LEFT_CHANGE) {
      if (target_lane_marks[j].lane_mark ==
          iflyauto::LaneDrivableDirection_DIRECTION_RIGHT_MERGE) {
        return false;
        break;
      }
    } else if (request_type == RIGHT_CHANGE) {
      if (target_lane_marks[j].lane_mark ==
          iflyauto::LaneDrivableDirection_DIRECTION_LEFT_MERGE) {
        return false;
        break;
      }
    }
  }
  return true;
}

bool MapRequest::CheckTargetLaneMergeDirection(RequestType request_type) {
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  bool is_merge_region = ego_lane_road_right_decider_output.is_merge_region;
  bool cur_lane_is_continue =
      ego_lane_road_right_decider_output.cur_lane_is_continue;
  int merge_lane_virtual_id =
      ego_lane_road_right_decider_output.merge_lane_virtual_id;
  int target_lane_virtual_id = 0;
  if (request_type == LEFT_CHANGE) {
    const auto& left_lane = virtual_lane_mgr_->get_left_lane();
    if (!left_lane) {
      std::cout << "left_lane is nullptr" << std::endl;
      return false;
    }
    target_lane_virtual_id = left_lane->get_virtual_id();
  } else if (request_type == RIGHT_CHANGE) {
    const auto& right_lane = virtual_lane_mgr_->get_right_lane();
    if (!right_lane) {
      std::cout << "right_lane is nullptr" << std::endl;
      return false;
    }
    target_lane_virtual_id = right_lane->get_virtual_id();
  }
  if (is_merge_region && cur_lane_is_continue &&
      target_lane_virtual_id == merge_lane_virtual_id) {
    return false;
  }
  return true;
}
}  // namespace planning