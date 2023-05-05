#include "vision_longitudinal_behavior_planner.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "ifly_time.h"

namespace planning {

void VisionLongitudinalBehaviorPlanner::update_planner_output() {
  auto &vision_longitudinal_output =
            frame_->mutable_session()->mutable_planning_context()->mutable_vision_longitudinal_behavior_planner_output();
  // planning result
  vision_longitudinal_output.timestamp = IflyTime::Now_ms();

  vision_longitudinal_output.velocity_target = v_target_;
  vision_longitudinal_output.a_target_min = a_target_.first;
  vision_longitudinal_output.a_target_max = a_target_.second;

  vision_longitudinal_output.cutin_msg = cutin_msg_;

  vision_longitudinal_output.vel_sequence.clear();

  auto extra_json = mjson::Json(mjson::Json::object());
  extra_json["hdmap_valid"] = mjson::Json(false);
  extra_json["enter_ramp"] = mjson::Json(false);
  extra_json["cutin_avoid"] = mjson::Json(false);
  vision_longitudinal_output.extra_json = extra_json.dump();
}

void VisionLongitudinalBehaviorPlanner::log_planner_debug_info() {
  std::string plan_msg;
  create_vision_longitudinal_behavior_planner_msg(plan_msg);

  auto &vision_longitudinal_output =
            frame_->mutable_session()->mutable_planning_context()->mutable_vision_longitudinal_behavior_planner_output();

  vision_longitudinal_output.plan_msg = plan_msg;
}

void VisionLongitudinalBehaviorPlanner::
    create_vision_longitudinal_behavior_planner_msg(std::string &plan_msg) {
  auto &vision_longitudinal_output =
            frame_->mutable_session()->mutable_planning_context()->mutable_vision_longitudinal_behavior_planner_output();

  rapidjson::Document publish_json;
  publish_json.SetObject();
  rapidjson::Document::AllocatorType& allocator = publish_json.GetAllocator();
  // --------- acc debug ---------
  // 1.record a_limit
  rapidjson::Value a_limit(rapidjson::kObjectType);
  {
    rapidjson::Value json_array(rapidjson::kArrayType);
    json_array.PushBack(vision_longitudinal_output.a_limit_in_turns.first, allocator);
    json_array.PushBack(vision_longitudinal_output.a_limit_in_turns.second, allocator);
    a_limit.AddMember("a_limit_in_turns", json_array, allocator);
  }
  {
    rapidjson::Value json_array(rapidjson::kArrayType);
    for (auto item : vision_longitudinal_output.a_limit_cutin) {
      json_array.PushBack(rapidjson::Value().SetDouble(item), allocator);
    }
    a_limit.AddMember("a_limit_cutin", json_array, allocator);
  }
  if (vision_longitudinal_output.a_limit_cutin_history.size() > 0) {
    rapidjson::Value track_id(rapidjson::kArrayType);
    rapidjson::Value a_limit_cutin(rapidjson::kArrayType);
    for (auto item : vision_longitudinal_output.a_limit_cutin_history) {
      rapidjson::Value a_limit_cutin_history(rapidjson::kObjectType);
      track_id.PushBack(rapidjson::Value().SetInt(item.first), allocator);
      a_limit_cutin.PushBack(rapidjson::Value().SetDouble(item.second), allocator);
    }
    a_limit.AddMember("a_limit_cutin_history_track_id", track_id, allocator);
    a_limit.AddMember("a_limit_cutin_history", a_limit_cutin, allocator);
  } else {
    a_limit.AddMember("a_limit_cutin_history_track_id", rapidjson::Value().SetString("none"), allocator);
    a_limit.AddMember("a_limit_cutin_history", rapidjson::Value().SetString("none"), allocator);
  }
  {
    rapidjson::Value json_array(rapidjson::kArrayType);
    json_array.PushBack(rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_lead.first), allocator);
    json_array.PushBack(rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_lead.second), allocator);
    a_limit.AddMember("a_target_lead", json_array, allocator);
  }
  {
    rapidjson::Value json_array(rapidjson::kArrayType);
    json_array.PushBack(rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_temp_lead_one.first), allocator);
    json_array.PushBack(rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_temp_lead_one.second), allocator);
    a_limit.AddMember("a_target_temp_lead_one", json_array, allocator);
  }
  {
    rapidjson::Value json_array(rapidjson::kArrayType);
    json_array.PushBack(rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_temp_lead_two.first), allocator);
    json_array.PushBack(rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_temp_lead_two.second), allocator);
    a_limit.AddMember("a_target_temp_lead_two", json_array, allocator);
  }
  a_limit.AddMember("a_target_ramp", rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_ramp), allocator);
  a_limit.AddMember("a_target_cutin_front", rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_cutin_front), allocator);
  a_limit.AddMember("a_target_pre_brake", rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_pre_brake), allocator);
  a_limit.AddMember("a_target_merge", rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_merge), allocator);
  a_limit.AddMember("a_target_lane_change", rapidjson::Value().SetDouble(vision_longitudinal_output.a_target_lane_change), allocator);
  a_limit.AddMember("decel_base", rapidjson::Value().SetDouble(vision_longitudinal_output.decel_base), allocator);
  a_limit.AddMember("a_target_min", rapidjson::Value().SetDouble(a_target_.first), allocator);
  a_limit.AddMember("a_target_max", rapidjson::Value().SetDouble(a_target_.second), allocator);
  // 2.record v_limit
  rapidjson::Value v_limit(rapidjson::kObjectType);
  v_limit.AddMember("v_limit_in_turns", rapidjson::Value().SetDouble(vision_longitudinal_output.v_limit_in_turns), allocator);
  {
    rapidjson::Value json_array(rapidjson::kArrayType);
    for (auto item : vision_longitudinal_output.v_limit_cutin) {
      json_array.PushBack(rapidjson::Value().SetDouble(item), allocator);
    }
    v_limit.AddMember("v_limit_cutin", json_array, allocator);
  }
  v_limit.AddMember("v_target_lead_one", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_lead_one), allocator);
  v_limit.AddMember("v_target_lead_two", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_lead_two), allocator);
  v_limit.AddMember("v_target_temp_lead_one", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_temp_lead_one), allocator);
  v_limit.AddMember("v_target_temp_lead_two", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_temp_lead_two), allocator);
  v_limit.AddMember("v_target_terminus", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_terminus), allocator);
  v_limit.AddMember("v_target_ramp", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_ramp), allocator);
  v_limit.AddMember("v_target_cutin_front", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_cutin_front), allocator);
  v_limit.AddMember("v_target_pre_brake", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_pre_brake), allocator);
  v_limit.AddMember("v_target_merge", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_merge), allocator);
  v_limit.AddMember("v_target_lane_change", rapidjson::Value().SetDouble(vision_longitudinal_output.v_target_lane_change), allocator);
  // acc_log
  rapidjson::Value acc_log(rapidjson::kObjectType);
  acc_log.AddMember("a_limit", a_limit, allocator);
  acc_log.AddMember("v_limit", v_limit, allocator);

  publish_json.AddMember("acc_log", acc_log, allocator);
  // --------- end of acc debug ---------

  rapidjson::StringBuffer jsonBuffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(jsonBuffer);
  publish_json.Accept(writer);
  // printf("%s\n",jsonBuffer.GetString());

  // 带格式化的输出
  // rapidjson::StringBuffer pretty_jsonBuffer;
  // rapidjson::PrettyWriter<rapidjson::StringBuffer> pretty_writer(pretty_jsonBuffer);
  // publish_json.Accept(pretty_writer);
  // printf("%s\n",pretty_jsonBuffer.GetString());

  plan_msg = std::string(jsonBuffer.GetString(), jsonBuffer.GetSize());
}

} // namespace planning