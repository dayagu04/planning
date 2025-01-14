#include "speed_limit_decider_output.h"

namespace planning {

bool SpeedLimitDeciderOutput::GetSpeedLimit(
    double* const limited_speed, SpeedLimitType* const speed_limit_type) const {
  if (speed_limit_map_.empty()) {
    *speed_limit_type = SpeedLimitType::NONE;
    return false;
  }
  auto speed_limit_result = speed_limit_map_.begin();
  (*limited_speed) = speed_limit_result->first;
  (*speed_limit_type) = speed_limit_result->second;
  // std::cout << "print speed map: " << std::endl;
  // for (const auto &entry : speed_limit_map_) {
  //   std::cout << "value: " << entry.first << " type: " << (int)(entry.second)
  //             << std::endl;
  // }
  return true;
}

std::string SpeedLimitDeciderOutput::ChangeSpeedLimitType(
    const SpeedLimitType type) {
  std::string ret;
  if (SpeedLimitType::NONE == type) {
    ret.append("NONE");
  } else if (SpeedLimitType::CRUISE == type) {
    ret.append("CRUISE");
  } else if (SpeedLimitType::CURVATURE == type) {
    ret.append("CURVATURE");
  } else if (SpeedLimitType::MERGE == type) {
    ret.append("MERGE");
  } else if (SpeedLimitType::CONE_BUCKET == type) {
    ret.append("CONE_BUCKET");
  } else if (SpeedLimitType::CIPV_LOST == type) {
    ret.append("CIPV_LOST");
  } else if (SpeedLimitType::ROUNDABOUT == type) {
    ret.append("ROUNDABOUT");
  } else if (SpeedLimitType::NOT_OVERTAKE_FROM_RIGHT == type) {
    ret.append("NOT_OVERTAKE_FROM_RIGHT");
  } else if (SpeedLimitType::VRU_ROUND == type) {
    ret.append("VRU_ROUND");
  } else if (SpeedLimitType::MERGE_ALC == type) {
    ret.append("MERGE_ALC");
  } else {
    ret.append("type error");
  }
  return ret;
}

int32_t SpeedLimitDeciderOutput::merge_alc_speed_limit_type() const {
  return merge_alc_speed_limit_type_;
}

void SpeedLimitDeciderOutput::set_merge_alc_speed_limit_type(
    const int32_t merge_alc_speed_limit_type) {
  merge_alc_speed_limit_type_ = merge_alc_speed_limit_type;
}

const std::string& SpeedLimitDeciderOutput::merge_alc_speed_limit_debug_string()
    const {
  return merge_alc_speed_limit_debug_string_;
}

void SpeedLimitDeciderOutput::set_merge_alc_speed_limit_debug_string(
    const std::string& merge_alc_speed_limit_debug_string) {
  merge_alc_speed_limit_debug_string_ = merge_alc_speed_limit_debug_string;
}

int32_t SpeedLimitDeciderOutput::roundabout_speed_limit_type() const {
  return roundabout_speed_limit_type_;
}

void SpeedLimitDeciderOutput::set_roundabout_speed_limit_type(
    const int32_t roundabout_speed_limit_type) {
  roundabout_speed_limit_type_ = roundabout_speed_limit_type;
}

std::string SpeedLimitDeciderOutput::roundabout_debug_string() const {
  return roundabout_speed_limit_debug_string_;
}
void SpeedLimitDeciderOutput::set_roundabout_debug_string(
    const std::string& roundabout_speed_limit_debug_string) {
  roundabout_speed_limit_debug_string_ = roundabout_speed_limit_debug_string;
}

int32_t SpeedLimitDeciderOutput::not_overtake_from_right_speed_limit_type()
    const {
  return not_overtake_from_right_speed_limit_type_;
}
void SpeedLimitDeciderOutput::set_not_overtake_from_right_speed_limit_type(
    const int32_t not_overtake_from_right_speed_limit_type) {
  not_overtake_from_right_speed_limit_type_ =
      not_overtake_from_right_speed_limit_type;
}
const std::string&
SpeedLimitDeciderOutput::not_overtake_from_right_speed_limit_debug_string()
    const {
  return not_overtake_from_right_speed_limit_debug_string_;
}
void SpeedLimitDeciderOutput::
    set_not_overtake_from_right_speed_limit_debug_string(
        const std::string& not_overtake_from_right_speed_limit_debug_string) {
  not_overtake_from_right_speed_limit_debug_string_ =
      not_overtake_from_right_speed_limit_debug_string;
}

std::string SpeedLimitDeciderOutput::cipv_lost_debug_string_() const {
  return cipv_lost_speed_limit_debug_string_;
}
void SpeedLimitDeciderOutput::set_cipv_lost_debug_string(
    const std::string& cipv_lost_speed_limit_debug_string) {
  cipv_lost_speed_limit_debug_string_ = cipv_lost_speed_limit_debug_string;
}

const std::vector<SpeedLimitType>&
SpeedLimitDeciderOutput::GetAllSpeedLimitTypes() const {
  return speed_limit_types_;
}

std::string SpeedLimitDeciderOutput::vru_round_debug_string() const {
  return vru_round_speed_limit_debug_string_;
}

void SpeedLimitDeciderOutput::set_vru_round_debug_string(
    const std::string& vru_round_speed_limit_debug_string) {
  vru_round_speed_limit_debug_string_ = vru_round_speed_limit_debug_string;
}

}  // namespace planning
