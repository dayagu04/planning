#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>
namespace planning {

enum class SpeedLimitType {
  NONE = 0,
  CURVATURE = 1,
  MERGE = 2,
  CRUISE = 3,
  CONE_BUCKET = 4,
  CIPV_LOST = 5,
  ROUNDABOUT = 6,
  NOT_OVERTAKE_FROM_RIGHT = 7,
  NORMAL_KAPPA = 8,
  VRU_ROUND = 9,
  MERGE_ALC = 10,
  MAP_NEAR_RAMP = 11,
  MAP_ON_RAMP = 12,
  INTERSECTION = 13
};

class SpeedLimitDeciderOutput {
 public:
  SpeedLimitDeciderOutput() = default;
  ~SpeedLimitDeciderOutput() = default;

  void SetSpeedLimit(const double limited_speed,
                     const SpeedLimitType& speed_limit_type);
  void SetSpeedLimitIntoMap(const double limited_speed,
                     const SpeedLimitType& speed_limit_type);

  bool GetSpeedLimit(double* const limited_speed,
                     SpeedLimitType* const speed_limit_type) const;

  const std::vector<SpeedLimitType>& GetAllSpeedLimitTypes() const;

  std::string ChangeSpeedLimitType(const SpeedLimitType type);

  int32_t merge_alc_speed_limit_type() const;
  void set_merge_alc_speed_limit_type(const int32_t merge_alc_speed_limit_type);

  const std::string& merge_alc_speed_limit_debug_string() const;
  void set_merge_alc_speed_limit_debug_string(
      const std::string& merge_alc_speed_limit_debug_string);

  int32_t roundabout_speed_limit_type() const;
  void set_roundabout_speed_limit_type(
      const int32_t roundabout_speed_limit_type);

  std::string cipv_lost_debug_string_() const;
  void set_cipv_lost_debug_string(
      const std::string& cipv_lost_speed_limit_debug_string_);

  std::string roundabout_debug_string() const;
  void set_roundabout_debug_string(
      const std::string& roundabout_speed_limit_debug_string);

  int32_t not_overtake_from_right_speed_limit_type() const;
  void set_not_overtake_from_right_speed_limit_type(
      const int32_t not_overtake_from_right_speed_limit_type);
  const std::string& not_overtake_from_right_speed_limit_debug_string() const;
  void set_not_overtake_from_right_speed_limit_debug_string(
      const std::string& not_overtake_from_right_speed_limit_debug_string);

  std::string vru_round_debug_string() const;
  void set_vru_round_debug_string(
      const std::string& cipv_lost_speed_limit_debug_string_);

 private:
  std::map<SpeedLimitType, double> speed_limit_map_; //(type, speedlimit) for all scenes one by one
  std::pair<SpeedLimitType, double> speed_limit_type_final_; //final speed limit and type
  int32_t roundabout_speed_limit_type_ = 0;
  int32_t not_overtake_from_right_speed_limit_type_ = 0;
  int32_t merge_alc_speed_limit_type_ = 0;
  std::string merge_alc_speed_limit_debug_string_ = "";
  std::string cone_bucket_speed_limit_debug_string_ = "";
  std::string cipv_lost_speed_limit_debug_string_ = "";
  std::string roundabout_speed_limit_debug_string_ = "";
  std::string not_overtake_from_right_speed_limit_debug_string_ = "";
  std::string vru_round_speed_limit_debug_string_ = "";
  std::vector<SpeedLimitType> speed_limit_types_ = {
      SpeedLimitType::NONE,         SpeedLimitType::CURVATURE,
      SpeedLimitType::CRUISE,       SpeedLimitType::MERGE,
      SpeedLimitType::CIPV_LOST,    SpeedLimitType::CONE_BUCKET,
      SpeedLimitType::ROUNDABOUT,   SpeedLimitType::NOT_OVERTAKE_FROM_RIGHT,
      SpeedLimitType::NORMAL_KAPPA, SpeedLimitType::VRU_ROUND,
      SpeedLimitType::MERGE_ALC,    SpeedLimitType::MAP_NEAR_RAMP,
      SpeedLimitType::MAP_ON_RAMP,  SpeedLimitType::INTERSECTION};
};

}  // namespace planning
