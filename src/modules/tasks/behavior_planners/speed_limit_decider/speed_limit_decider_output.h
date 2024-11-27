#pragma once

#include <cstdint>
#include <map>
#include <vector>
#include <string>
namespace planning {

enum class SpeedLimitType {
  NONE = 0,
  CURVATURE = 1,
  MERGE = 2,
  CRUISE = 3,
  LIDAR_RB = 4,
  CONE_BUCKET = 5,
  CIPV_LOST = 6,
  ROUNDABOUT = 7,
  DDLD = 8,
  NOT_OVERTAKE_FROM_RIGHT = 9,
  NORMAL_KAPPA = 10,
  VRU_ROUND = 11,
  MERGE_ALC = 12,
};

class SpeedLimitDeciderOutput {
 public:
  SpeedLimitDeciderOutput() = default;
  ~SpeedLimitDeciderOutput() = default;

  void SetSpeedLimit(const double limited_speed,
                     const SpeedLimitType& speed_limit_type);

  bool GetSpeedLimit(double* const limited_speed,
                     SpeedLimitType* const speed_limit_type) const;

  const std::vector<SpeedLimitType>& GetAllSpeedLimitTypes() const;

  std::string ChangeSpeedLimitType(const SpeedLimitType type);

  int32_t ddld_speed_limit_type() const;
  void set_ddld_speed_limit_type(const int32_t ddld_speed_limit_type);

  const std::string& ddld_speed_limit_debug_string() const;
  void set_ddld_speed_limit_debug_string(
      const std::string& ddld_speed_limit_debug_string);

  int32_t merge_alc_speed_limit_type() const;
  void set_merge_alc_speed_limit_type(const int32_t merge_alc_speed_limit_type);

  const std::string& merge_alc_speed_limit_debug_string() const;
  void set_merge_alc_speed_limit_debug_string(
      const std::string& merge_alc_speed_limit_debug_string);

  int32_t lidar_rb_speed_limit_type() const;
  void set_lidar_rb_speed_limit_type(const int32_t lidar_rb_speed_limit_type);

  int32_t roundabout_speed_limit_type() const;
  void set_roundabout_speed_limit_type(
      const int32_t roundabout_speed_limit_type);

  std::string lidar_rb_debug_string() const;
  void set_lidar_rb_debug_string(
      const std::string& lidar_rb_speed_limit_debug_string);

  std::string cone_bucket_debug_string() const;
  void set_cone_bucket_debug_string(
      const std::string& lidar_rb_speed_limit_debug_string);

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
  std::map<double, SpeedLimitType> speed_limit_map_;
  int32_t ddld_speed_limit_type_ = 0;
  int32_t lidar_rb_speed_limit_type_ = 0;
  int32_t roundabout_speed_limit_type_ = 0;
  int32_t not_overtake_from_right_speed_limit_type_ = 0;
  int32_t merge_alc_speed_limit_type_ = 0;
  std::string ddld_speed_limit_debug_string_ = "";
  std::string merge_alc_speed_limit_debug_string_ = "";
  std::string lidar_rb_speed_limit_debug_string_ = "";
  std::string cone_bucket_speed_limit_debug_string_ = "";
  std::string cipv_lost_speed_limit_debug_string_ = "";
  std::string roundabout_speed_limit_debug_string_ = "";
  std::string not_overtake_from_right_speed_limit_debug_string_ = "";
  std::string vru_round_speed_limit_debug_string_ = "";
  std::vector<SpeedLimitType> speed_limit_types_ = {
      SpeedLimitType::NONE,         SpeedLimitType::CURVATURE,
      SpeedLimitType::CRUISE,       SpeedLimitType::MERGE,
      SpeedLimitType::CIPV_LOST,    SpeedLimitType::CONE_BUCKET,
      SpeedLimitType::LIDAR_RB,     SpeedLimitType::ROUNDABOUT,
      SpeedLimitType::DDLD,         SpeedLimitType::NOT_OVERTAKE_FROM_RIGHT,
      SpeedLimitType::NORMAL_KAPPA, SpeedLimitType::VRU_ROUND,
      SpeedLimitType::MERGE_ALC};
};

}  // namespace planning
