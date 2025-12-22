#ifndef ADAS_FUNCTION_PREPROCESS_H_
#define ADAS_FUNCTION_PREPROCESS_H_

#include "adas_function_context.h"
#include "adas_function_lib.h"
#include "transform_lib.h"

#define ADAS_JSON_READ_VALUE(var_name, type, json_name) \
  var_name = adas_config.get<type>(json_name, false, var_name)

#define VEHICLE_COMMON_JSON_READ_VALUE(var_name, type, json_name) \
  var_name = vehicle_common_json_reader.get<type>(json_name, false, var_name)
namespace adas_function {
namespace preprocess {

class Preprocess {
 public:
  void Init(const bool is_simulation = false);

  void SyncParameters(const bool is_simulation = false);

  void UpdateStateInfo(void);

  void UpdateRoadInfo(void);

  void UpdateObjsInfo(void);

  void UpdateTsrInfo(void);

  void RunOnce(void);

  double count_ = 0;

 private:
 std::string ReadJsonFile(const std::string &path);
  void SetLineInfoDefault(adas_function::context::LineInfo *line_info_ptr,
                          double c0);
  bool SetLineInfo(adas_function::context::LineInfo *line_info_ptr,
                   const iflyauto::LaneBoundary &lane_boundary_ptr);
  void SetRoadedgeInfo(void);

  void SetNavMapInfo(void);

  iflyauto::DrivingRoadType GetRoadTypeFromNaviInfo(int32 road_class, int32 form_way);

  iflyauto::DrivingRoadType GetRoadTypeFromProMap(const iflymapdata::sdpro::LinkInfo_Link* link);

  void ObjInLaneJudge(void);
  void SetEgoAroundAreaRange(void);
  void SingleAreaObjSelect(void);
  void SidewayExistJudge(void);
  void SafeDeparturePermissionJudge(void);
  double ObjCalculateTTC(const context::FusionObjExtractInfo &obj);
  bool last_left_line_valid_flag_ = false;
  bool last_right_line_valid_flag_ = false;
  double last_left_line_distance_ = 0.0;
  double last_right_line_distance_ = 0.0;
  double no_sideway_exist_time_counts = 0.0;
  bool read_json_file_ok_flag_ = false;
  
};

}  // namespace preprocess
}  // namespace adas_function

#endif