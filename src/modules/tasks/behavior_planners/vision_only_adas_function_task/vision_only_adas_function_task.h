#include "session.h"
#include "tasks/task.h"
// #include "lane_keep_assist_manager.h"
#include "Platform_Types.h"
#include "adas_function/ihc_function/intelligent_headlight_control.h"
#include "adas_function/lkas_function/lane_keep_assist_manager.h"
#include "adas_function/tsr_function/traffic_sign_recognition.h"
#include "debug_info_log.h"
#include "ifly_time.h"

namespace planning {

class VisionOnlyAdasFunctionTask : public Task {
 public:
  explicit VisionOnlyAdasFunctionTask(
      const EgoPlanningConfigBuilder *config_builder,
      framework::Session *session);
  virtual ~VisionOnlyAdasFunctionTask() = default;

  bool Execute() override;

 private:
  // LaneKeepAssistManager lane_keep_assit_manager_;
  VisionOnlyAdasFunctionTaskConfig config_;
};

}  // namespace planning