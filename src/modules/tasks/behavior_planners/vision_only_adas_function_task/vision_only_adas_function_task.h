#include "frame.h"
#include "task.h"
// #include "lane_keep_assist_manager.h"
#include "Platform_Types.h"
#include "ihc_function/intelligent_headlight_control.h"
#include "lkas_function/lane_keep_assist_manager.h"
#include "tsr_function/traffic_sign_recognition.h"

namespace planning {

class VisionOnlyAdasFunctionTask : public Task {
 public:
  explicit VisionOnlyAdasFunctionTask(const EgoPlanningConfigBuilder *config_builder,
                                      const std::shared_ptr<TaskPipelineContext> &pipeline_context);
  virtual ~VisionOnlyAdasFunctionTask() = default;

  bool Execute(framework::Frame *frame) override;

 private:
  // LaneKeepAssistManager lane_keep_assit_manager_;
  VisionOnlyAdasFunctionTaskConfig config_;
};

}  // namespace planning