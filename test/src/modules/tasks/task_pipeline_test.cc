#include "src/modules/tasks/task_pipeline.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "src/common/log.h"
#include "src/modules/tasks/task_pipeline_context.h"
#include "src/modules/scenario/ego_planning_candidate.h"

namespace planning {

TEST(TestTaskPpeline, TaskPipeline) {
  printf("TestTaskPpeline: TaskPipeline");
  // 初始化log配置
  std::string log_file = "/asw/Planning/log/planning_log";
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    bst::DEBUG);
  // 初始化task pipeline所需的frame
  framework::Session session{};
  session.Init();
  framework::Frame frame{&session};

  auto highway_config_builder =
      std::make_shared<EgoPlanningConfigBuilder>("", "empty");

  session.mutable_environmental_model()->set_highway_config_builder(
      highway_config_builder.get());

  std::shared_ptr<TaskPipeline> taskpipeline;

  common::SceneType scene_type = frame.session()->get_scene_type();
  auto config_builder =
      frame.session()->environmental_model().config_builder(scene_type);
  taskpipeline =
      TaskPipeline::Make(TaskPipelineType::NORMAL, config_builder, &frame);
  printf("TestTaskPpeline: TaskPipeline finish");

  EgoPlanningCandidate candidate(&frame);
  // candidate.set_coarse_planning_info(transition_context);
  bool ok = taskpipeline->Run(candidate);
}
}  // namespace planning
