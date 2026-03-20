#include "ifly_unit.h"

#include "struct_convert/control_command_c.h"
#include "struct_msgs/ControlOutput.h"

#include "struct_convert_legacy/interface2.4.6/common_c.h"
#include "struct_convert_legacy/interface2.4.6/localization_c.h"
#include "struct_msgs_legacy_v2_4_6/LocalizationEstimate.h"

#include "struct_convert_legacy/interface2.4.5/common_c.h"
#include "struct_convert_legacy/interface2.4.5/func_state_machine_c.h"
#include "struct_msgs_legacy_v2_4_5/FuncStateMachine.h"


int tests_access = 0;
int tests_failed = 0;
int tests_summary = 0;

static std::string TEST_ControlOutput_Convert() {
  iflyauto::ControlOutput struct_msg;
  struct_msgs::ControlOutput ros_msg;
  struct_msg.steering = 0.001;
  convert(struct_msg, ros_msg, ConvertTypeInfo::TO_ROS);
  ifly_assert("ControlOutput.steering is not equal",
              struct_msg.steering == ros_msg.steering);
  return "";
}

static std::string TEST_LocalizationEstimate_246_Convert() {
  iflyauto::interface_2_4_6::LocalizationEstimate struct_msg;
  struct_msgs_legacy_v2_4_6::LocalizationEstimate ros_msg;
  struct_msg.pose.heading = 0.002;
  convert(struct_msg, ros_msg, ConvertTypeInfo::TO_ROS);
  ifly_assert("ControlOutput.pose.heading is not equal",
              struct_msg.pose.heading == ros_msg.pose.heading);
  return "";
}

static std::string TEST_FuncStateMachine_245_Convert() {
  iflyauto::interface_2_4_5::FuncStateMachine struct_msg;
  struct_msgs_legacy_v2_4_5::FuncStateMachine ros_msg;
  struct_msg.state_duration = 20;
  convert(struct_msg, ros_msg, ConvertTypeInfo::TO_ROS);
  ifly_assert("ControlOutput.state_duration is not equal",
              struct_msg.state_duration == ros_msg.state_duration);
  return "";
}

static void all_tests() {
  ifly_run_test(TEST_ControlOutput_Convert);
  ifly_run_test(TEST_LocalizationEstimate_246_Convert);
  ifly_run_test(TEST_FuncStateMachine_245_Convert);
}

int main(int argc, char **argv) {
  all_tests();
  printf("-------Tests access : %d\n", tests_access);
  printf("-------Tests failed : %d\n", tests_failed);
  printf("-------Tests summary: %d\n", tests_summary);
  return 0;
}