#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include "behavior_planners/real_time_longitudinal_behavior_planner/real_time_lon_behavior_planner.h"
#include "planning_debug_info.pb.h"
#include "real_time_lon_behavior_planner.pb.h"

namespace py = pybind11;

static planning::RealTimeLonBehaviorPlanner *pBase = nullptr;

int Init(std::string cfg_path) {
  std::string file_name = "general_planner_module_highway.json";
  std::string ego_planning_config_json_file = cfg_path + "/" + file_name;

  std::cout << ego_planning_config_json_file << std::endl;

  //std::string json_str;
  planning::Json ego_planning_config_json;
  std::ifstream fin(ego_planning_config_json_file);
  if (!fin.is_open()) {
    std::cout << "Failed to open json config file. " << std::endl;
  }
  else {
    std::cout << "open json config file. " << std::endl;
  }
  //fin >> ego_planning_config_json;
  //fin >> json_str;
  ego_planning_config_json = planning::Json::parse(fin);
  fin.close();
  //std::string json_str = ego_planning_config_json.dump();
  //std::cout << json_str << std::endl;
  planning::EgoPlanningConfigBuilder cfg_builder(ego_planning_config_json, file_name.c_str());
  pBase = new planning::RealTimeLonBehaviorPlanner(&cfg_builder);
  return 0;
}

template <class T>
inline T BytesToProto(py::bytes &bytes) {
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char *input_ptr = static_cast<char *>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

int UpdateBytes(py::bytes &planning_input_bytes) {
  planning::common::RealTimeLonBehaviorInput planning_input =
      BytesToProto<planning::common::RealTimeLonBehaviorInput>(planning_input_bytes);

  pBase->SetInput(planning_input);
  pBase->Update();

  return 0;
}

int SetConfigFromPy(py::bytes &planning_config) {
  planning::common::RealTimeLonBehaviorTunedParams tuned_params =
      BytesToProto<planning::common::RealTimeLonBehaviorTunedParams>(planning_config);

  pBase->SetConfig(tuned_params);

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

planning::common::LonRefPath GetOutput() {
  return pBase->GetOutput();
}

PYBIND11_MODULE(real_time_lon_behavior_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &UpdateBytes)
      .def("SetConfigFromPy", &SetConfigFromPy)
      .def("GetOutput", &GetOutput)
      .def("GetOutputBytes", &GetOutputBytes);
}
