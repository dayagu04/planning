#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "planning_debug_info.pb.h"
#include "slot_management.h"

namespace py = pybind11;
using namespace planning;

static SlotManagement *pBase = nullptr;

int Init() {
  pBase = new SlotManagement();
  pBase->Reset();
  return 0;
}

template <class T>
inline T BytesToProto(py::bytes &bytes) {
  T proto_obj;
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char *input_ptr = static_cast<char *>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

int UpdateBytes(py::bytes &func_statemachine_bytes,
                py::bytes &parking_slot_info_bytes,
                py::bytes &localization_info_bytes,
                py::bytes &uss_wave_info_bytes) {
  auto func_statemachine =
      BytesToProto<FuncStateMachine::FuncStateMachine>(func_statemachine_bytes);

  auto parking_slot_info =
      BytesToProto<ParkingFusion::ParkingFusionInfo>(parking_slot_info_bytes);

  auto localization_info =
      BytesToProto<LocalizationOutput::LocalizationEstimate>(
          localization_info_bytes);

  auto uss_wave_info =
      BytesToProto<UssWaveInfo::UssWaveInfo>(uss_wave_info_bytes);

  pBase->Update(&func_statemachine, &parking_slot_info, &localization_info,
                &uss_wave_info);

  return 0;
}

int UpdateBytesByParam(py::bytes &func_statemachine_bytes,
                       py::bytes &parking_slot_info_bytes,
                       py::bytes &localization_info_bytes,
                       py::bytes &uss_wave_info_bytes,
                       bool force_apa_on,
                       bool force_clear,
                       double max_slots_update_angle_dis_limit_deg,
                       double max_slot_boundary_line_angle_dif_deg,
                       double max_slot_update_lon_dif_slot_center_to_mirror,
                       double min_slot_update_lon_dif_slot_center_to_mirror) {
  auto func_statemachine =
      BytesToProto<FuncStateMachine::FuncStateMachine>(func_statemachine_bytes);

  auto parking_slot_info =
      BytesToProto<ParkingFusion::ParkingFusionInfo>(parking_slot_info_bytes);

  auto localization_info =
      BytesToProto<LocalizationOutput::LocalizationEstimate>(
          localization_info_bytes);

  auto uss_wave_info =
      BytesToProto<UssWaveInfo::UssWaveInfo>(uss_wave_info_bytes);

  SlotManagement::Param param;
  param.force_apa_on = force_apa_on;
  param.force_clear = force_clear;
  param.max_slots_update_angle_dis_limit_deg =
      max_slots_update_angle_dis_limit_deg;
  param.max_slot_boundary_line_angle_dif_deg =
      max_slot_boundary_line_angle_dif_deg;
  param.max_slot_update_lon_dif_slot_center_to_mirror =
      max_slot_update_lon_dif_slot_center_to_mirror;
  param.min_slot_update_lon_dif_slot_center_to_mirror =
      min_slot_update_lon_dif_slot_center_to_mirror;

  pBase->SetParam(param);
  pBase->Update(&func_statemachine, &parking_slot_info, &localization_info,
                &uss_wave_info);

  return 0;
}

py::bytes GetOutputBytes() {
  auto res = pBase->GetOutput();
  std::string serialized_message;
  res.SerializeToString(&serialized_message);

  return serialized_message;
}

PYBIND11_MODULE(slot_management_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &UpdateBytes)
      .def("UpdateBytes", &UpdateBytes)
      .def("UpdateBytesByParam", &UpdateBytesByParam)
      .def("GetOutputBytes", &GetOutputBytes);
}
