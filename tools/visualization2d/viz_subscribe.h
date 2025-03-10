/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>

#include "ifly_localization.pb.h"
#include "ifly_time.h"
#include "localization.pb.h"

#include "control_command.pb.h"
#include "ehr.pb.h"
#include "func_state_machine.pb.h"
#include "fusion_objects.pb.h"
#include "fusion_road.pb.h"
#include "groundline_perception.pb.h"
#include "hmi_mcu_inner.pb.h"
#include "ifly_parking_map.pb.h"
#include "ifly_time.h"
#include "parking_fusion.pb.h"
#include "parking_slot_list.pb.h"
#include "planning_debug_info.pb.h"
#include "planning_plan.pb.h"
#include "prediction.pb.h"
#include "uss_percept_info.pb.h"
#include "uss_wave_info.pb.h"
#include "vehicle_service.pb.h"

namespace planning {
/**
 * @struct local_view
 * @brief LocalView contains all necessary data as viz input,
 * 每一帧的数据,有的数据是每一帧刷新，有的数据是历史数据，注意：cyber
 * rt传输的数据最好拷贝到这里
 */

struct VizSubscribe {
  LocalizationOutput::LocalizationEstimate localization_estimate;

  IFLYLocalization::IFLYLocalization localization;

  GroundLinePerception::FusionGroundLineInfo ground_line_perception;

  FusionObjects::FusionObjectsInfo fusion_objects_info;

  ControlCommand::ControlOutput control_output;

  VehicleService::VehicleServiceOutputInfo vehicle_service;

  ParkingFusion::ParkingFusionInfo parking_fusion_info;

  FuncStateMachine::FuncStateMachine function_state_machine_info;

  UssWaveInfo::UssWaveInfo uss_wave_info;

  UssPerceptInfo::UssPerceptInfo uss_percept_info;

  PlanningOutput::PlanningOutput last_planning_output_;

  planning::common::PlanningDebugInfo planning_debug_;
};

}  // namespace planning