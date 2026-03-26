#pragma once

#include "func_state_machine_c.h"
#include "fusion_parking_slot_c.h"
#include "planning_plan_c.h"
#include "session.h"

namespace planning {

std::vector<std::vector<Eigen::Vector2d>> GetODTraj();

const bool IsODVeh(const iflyauto::ObjectType type);

const bool IsODSpecificationer(const iflyauto::ObjectType type);

const bool IsDynamicODVeh(const double v, const iflyauto::ObjectType type);

const bool IsODLivingThings(const iflyauto::ObjectType type);

const bool IsDynamicLivingThings(const iflyauto::ObjectType type);

std::vector<Eigen::VectorXd> TransformDpSpeedConstraints();

std::vector<Eigen::Vector2d> TransformQPSpeedConstraints();

const double GetDebugRefCruiseSpeed();

std::vector<Eigen::VectorXd> TransformDPSpeedOptimizationData();

std::vector<Eigen::VectorXd> TransformQPSpeedOptimizationData();

std::vector<Eigen::VectorXd> TransformStopSigns();

std::vector<Eigen::VectorXd> TransformJLTSpeedData();
}  // namespace planning