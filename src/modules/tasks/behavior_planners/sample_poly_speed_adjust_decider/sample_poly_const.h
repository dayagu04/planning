#pragma once

#include <cstdint>
namespace planning {
enum SampleSpeedStatus { OK = 0, kEgoPredSExceedLeadOne };

enum SampleScene {
  NormalSampleScene = 0,
  PurseFlowVelScene,
  DecelerationPriorityScene
};
enum TrafficDensityStatus { LineClear, Congested };

constexpr double kMaxPenalty = 10000.0;
constexpr double kMaxMergeDistance = 1000.0;
constexpr double kDistanceToMapRequestPoint = 120.0;
constexpr double kFollowSpeedBenchmark = 10.0;
constexpr double kStopLineRiskPenaltyDis = 30.0;
constexpr double kStopLineBasicPenaltyDis = 150.0;
constexpr double kStopLineNormalPenaltyDis = 75.0;
constexpr int32_t kNoAgentId = -100;
constexpr double kBasicSafeDistance = 7.5;
constexpr double kLeadOneSafeDistance = 13.5;
constexpr double kMinSafeDistance = 7.0;
constexpr double kSafeVelEpsion = 0.5;
constexpr double kPenaltyVel = 1.0;
constexpr double kPlanningDuration = 5.0;
constexpr double kStaticObjVel = 1.2;
constexpr double kTimeResolution = 0.1;
constexpr int32_t kSampleSpaceReserveNum =
    (kPlanningDuration) / kTimeResolution + 1;
constexpr double kPlanningStep = 0.2;
constexpr double kMaxPathLength = 400.0;
constexpr double kEgoHalfLength = 2.6;
const int kPlanningHorizions = 26;
constexpr double kMaxVelVariableValue = 7.5;
constexpr double kMaxVelVariableValueInverse = 1 / kMaxVelVariableValue;
constexpr double kMatchGapCenterDis = 2.5;
constexpr double kJudePurseFlowVelValue = 12.0 / 3.6;
constexpr int kNormalToHoverThreshold = 25;
constexpr int kHoverToNormalThreshold = 15;
constexpr int kJudgeCongestedVehNumThreshold = 4;
constexpr double kVehDensityDistanceThreshold = 70.0;
constexpr double kJudgeCongestedSceneDensity = 0.05;
constexpr double kMatchGapVelPenaltyThreshold = 1.5;
constexpr double kAgentNoValidVel = 100.0;
constexpr double kAgentNoValidDis = 100.0;
constexpr double kZeroEpsilon = 1e-6;
constexpr double kAccPenaltyLimit = 1.3;
constexpr double kAccPenaltyScaleFactor = 3.0;
constexpr double kEgoVelMin = 1.0;
constexpr double kEgoVelMax = 35.0;
constexpr double kExtraExpandDisMax = 5.0;
constexpr double kExtraExpandDisMin = 3.5;
}  // namespace planning