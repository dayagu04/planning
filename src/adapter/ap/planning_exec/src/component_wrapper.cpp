#include "component_wrapper.h"

#include "applicationswcexecutable2_manager.h"
#include "ehr_sdmap.pb.h"
#include "iflyauto_parse_file.h"
#include "iflyauto_timer_component.h"
#include "interface/src/c/fusion_occupancy_objects_c.h"
#include "mdc/applicationswcexecutable2_logger.h"

namespace iflyauto {
using SPL_LOG_SPACE =
    mdc::applicationswcexecutable2::Applicationswcexecutable2Logger;

ComponentWrapper::ComponentWrapper()
    : swc_ptr_(std::make_unique<mdc::common_swc_lib::CommonSwcLibSwC>()),
      work_flag_(true) {
  mviz::NodeHandle::Connect();
}

ComponentWrapper::~ComponentWrapper() {
  mviz::NodeHandle::Close();
  if (work_flag_) {
    Stop();
  }
}

/**
 * @brief 用户在初始化过程中自定义操作
 *
 */
bool ComponentWrapper::InitHandle() {
  SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo()
      << "Some operations during initialization.";
  return true;
}

/**
 * @brief 用户在停止服务时自定义操作
 *
 */
void ComponentWrapper::StopHandle() {
  SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo()
      << "Some operations during stopping";
}

static mdc::fm::AlarmInfo FminfoToAlarmInfo(const iflyauto::FmInfo &fminfo) {
  mdc::fm::AlarmInfo alarm_info = {};
  alarm_info.alarmId = fminfo.alarmId;
  alarm_info.alarmObj = fminfo.alarmObj;
  alarm_info.clss = fminfo.clss;
  alarm_info.level = fminfo.level;
  alarm_info.status = fminfo.status;
  alarm_info.time = fminfo.time;
  alarm_info.desc = ::String(fminfo.desc);
  return alarm_info;
}

void ComponentWrapper::SendAlarmInfo(const ara::core::String &portName,
                                     const mdc::fm::AlarmInfo &alarmInfo) {
  if (!swc_ptr_) {
    SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
        << "The swc object is empty.";
    return;
  }

  /* 获取发送端服务，其中portName对应MMC上配置的此应用对应的发送端portName */
  auto clientPtr = swc_ptr_->GetFmAlarmReceiveServiceClient(portName);
  if (!clientPtr) {
    SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
        << "Failed to initialize the instance: " << portName;
    return;
  }
  if (clientPtr->ReportAlarm(alarmInfo)) {
    mdc::fm::ReportAlarmOutput res = clientPtr->GetReportAlarmData();
    std::cout << "receive server result: " << res.result << std::endl;
  } else {
    std::cout << "Request failed!" << std::endl;
  }
}

void ComponentWrapper::InitClient() {
  auto structContainerInterfaceClientPortVec =
      swc_ptr_->GetStructContainerInterfaceClientVec();
  std::unordered_set<std::string> client_set(
      structContainerInterfaceClientPortVec.begin(),
      structContainerInterfaceClientPortVec.end());
  while (!client_set.empty()) {
    for (const auto &portName : structContainerInterfaceClientPortVec) {
      auto clientPtr = swc_ptr_->GetStructContainerInterfaceClient(portName);
      if (!clientPtr) {
        SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
            << "Failed to initialize the instance: " << portName;
        continue;
      }
#define REGISTER_CLIENT_HANDLER(name, func, type)               \
  do {                                                          \
    if (portName == name) {                                     \
      clientPtr->RegisterEventNotifyHandler(                    \
          [this](const struct_container::eventDataType &data) { \
            SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo()       \
                << "received event data" << name;               \
            const auto struct_ptr =                             \
                reinterpret_cast<const type *>(data.c_str());   \
            component_ptr_->func(*struct_ptr);                  \
          });                                                   \
      client_set.erase(portName);                               \
    }                                                           \
  } while (0);

      REGISTER_CLIENT_HANDLER("IflytekFusionObjects", FeedFusionObjects,
                              FusionObjectsInfo);
      REGISTER_CLIENT_HANDLER("IflytekFusionOccupancyObjects",
                              FeedFusionOccupancyObjects,
                              FusionOccupancyObjectsInfo);
      REGISTER_CLIENT_HANDLER("IflytekFusionRoadFusion", FeedFusionRoad,
                              RoadInfo);
      REGISTER_CLIENT_HANDLER("IflytekLocalizationEgoPose",
                              FeedLocalizationEstimateOutput,
                              LocalizationEstimate);
      REGISTER_CLIENT_HANDLER("IflytekLocalizationEgomotion",
                              FeedLocalizationOutput, IFLYLocalization);
      REGISTER_CLIENT_HANDLER("IflytekPredictionPredictionResult",
                              FeedPredictionResult, PredictionResult);
      REGISTER_CLIENT_HANDLER("IflytekVehicleService", FeedVehicleService,
                              VehicleServiceOutputInfo);
      REGISTER_CLIENT_HANDLER("IflytekControlControlCommand",
                              FeedControlCommand, ControlOutput);
      REGISTER_CLIENT_HANDLER("IflytekHmiMcuInner", FeedHmiMcuInner,
                              HmiMcuInner);
      REGISTER_CLIENT_HANDLER("IflytekFusionParkingSlot", FeedParkingFusion,
                              ParkingFusionInfo);
      REGISTER_CLIENT_HANDLER("IflytekSystemStateSocState",
                              FeedFuncStateMachine, FuncStateMachine);
      REGISTER_CLIENT_HANDLER("IflytekUssUsswaveInfo", FeedUssWaveInfo,
                              UssWaveInfo);
      REGISTER_CLIENT_HANDLER("IflytekUssUssPerceptInfo", FeedUssPerceptInfo,
                              UssPerceptInfo);

      SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
          << "RegisterEvent: " << portName;
      if (portName == "IflytekEhrSdmapInfo") {
        SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError() << "EEEEEE " << portName;
        clientPtr->RegisterEventNotifyHandler(
            [this](const struct_container::eventDataType &data) {
              SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo()
                  << "received event data";
              auto sd_map_msg = std::make_shared<SdMapSwtx::SdMap>();
              sd_map_msg->ParseFromString(data);
              component_ptr_->FeedSdMap(sd_map_msg);
            });
        client_set.erase(portName);
      }

#undef REGISTER_CLIENT_HANDLER
    }
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}

/**
 * @brief
 * 请在此方法中添加组件代码初始化逻辑，若有阻塞操作，建议开启新线程来运行阻塞操作
 *
 */
bool ComponentWrapper::Init() {
  SPL_LOG_SPACE::InitAppLogging();
  SPL_LOG_SPACE::InitLoggerCtx("SPL", "sample log contex");

  if (!swc_ptr_) {
    SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
        << "Failed to create the SWC object.";
    return false;
  }
  swc_ptr_->SetInitCallback(std::bind(&ComponentWrapper::InitHandle));
  swc_ptr_->SetStopCallback(std::bind(&ComponentWrapper::StopHandle));
  if (!swc_ptr_->Init()) {
    SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
        << "SWC initialization failed.";
    return false;
  }

  SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo()
      << "planning::PlanningAdapter size: "
      << sizeof(planning::PlanningAdapter);
  component_ptr_ = std::make_unique<planning::PlanningAdapter>();
  SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo() << "component_ptr_->Init";
  component_ptr_->Init();
  SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo() << "component_ptr_->Init done";
  std::thread t_client(&ComponentWrapper::InitClient, this);
  t_client.detach();

  auto structContainerInterfaceServerPortVec =
      swc_ptr_->GetStructContainerInterfaceServerVec();
  for (const auto &portName : structContainerInterfaceServerPortVec) {
    auto serverPtr = swc_ptr_->GetStructContainerInterfaceServer(portName);
    if (!serverPtr) {
      SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
          << "Failed to initialize the instance: " << portName;
      continue;
    }
#define REGISTER_SERVER_HANDLER(name, original_name, func, type)             \
  do {                                                                       \
    if (portName == name) {                                                  \
      auto mviz_pub = std::make_shared<ContainerPubChannel>(original_name);  \
      component_ptr_->func(                                                  \
          [serverPtr,                                                        \
           mviz_pub](const std::shared_ptr<iflyauto::StructContainer> msg) { \
            auto const_char =                                                \
                reinterpret_cast<const char *>(msg->payload().c_str());      \
            auto data = std::make_shared<struct_container::eventDataType>(   \
                const_char, sizeof(type));                                   \
            serverPtr->SendEventData(data);                                  \
            mviz_pub->SendToMviz((const uint8_t *)const_char, sizeof(type)); \
          });                                                                \
    }                                                                        \
  } while (0)

    REGISTER_SERVER_HANDLER("IflytekPlanningPlan", "/iflytek/planning/plan",
                            RegisterOutputWriter, iflyauto::PlanningOutput);
    REGISTER_SERVER_HANDLER("IflytekPlanningHmi", "/iflytek/planning/hmi",
                            RegisterHMIOutputInfoWriter,
                            iflyauto::PlanningHMIOutputInfoStr);
    /*
    REGISTER_SERVER_HANDLER("IflytekPlanningDebugInfo",
    "/iflytek/planning/debug_info", RegisterDebugInfoWriter,
                            planning::common::PlanningDebugInfo);
    */

    {
      auto mviz_pub =
          std::make_shared<ContainerPubChannel>("/iflytek/planning/debug_info");
      component_ptr_->RegisterDebugInfoWriter(
          [mviz_pub](const std::shared_ptr<iflyauto::StructContainer> msg) {
            mviz_pub->SendToMviz((const uint8_t *)(msg->payload().c_str()),
                                 msg->payload().size());
          });
    }
#undef REGISTER_SERVER_HANDLER
  }

  component_ptr_->RegisterFmInfoWriter([this](const iflyauto::FmInfo &fmInfo) {
    mdc::fm::AlarmInfo alarmInfo = FminfoToAlarmInfo(fmInfo);
    SendAlarmInfo("IflytekAlarmPlanning", alarmInfo);
  });

  return true;
}

/**
 * @brief
 * 请在此方法中添加组件代码运行逻辑，若有阻塞操作，建议开启新线程来运行阻塞操作
 *
 */
void ComponentWrapper::Run() {
  timer_ptr_ = std::make_unique<TimerComponent>(
      10, [this]() { this->component_ptr_->Proc(); });
}

/**
 * @brief 若组件作为CM通信服务端，请在此处停止服务
 *
 */
void ComponentWrapper::Stop() {
  work_flag_ = false;

  SPL_LOG_SPACE::GetLoggerIns("SPL")->LogInfo()
      << "Thread resources reclaimed successfully..";
  if (!swc_ptr_) {
    SPL_LOG_SPACE::GetLoggerIns("SPL")->LogError()
        << "The swc object is empty.";
    return;
  }
  return;
}

}  // namespace iflyauto