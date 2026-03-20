/*
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * Description: class swc
 */

#include "mdc/common_swc_lib/common_swc_lib_swc.h"

#include <cstring>
#include <fstream>
#include <climits>
#include <cstdlib>
#include <sys/stat.h>
#include <unistd.h>
#ifndef SCFI_SDK_X86
#include "ara/rm/rm.h"
#include "driver/ascend_hal.h"
#endif
#include "mdc/common_swc_lib_logger.h"
#include "mdc/utility/mdc_yaml_node.h"
#include "driver/ascend_hal.h"
#include "mdc/common_swc_lib/structcontainerinterface_server.h"
#include "iflyauto/struct_container/structcontainerinterface_skeleton.h"
#include "mdc/common_swc_lib/camerastructcontainerinterface_server.h"
#include "iflyauto/camera_struct_container/camerastructcontainerinterface_skeleton.h"
#include "mdc/common_swc_lib/fmalarmreceiveservice_server.h"
#include "mdc/fm/fmalarmreceiveservice_skeleton.h"
#include "mdc/common_swc_lib/clientservicecontainerinterface_server.h"
#include "iflyauto/struct_container/clientservicecontainerinterface_skeleton.h"
#include "mdc/common_swc_lib/structcontainerinterface_client.h"
#include "mdc/common_swc_lib/camerastructcontainerinterface_client.h"
#include "mdc/common_swc_lib/fmalarmreceiveservice_client.h"
#include "mdc/common_swc_lib/clientservicecontainerinterface_client.h"
#include "ara/expt_record/sig_expt.h"
#include "sig_backtrace.h"

#define RETURN_IF_ERROR(ret, str) \
if (!(ret)) { \
    LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Cannot find " << (str) << " in Config File!"; \
    return (HAF_ERROR); \
}

#define COUT_IF_ERROR(ret, str) \
if (!(ret)) { \
    LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Cannot find " << (str) << " in Config File!"; \
    return (HAF_ERROR); \
}

#define RETURN_IF_INIT_SERVICE_FAILED(ret, str, id) \
if (!(ret)) { \
    LOG_SPACE::GetLoggerIns("SWC")->LogError() << (str) << " create failed, instance ID: " << (id); \
    return false; \
}

namespace {
    /**
     * @brief 删除重复元素, 剩余元素相对位置不变
     *
     */
    template <class T>
    void StableUnique(ara::core::Vector<T>& vv)
    {
        std::unordered_map<T, bool> flag;
        for (auto it = vv.begin(); it != vv.end();) {
            if (!flag[*it]) {
                flag[*it] = true;
                ++it;
            } else {
                it = vv.erase(it);
            }
        }
    }

    /**
     * @brief 将pod数据类型装换为string以便于打印
     *
     */
    template <class T>
    ara::core::String Vec2Str(const ara::core::Vector<T>& vec)
    {
        static_assert(std::is_pod<T>::value, "Template T must be plain old data (POD) type");
        std::ostringstream oss;
        if (!vec.empty()) {
            std::copy(vec.begin(), vec.end() - 1, std::ostream_iterator<T>(oss, ", "));
            oss << vec.back();
        }
        return oss.str();
    }
}

namespace mdc {
namespace common_swc_lib {
using ara::core::String;
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
using namespace iaps::ipbm::oss_adp;
CommonSwcLibSwC::CommonSwcLibSwC()
    : initFlag_(false),
      workFlag_(true),
      initHandler_(nullptr),
      stopHandler_(nullptr),
      identifier_("SWC"),
      loggerId_("SWC")
{}

CommonSwcLibSwC::~CommonSwcLibSwC()
{
    if (workFlag_) {
        Stop();
    }
}

bool CommonSwcLibSwC::Init()
{
    
    if (initFlag_) {
        LOG_SPACE::GetLoggerIns("SWC")->LogWarn()<< "Service has been initialized";
        return false;
    }

    if (!LoadConfig()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Load config failed. Program will now exit.";
        return false;
    }

    signal_backtrace_set_file_saved_path("/opt/usr/log/app_log");
    signal_backtrace_init();
    // SigExpt& inst = SigExpt::GetInstance();
    // int ret = inst.InitSigExptProc(ExptInitMode::DEFAULT_MODE);
    // if(ret != 0)
    // {
    //     LOG_SPACE::GetLoggerIns("SWC")->LogError() << "InitSigExptProc failed, ret = " << ret << "\n";
    // }

    #ifndef SCFI_SDK_X86
    BuffCfg bufCfg{};
    const int32_t halRet = halBuffInit(&bufCfg);
    if ((DRV_ERROR_NONE != halRet) && (DRV_ERROR_REPEATED_INIT != halRet)) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Hal buffer init failed, hal error code: " << halRet;
        return false;
    }

    if (!ara::rm::RegisterHisiResource()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Register process to RM failed! Check if RM has been started.";
        return false;
    }
    #endif
    /* server agent */
    for (auto id : structContainerInterfaceServerIdMap_) {
        // auto resultToken = iflyauto::struct_container::skeleton::StructContainerInterfaceSkeleton::Preconstruct(
        //     ara::com::InstanceIdentifier(ara::core::StringView(std::to_string(id.second).c_str())),
        //     ara::com::MethodCallProcessingMode::kPoll);
        // if (resultToken.HasValue()) {
            structContainerInterfaceServerInsMap_[id.first] = std::make_shared<StructContainerInterfaceServer>(id.second);
        // } else {
        //     LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Skeleton preconstruct Failed!";
        //     return false;
        // }
        RETURN_IF_INIT_SERVICE_FAILED(
            (structContainerInterfaceServerInsMap_[id.first]!= nullptr),
            "StructContainerInterfaceSkeleton",
            id.second);
    }
    /* server agent */
    for (auto id : cameraStructContainerInterfaceServerIdMap_) {
        auto resultToken = iflyauto::camera_struct_container::skeleton::CameraStructContainerInterfaceSkeleton::Preconstruct(
            ara::com::InstanceIdentifier(ara::core::StringView(std::to_string(id.second).c_str())),
            ara::com::MethodCallProcessingMode::kPoll);
        if (resultToken.HasValue()) {
            cameraStructContainerInterfaceServerInsMap_[id.first] = std::make_shared<CameraStructContainerInterfaceServer>(id.second);
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Skeleton preconstruct Failed!";
            return false;
        }
        RETURN_IF_INIT_SERVICE_FAILED(
            (cameraStructContainerInterfaceServerInsMap_[id.first]!= nullptr),
            "CameraStructContainerInterfaceSkeleton",
            id.second);
    }
    /* server agent */
    for (auto id : fmAlarmReceiveServiceServerIdMap_) {
        auto resultToken = mdc::fm::skeleton::FmAlarmReceiveServiceSkeleton::Preconstruct(
            ara::com::InstanceIdentifier(ara::core::StringView(std::to_string(id.second).c_str())),
            ara::com::MethodCallProcessingMode::kPoll);
        if (resultToken.HasValue()) {
            fmAlarmReceiveServiceServerInsMap_[id.first] = std::make_shared<FmAlarmReceiveServiceServer>(id.second);
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Skeleton preconstruct Failed!";
            return false;
        }
        RETURN_IF_INIT_SERVICE_FAILED(
            (fmAlarmReceiveServiceServerInsMap_[id.first]!= nullptr),
            "FmAlarmReceiveServiceSkeleton",
            id.second);
    }
    /* server agent */
    for (auto id : clientServiceContainerInterfaceServerIdMap_) {
        auto resultToken = iflyauto::struct_container::skeleton::ClientServiceContainerInterfaceSkeleton::Preconstruct(
            ara::com::InstanceIdentifier(ara::core::StringView(std::to_string(id.second).c_str())),
            ara::com::MethodCallProcessingMode::kPoll);
        if (resultToken.HasValue()) {
            clientServiceContainerInterfaceServerInsMap_[id.first] = std::make_shared<ClientServiceContainerInterfaceServer>(id.second);
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Skeleton preconstruct Failed!";
            return false;
        }
        RETURN_IF_INIT_SERVICE_FAILED(
            (clientServiceContainerInterfaceServerInsMap_[id.first]!= nullptr),
            "ClientServiceContainerInterfaceSkeleton",
            id.second);
    }
    /* client agent */
    for (auto id : structContainerInterfaceClientIdMap_) {
        structContainerInterfaceClientInsMap_[id.first] = std::make_shared<StructContainerInterfaceClient>(id.second);
        RETURN_IF_INIT_SERVICE_FAILED(
            (structContainerInterfaceClientInsMap_[id.first]!= nullptr),
            "StructContainerInterfaceProxy",
            id.second);
    }
    /* client agent */
    for (auto id : cameraStructContainerInterfaceClientIdMap_) {
        cameraStructContainerInterfaceClientInsMap_[id.first] = std::make_shared<CameraStructContainerInterfaceClient>(id.second);
        RETURN_IF_INIT_SERVICE_FAILED(
            (cameraStructContainerInterfaceClientInsMap_[id.first]!= nullptr),
            "CameraStructContainerInterfaceProxy",
            id.second);
    }
    /* client agent */
    for (auto id : fmAlarmReceiveServiceClientIdMap_) {
        fmAlarmReceiveServiceClientInsMap_[id.first] = std::make_shared<FmAlarmReceiveServiceClient>(id.second);
        RETURN_IF_INIT_SERVICE_FAILED(
            (fmAlarmReceiveServiceClientInsMap_[id.first]!= nullptr),
            "FmAlarmReceiveServiceProxy",
            id.second);
    }
    /* client agent */
    for (auto id : clientServiceContainerInterfaceClientIdMap_) {
        clientServiceContainerInterfaceClientInsMap_[id.first] = std::make_shared<ClientServiceContainerInterfaceClient>(id.second);
        RETURN_IF_INIT_SERVICE_FAILED(
            (clientServiceContainerInterfaceClientInsMap_[id.first]!= nullptr),
            "ClientServiceContainerInterfaceProxy",
            id.second);
    }

    if ((initHandler_) && (!initHandler_())) {
        return false;
    }

    initFlag_ = true;
    return true;
}

void CommonSwcLibSwC::Stop()
{
    workFlag_ = false;
    /* server agent */
    for (auto id : structContainerInterfaceServerIdMap_) {
        if (structContainerInterfaceServerInsMap_[id.first]) {
            structContainerInterfaceServerInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }
    for (auto id : cameraStructContainerInterfaceServerIdMap_) {
        if (cameraStructContainerInterfaceServerInsMap_[id.first]) {
            cameraStructContainerInterfaceServerInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }
    for (auto id : fmAlarmReceiveServiceServerIdMap_) {
        if (fmAlarmReceiveServiceServerInsMap_[id.first]) {
            fmAlarmReceiveServiceServerInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }
    for (auto id : clientServiceContainerInterfaceServerIdMap_) {
        if (clientServiceContainerInterfaceServerInsMap_[id.first]) {
            clientServiceContainerInterfaceServerInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }

    /* client agent */
    for (auto id : structContainerInterfaceClientIdMap_) {
        if (structContainerInterfaceClientInsMap_[id.first]) {
            structContainerInterfaceClientInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }
    for (auto id : cameraStructContainerInterfaceClientIdMap_) {
        if (cameraStructContainerInterfaceClientInsMap_[id.first]) {
            cameraStructContainerInterfaceClientInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }
    for (auto id : fmAlarmReceiveServiceClientIdMap_) {
        if (fmAlarmReceiveServiceClientInsMap_[id.first]) {
            fmAlarmReceiveServiceClientInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }
    for (auto id : clientServiceContainerInterfaceClientIdMap_) {
        if (clientServiceContainerInterfaceClientInsMap_[id.first]) {
            clientServiceContainerInterfaceClientInsMap_[id.first]->Stop();
        } else {
            LOG_SPACE::GetLoggerIns("SWC")->LogError()<< "Cannot stop because " << id.first << " construction failed";
        }
    }

    if (stopHandler_) {
        stopHandler_();
    }
    return;
}


std::shared_ptr<StructContainerInterfaceServer> CommonSwcLibSwC::GetStructContainerInterfaceServer(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(structContainerInterfaceServerInsMapMtx_);
    if (structContainerInterfaceServerInsMap_.find(portName) == structContainerInterfaceServerInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(structContainerInterfaceServerInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return structContainerInterfaceServerInsMap_[portName];
}
std::shared_ptr<StructContainerInterfaceClient> CommonSwcLibSwC::GetStructContainerInterfaceClient(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(structContainerInterfaceClientInsMapMtx_);
    if (structContainerInterfaceClientInsMap_.find(portName) == structContainerInterfaceClientInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(structContainerInterfaceClientInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return structContainerInterfaceClientInsMap_[portName];
}
std::shared_ptr<CameraStructContainerInterfaceServer> CommonSwcLibSwC::GetCameraStructContainerInterfaceServer(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(cameraStructContainerInterfaceServerInsMapMtx_);
    if (cameraStructContainerInterfaceServerInsMap_.find(portName) == cameraStructContainerInterfaceServerInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(cameraStructContainerInterfaceServerInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return cameraStructContainerInterfaceServerInsMap_[portName];
}
std::shared_ptr<CameraStructContainerInterfaceClient> CommonSwcLibSwC::GetCameraStructContainerInterfaceClient(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(cameraStructContainerInterfaceClientInsMapMtx_);
    if (cameraStructContainerInterfaceClientInsMap_.find(portName) == cameraStructContainerInterfaceClientInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(cameraStructContainerInterfaceClientInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return cameraStructContainerInterfaceClientInsMap_[portName];
}
std::shared_ptr<FmAlarmReceiveServiceServer> CommonSwcLibSwC::GetFmAlarmReceiveServiceServer(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(fmAlarmReceiveServiceServerInsMapMtx_);
    if (fmAlarmReceiveServiceServerInsMap_.find(portName) == fmAlarmReceiveServiceServerInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(fmAlarmReceiveServiceServerInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return fmAlarmReceiveServiceServerInsMap_[portName];
}
std::shared_ptr<FmAlarmReceiveServiceClient> CommonSwcLibSwC::GetFmAlarmReceiveServiceClient(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(fmAlarmReceiveServiceClientInsMapMtx_);
    if (fmAlarmReceiveServiceClientInsMap_.find(portName) == fmAlarmReceiveServiceClientInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(fmAlarmReceiveServiceClientInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return fmAlarmReceiveServiceClientInsMap_[portName];
}
std::shared_ptr<ClientServiceContainerInterfaceServer> CommonSwcLibSwC::GetClientServiceContainerInterfaceServer(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(clientServiceContainerInterfaceServerInsMapMtx_);
    if (clientServiceContainerInterfaceServerInsMap_.find(portName) == clientServiceContainerInterfaceServerInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(clientServiceContainerInterfaceServerInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return clientServiceContainerInterfaceServerInsMap_[portName];
}
std::shared_ptr<ClientServiceContainerInterfaceClient> CommonSwcLibSwC::GetClientServiceContainerInterfaceClient(const ara::core::String& portName)
{
    std::lock_guard<std::mutex> lck(clientServiceContainerInterfaceClientInsMapMtx_);
    if (clientServiceContainerInterfaceClientInsMap_.find(portName) == clientServiceContainerInterfaceClientInsMap_.end()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << "does not exist.";
        return nullptr;
    }
    if (!(clientServiceContainerInterfaceClientInsMap_[portName]->Init())) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << portName << " init failed.";
        return nullptr;
    }
    return clientServiceContainerInterfaceClientInsMap_[portName];
}

bool CommonSwcLibSwC::LoadConfig()
{
    /* 通过环境变量获取配置文件路径 */
    char* path = secure_getenv("CM_CONFIG_FILE_PATH");
    if (path == nullptr) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Empty process config path!";
        return false;
    }
    char fileRealPath[PATH_MAX + 1] = {0};
    if (realpath(path, &fileRealPath[0]) == nullptr) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Invalid process config path!";
        return false;
    }

    configFile_ = path;
    configFile_ += "/scfi_mapping.yaml";

    /* 配置文件路径为空或非yaml文件 */
    if (configFile_.empty()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "config path must be a valid yaml file!";
        return false;
    }

    if (!DoesFileExist(configFile_)) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << configFile_ << " does not exist.";
        return false;
    }

    auto config = std::make_unique<MdcYamlNode>(configFile_);
    if (!config->GetValue<std::string>("identifier", identifier_)) {
        LOG_SPACE::GetLoggerIns("SWC")->LogWarn() << "Warning: swc identifier not specified, use default config.";
    }

    if (!ParsingInstanceId(config)) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Failed to parse instance id!";
        return false;
    }
    return true;
}

bool CommonSwcLibSwC::DoesFileExist(const ara::core::String& filePath)
{
    struct stat fileInfo = {};
    if (memset_s(&fileInfo, sizeof(fileInfo), 0, sizeof(fileInfo)) != 0) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "DoesFileExist Initialization of fileInfo failed.";
        return false;
    }
    if (lstat(filePath.c_str(), &fileInfo) != 0) {
        return false;
    }
    if (!IsDirType(fileInfo.st_mode)) {
        return true;
    }
    return false;
}

bool CommonSwcLibSwC::IsDirType(const mode_t& fileMode)
{
    if (S_ISDIR(fileMode)) {
        return true;
    }
    return false;
}

bool CommonSwcLibSwC::ParsingInstanceId(const std::unique_ptr<MdcYamlNode>& config)
{
    if (!config->HasKeyValue("common_swc_lib")) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Cannot find common_swc_lib in config file!";
        return false;
    }
    auto swcConfig = (*(config.get()))["common_swc_lib"];
    /* server agent */
    if (!swcConfig.HasKeyValue("sendInstanceID")) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Cannot find sendInstanceID in config file!";
        return false;
    }
    swcConfig["sendInstanceID"]["StructContainerInterface"].GetValue<std::vector<uint32_t>>(
        "instanceId", structContainerInterfaceServerIdVec_);
    StableUnique(structContainerInterfaceServerIdVec_);
    swcConfig["sendInstanceID"]["StructContainerInterface"].GetValue<std::vector<std::string>>(
        "portName", structContainerInterfaceServerPortVec_);
    StableUnique(structContainerInterfaceServerPortVec_);
    if (structContainerInterfaceServerIdVec_.size() != structContainerInterfaceServerPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < structContainerInterfaceServerIdVec_.size(); ++i) {
        structContainerInterfaceServerIdMap_[structContainerInterfaceServerPortVec_[i]] = structContainerInterfaceServerIdVec_[i];
    }
    swcConfig["sendInstanceID"]["CameraStructContainerInterface"].GetValue<std::vector<uint32_t>>(
        "instanceId", cameraStructContainerInterfaceServerIdVec_);
    StableUnique(cameraStructContainerInterfaceServerIdVec_);
    swcConfig["sendInstanceID"]["CameraStructContainerInterface"].GetValue<std::vector<std::string>>(
        "portName", cameraStructContainerInterfaceServerPortVec_);
    StableUnique(cameraStructContainerInterfaceServerPortVec_);
    if (cameraStructContainerInterfaceServerIdVec_.size() != cameraStructContainerInterfaceServerPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < cameraStructContainerInterfaceServerIdVec_.size(); ++i) {
        cameraStructContainerInterfaceServerIdMap_[cameraStructContainerInterfaceServerPortVec_[i]] = cameraStructContainerInterfaceServerIdVec_[i];
    }
    swcConfig["sendInstanceID"]["FmAlarmReceiveService"].GetValue<std::vector<uint32_t>>(
        "instanceId", fmAlarmReceiveServiceServerIdVec_);
    StableUnique(fmAlarmReceiveServiceServerIdVec_);
    swcConfig["sendInstanceID"]["FmAlarmReceiveService"].GetValue<std::vector<std::string>>(
        "portName", fmAlarmReceiveServiceServerPortVec_);
    StableUnique(fmAlarmReceiveServiceServerPortVec_);
    if (fmAlarmReceiveServiceServerIdVec_.size() != fmAlarmReceiveServiceServerPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < fmAlarmReceiveServiceServerIdVec_.size(); ++i) {
        fmAlarmReceiveServiceServerIdMap_[fmAlarmReceiveServiceServerPortVec_[i]] = fmAlarmReceiveServiceServerIdVec_[i];
    }
    swcConfig["sendInstanceID"]["ClientServiceContainerInterface"].GetValue<std::vector<uint32_t>>(
        "instanceId", clientServiceContainerInterfaceServerIdVec_);
    StableUnique(clientServiceContainerInterfaceServerIdVec_);
    swcConfig["sendInstanceID"]["ClientServiceContainerInterface"].GetValue<std::vector<std::string>>(
        "portName", clientServiceContainerInterfaceServerPortVec_);
    StableUnique(clientServiceContainerInterfaceServerPortVec_);
    if (clientServiceContainerInterfaceServerIdVec_.size() != clientServiceContainerInterfaceServerPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < clientServiceContainerInterfaceServerIdVec_.size(); ++i) {
        clientServiceContainerInterfaceServerIdMap_[clientServiceContainerInterfaceServerPortVec_[i]] = clientServiceContainerInterfaceServerIdVec_[i];
    }

    /* client agent */
    if (!swcConfig.HasKeyValue("recvInstanceID")) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "Cannot find recvInstanceID in config file!";
        return false;
    }
    swcConfig["recvInstanceID"]["StructContainerInterface"].GetValue<std::vector<uint32_t>>(
        "instanceId", structContainerInterfaceClientIdVec_);
    StableUnique(structContainerInterfaceClientIdVec_);
    swcConfig["recvInstanceID"]["StructContainerInterface"].GetValue<std::vector<std::string>>(
        "portName", structContainerInterfaceClientPortVec_);
    StableUnique(structContainerInterfaceClientPortVec_);
    if (structContainerInterfaceClientIdVec_.size() != structContainerInterfaceClientPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < structContainerInterfaceClientIdVec_.size(); ++i) {
        structContainerInterfaceClientIdMap_[structContainerInterfaceClientPortVec_[i]] = structContainerInterfaceClientIdVec_[i];
    }
    swcConfig["recvInstanceID"]["CameraStructContainerInterface"].GetValue<std::vector<uint32_t>>(
        "instanceId", cameraStructContainerInterfaceClientIdVec_);
    StableUnique(cameraStructContainerInterfaceClientIdVec_);
    swcConfig["recvInstanceID"]["CameraStructContainerInterface"].GetValue<std::vector<std::string>>(
        "portName", cameraStructContainerInterfaceClientPortVec_);
    StableUnique(cameraStructContainerInterfaceClientPortVec_);
    if (cameraStructContainerInterfaceClientIdVec_.size() != cameraStructContainerInterfaceClientPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < cameraStructContainerInterfaceClientIdVec_.size(); ++i) {
        cameraStructContainerInterfaceClientIdMap_[cameraStructContainerInterfaceClientPortVec_[i]] = cameraStructContainerInterfaceClientIdVec_[i];
    }
    swcConfig["recvInstanceID"]["FmAlarmReceiveService"].GetValue<std::vector<uint32_t>>(
        "instanceId", fmAlarmReceiveServiceClientIdVec_);
    StableUnique(fmAlarmReceiveServiceClientIdVec_);
    swcConfig["recvInstanceID"]["FmAlarmReceiveService"].GetValue<std::vector<std::string>>(
        "portName", fmAlarmReceiveServiceClientPortVec_);
    StableUnique(fmAlarmReceiveServiceClientPortVec_);
    if (fmAlarmReceiveServiceClientIdVec_.size() != fmAlarmReceiveServiceClientPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < fmAlarmReceiveServiceClientIdVec_.size(); ++i) {
        fmAlarmReceiveServiceClientIdMap_[fmAlarmReceiveServiceClientPortVec_[i]] = fmAlarmReceiveServiceClientIdVec_[i];
    }
    swcConfig["recvInstanceID"]["ClientServiceContainerInterface"].GetValue<std::vector<uint32_t>>(
        "instanceId", clientServiceContainerInterfaceClientIdVec_);
    StableUnique(clientServiceContainerInterfaceClientIdVec_);
    swcConfig["recvInstanceID"]["ClientServiceContainerInterface"].GetValue<std::vector<std::string>>(
        "portName", clientServiceContainerInterfaceClientPortVec_);
    StableUnique(clientServiceContainerInterfaceClientPortVec_);
    if (clientServiceContainerInterfaceClientIdVec_.size() != clientServiceContainerInterfaceClientPortVec_.size()) {
        LOG_SPACE::GetLoggerIns("SWC")->LogError() << "The port and instance mapping information is incorrect.";
        return false;
    }
    for (size_t i = 0; i < clientServiceContainerInterfaceClientIdVec_.size(); ++i) {
        clientServiceContainerInterfaceClientIdMap_[clientServiceContainerInterfaceClientPortVec_[i]] = clientServiceContainerInterfaceClientIdVec_[i];
    }
    return true;
}
}  /* namespace common_swc_lib */
}  /* namespace mdc */