/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: MDC平台AP标准框架
 */
#include "applicationswcexecutable2_manager.h"
#include "mdc/applicationswcexecutable2_logger.h"

namespace mdc {
namespace applicationswcexecutable2 {
/**
* @brief 请在此方法中添加组件代码初始化逻辑，若有阻塞操作，建议开启新线程来运行阻塞操作
*
*/
bool Applicationswcexecutable2Manager::OnInitialize()
{
    componentPtr_ = std::make_unique<iflyauto::ComponentWrapper>();
    if (!componentPtr_->Init()) {
        return false;
    }
    return true;
}

/**
* @brief 请在此方法中添加组件代码运行逻辑，若有阻塞操作，建议开启新线程来运行阻塞操作
*
*/
void Applicationswcexecutable2Manager::Run()
{
    componentPtr_->Run();
    return;
}

/**
* @brief 若组件作为CM通信服务端，请在此处停止服务
*
*/
void Applicationswcexecutable2Manager::OnTerminate()
{
    componentPtr_->Stop();
    return;
}
} /* namespace applicationswcexecutable2 */
} /* namespace mdc */