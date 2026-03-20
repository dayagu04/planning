/*******************************************************************************
** Copyright (C) iflytek Technologies (2024)                                  **
**                                                                            **
** All rights reserved.                                                       **
**                                                                            **
** This document contains proprietary information belonging to iflytek        **
** Technologies. Passing on and copying of this document, and communication   **
** of its contents is not permitted without prior written authorization.      **
**                                                                            **
*******************************************************************************/
#ifndef IFLYAUTO_SEND_ALARM_INFO_H
#define IFLYAUTO_SEND_ALARM_INFO_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <filesystem>
#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include "iflyadlog/logger.h"
#include "iflyauto_time.h"
#include "../../../../src/c/fm_info_c.h"
#include "mdc/common_swc_lib/common_swc_lib_swc.h"
#include "mdc/common_swc_lib/fmalarmreceiveservice_client.h"

namespace iflyauto {

/*
 * 线程安全队列
 * T为队列元素类型
 * 因为有std::mutex和std::condition_variable类成员,所以此类不支持复制构造函数也不支持赋值操作符(=)
 * */
template <typename T>
class threadsafe_queue {
 private:
  mutable std::mutex mut;
  mutable std::condition_variable data_cond;
  using queue_type = std::queue<T>;
  queue_type data_queue;
  // 通知线程是否关闭
  bool flag = false;

 public:
  using value_type = typename queue_type::value_type;
  using container_type = typename queue_type::container_type;
  threadsafe_queue() = default;
  threadsafe_queue(const threadsafe_queue &) = delete;
  threadsafe_queue &operator=(const threadsafe_queue &) = delete;
  /*
   * 使用迭代器为参数的构造函数,适用所有容器对象
   * */
  template <typename _InputIterator>
  threadsafe_queue(_InputIterator first, _InputIterator last) {
    for (auto itor = first; itor != last; ++itor) {
      data_queue.push(*itor);
    }
  }
  explicit threadsafe_queue(const container_type &c) : data_queue(c) {}
  /*
   * 使用初始化列表为参数的构造函数
   * */
  threadsafe_queue(std::initializer_list<value_type> list)
      : threadsafe_queue(list.begin(), list.end()) {}
  /*
   * 将元素加入队列
   * */
  void push(const value_type &new_value) {
    std::lock_guard<std::mutex> lk(mut);
    data_queue.push(std::move(new_value));
    data_cond.notify_one();
  }
  /*
   * 从队列中弹出一个元素,如果队列为空就阻塞
   * */
  value_type wait_and_pop() {
    std::unique_lock<std::mutex> lk(mut);
    data_cond.wait(lk, [this] { return !this->data_queue.empty() || flag; });
    if (flag) {
      value_type value;
      return value;
    }
    auto value = std::move(data_queue.front());
    data_queue.pop();
    return value;
  }
  /*
   * 从队列中弹出一个元素,如果队列为空返回false
   * */
  bool try_pop(value_type &value) {
    std::lock_guard<std::mutex> lk(mut);
    if (data_queue.empty()) return false;
    value = std::move(data_queue.front());
    data_queue.pop();
    return true;
  }
  /*
   * 返回队列是否为空
   * */
  auto empty() const -> decltype(data_queue.empty()) {
    std::lock_guard<std::mutex> lk(mut);
    return data_queue.empty();
  }
  /*
   * 返回队列中元素数个
   * */
  auto size() const -> decltype(data_queue.size()) {
    std::lock_guard<std::mutex> lk(mut);
    return data_queue.size();
  }
  /*
   * 通知线程关闭
   * */
  void notify_stop() {
    flag = true;
    data_cond.notify_all();
  }
};

class ReportAlarmInfo {
 public:
     ~ReportAlarmInfo(){
         if (workFlag_.load()){
            StopReportAlarmInfo();
         }
     }

  /*
   * 获取单例
   * */
  static ReportAlarmInfo &GetInst() {
    static ReportAlarmInfo instanceReportAlarmInfo;
    return instanceReportAlarmInfo;
  }

  /*
   * 初始化函数
   * */
  void Init(std::string portname, std::string threadname,
            mdc::common_swc_lib::CommonSwcLibSwC *swcPtr) {
    workFlag_.store(true);
    thread_ptr_ = std::make_unique<std::thread>(
        std::bind(&ReportAlarmInfo::ReportAlarmInfoRun, this));
    pthread_setname_np(thread_ptr_->native_handle(), threadname.c_str());
    portName_ = portname;
    swcPtr_ = swcPtr;

    triggerFile_ = "/tmp/" + portname + ".json";
    // 创建json文件triggerFile_
    std::ofstream ofs(triggerFile_);
    if (ofs.is_open()) {
        ofs << "[{}]";  // 写入一个空的 JSON 对象
        ofs.close();
        AINFO << "Success to create trigger file: " << triggerFile_;
    } else {
        AERROR << "Failed to create trigger file: " << triggerFile_;
    }

    trigger_thread_ptr_ = std::make_unique<std::thread>(
        std::bind(&ReportAlarmInfo::MonitorFileTrigger, this));
    pthread_setname_np(trigger_thread_ptr_->native_handle(), "Fm_Trigger");
  }

  /*
   * 停止上报告警信息
   * */
  void StopReportAlarmInfo() {
    workFlag_.store(false);
    alarmInfoQueue_.notify_stop();
    if ((thread_ptr_ != nullptr) && (thread_ptr_->joinable())) {
      thread_ptr_->join();
    }
    thread_ptr_ = nullptr;

    if ((trigger_thread_ptr_ != nullptr) && (trigger_thread_ptr_->joinable())) {
      trigger_thread_ptr_->join();
    }
    trigger_thread_ptr_ = nullptr;
  }

  /*
   * 上报告警信息
   * */
  void PushAlarmInfo(const iflyauto::FmInfo &fmInfo) {
    mdc::fm::AlarmInfo alarminfo = FminfoToAlarmInfo(fmInfo);
    alarmInfoQueue_.push(alarminfo);
  }

 private:
  ReportAlarmInfo() = default;
  ReportAlarmInfo(const ReportAlarmInfo &) = delete;
  ReportAlarmInfo &operator=(const ReportAlarmInfo &) = delete;

  /*
   * 上报告警线程函数
   * */
  void ReportAlarmInfoRun() {
    while (workFlag_) {
      ::mdc::fm::AlarmInfo alarmInfo = alarmInfoQueue_.wait_and_pop();
      if (!workFlag_) {
        return;
      }
      SendAlarmInfo(alarmInfo);
    }
  }

  /*
   * 发送告警信息
   * */
  void SendAlarmInfo(const ::mdc::fm::AlarmInfo &alarminfo) {
    if (!swcPtr_) {
      return;
    }

    /* 获取发送端服务，其中portName对应MMC上配置的此应用对应的发送端portName
     * <关键接口> */
    auto clientPtr = swcPtr_->GetFmAlarmReceiveServiceClient(portName_);
    if (!clientPtr) {
      return;
    }
    if (clientPtr->ReportAlarm(alarminfo)) {
      (void)clientPtr->GetReportAlarmData();
    } else {
      AINFO << "ReportAlarm failed!";
    }
  }

  mdc::fm::AlarmInfo FminfoToAlarmInfo(const iflyauto::FmInfo &fminfo) {
    mdc::fm::AlarmInfo alarm_info{};
    alarm_info.alarmId = fminfo.alarmId;
    alarm_info.alarmObj = fminfo.alarmObj;
    alarm_info.clss = fminfo.clss;
    alarm_info.level = fminfo.level;
    alarm_info.status = fminfo.status;
    alarm_info.time = fminfo.time;
    alarm_info.desc = ::String(fminfo.desc);
    return alarm_info;
  }

void MonitorFileTrigger()
  {
      while (workFlag_)
      {
          if (std::filesystem::exists(triggerFile_))
          {
              try
              {
                  LoadFaultInjection(triggerFile_);
              }
              catch (const std::exception &e)
              {
                  AINFO << "[inject]Error parsing fault file: " << e.what();
              }
          }

          std::this_thread::sleep_for(std::chrono::seconds(1));
      }
  }

  void LoadFaultInjection(const std::string &path)
  {
      try
      {
          std::ifstream in(path);
          if (!in.is_open())
          {
              AINFO << "Fault injection file not found: " << path;
              return;
          }

          nlohmann::json j;
          in >> j;

          for (auto &item : j)
          {
              iflyauto::FmInfo fminfo{};
              fminfo.alarmId = item["alarmId"].get<uint16_t>();
              fminfo.alarmObj = item["alarmObj"].get<uint16_t>();
              fminfo.status = item["status"].get<uint8_t>();
              fminfo.time = iflyauto::Time::VirtTime().ToMicrosecond();
              AINFO << "[inject]id: " << fminfo.alarmId << ", obj: " << fminfo.alarmObj << ", clss: " << unsigned(fminfo.clss)
                     << ", level: " << unsigned(fminfo.level) << ", status: " << unsigned(fminfo.status) << ", time: " << fminfo.time;
              PushAlarmInfo(fminfo);
          }
      }
      catch (std::exception &e)
      {
          // AINFO << "[inject]Exception while parsing fault injection file: " << e.what();
      }
  }

  std::string portName_;
  std::atomic_bool workFlag_;
  threadsafe_queue<::mdc::fm::AlarmInfo> alarmInfoQueue_;
  mdc::common_swc_lib::CommonSwcLibSwC *swcPtr_;
  std::unique_ptr<std::thread> thread_ptr_;
  std::unique_ptr<std::thread> trigger_thread_ptr_;
  std::unordered_map<uint32_t, ::mdc::fm::AlarmInfo> alarmInfoMap_;
  mutable std::mutex mut;
  std::string triggerFile_;
};

}  // namespace iflyauto

#endif  // IFLYAUTO_SEND_ALARM_INFO_H