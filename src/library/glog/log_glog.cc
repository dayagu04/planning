#include "log_glog.h"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <fstream>
#include <iostream>
#include <string>

#include "./cyber/logger/async_logger.h"

namespace planning {
#ifndef ENABLE_MDC_AP_LOG

#define BACKTRACE_LOG (0)

planning::cyber::logger::AsyncLogger *async_logger_ = nullptr;
static GlogFlag glog_flag_;

#if BACKTRACE_LOG
void SignalHandler(const char *data, int size) {
  std::string path_dir;
#ifdef IFLY_GLOG_PATH
  path_dir = "/asw/planning/glog/backtrace.log";
#else
  path_dir = "/opt/usr/log/app_log/planning";
  bool create_path = CreateLogDirectory("/opt/usr/log/app_log");
  if (create_path) {
    printf("create /opt/usr/log/app_log success\n");
    create_path = CreateLogDirectory(path_dir);
  } else {
    printf("create log path fail\n");
  }
  path_dir += "/backtrace.log";
#endif

  std::ofstream file_stream(path_dir.c_str(), std::ios::app);
  std::string str = std::string(data, size);
  file_stream << str;
  file_stream.close();
  ILOG_ERROR << str;

  return;
}
#endif

const bool CreateLogDirectory(const std::string &path) {
  DIR *log_dir = nullptr;
  log_dir = opendir(path.c_str());

  if (log_dir == nullptr) {
    int ret = mkdir(path.c_str(), 0755);

    if (ret != 0) {
      printf("make dir fail, %s \n", path.c_str());
      return false;
    }

    printf("make dir is success\n");
  } else {
    printf("dir is exist\n");
  }

  closedir(log_dir);

  return true;
}

void InitGlog(const char *file) {
  std::string path_dir;

#ifdef IFLY_GLOG_PATH
  path_dir = "/asw/planning/glog";
  bool create_path = CreateLogDirectory(path_dir);
#else
  path_dir = "/opt/usr/log/app_log/planning";
  bool create_path = CreateLogDirectory("/opt/usr/log/app_log");
  if (create_path) {
    printf("create /opt/usr/log/app_log success\n");
    create_path = CreateLogDirectory(path_dir);
  } else {
    printf("create log path fail\n");
  }
#endif

  FLAGS_log_dir = path_dir;
  // FLAGS_alsologtostderr = true;
  // FLAGS_colorlogtostderr = true;
  // FLAGS_minloglevel = 0;
  //  10 Mb
  FLAGS_max_log_size = 10;

  // Init glog
  if (glog_flag_.is_init == false) {
#if BACKTRACE_LOG
    google::InstallFailureSignalHandler();
    google::InstallFailureWriter(&SignalHandler);
#endif

    google::InitGoogleLogging(file);
    google::SetLogDestination(google::ERROR, file);
    google::SetLogDestination(google::WARNING, file);
    google::SetLogDestination(google::FATAL, file);

    // Init async logger
    async_logger_ = new planning::cyber::logger::AsyncLogger(
        google::base::GetLogger(FLAGS_minloglevel));

    google::base::SetLogger(FLAGS_minloglevel, async_logger_);

    async_logger_->Start();

    FLAGS_stop_logging_if_full_disk = true;
    ILOG_INFO << "glog init, create_path = " << create_path;
    glog_flag_.is_init = true;
  }

  return;
}

void StopGlog() {
  ILOG_INFO << "shut down glog ";

  if (glog_flag_.is_init) {
    if (async_logger_ != nullptr) {
      async_logger_->Stop();
      delete async_logger_;
    }

    google::ShutdownGoogleLogging();
  }
  return;
}

const void ResetGLogFile() {
  ILOG_INFO << "reset glog file";
  if (async_logger_ != nullptr) {
    async_logger_->CreateNewFile();
  }

  return;
}

#else

void InitGlog(const char *file) { return; }

void StopGlog() { return; }

const void ResetGLogFile() { return; }

#endif

}  // namespace planning