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

#include <cyber/binary.h>
#include "cyber/init.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "glog/raw_logging.h"
#include "log_glog.h"
#include "src/common/cyber/logger/async_logger.h"

int main(int argc, char* argv[]) {
  FilePath::SetName("test_cyber_log");

  FLAGS_log_dir = "/asw/planning/glog";
  // FLAGS_alsologtostderr = true;
  // FLAGS_colorlogtostderr = true;
  // FLAGS_max_log_size = 500;
  // FLAGS_minloglevel = 1;
  // FLAGS_v = 4;

  // Init glog
  InitGlog(FilePath::GetName().c_str());

  // test apollo cyber log api
  ILOG_INFO << "log init finish";

  // test google log api
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream()
      << "LEFT_BRACKET << module << RIGHT_BRACKET";

  LOG(INFO) << "Hello,GOOGLE!";

  return 0;
}