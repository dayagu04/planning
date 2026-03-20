// Copyright (C) iflyauto Technologies (2024). All rights reserved.
// Modified: 2024/04/15

#ifndef _IFLYAUTO_SYSTEM_MONITOR_H_
#define _IFLYAUTO_SYSTEM_MONITOR_H_

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)
#define MAX_PROCESS_NAME 64

typedef struct {
    int soc_id;               // 0-soc_a 1-soc_b
    int pid;                  //进程pid
    uint64 stamp;  //记录时间戳
    char name[MAX_PROCESS_NAME];   //进程名
    float cpu_usage;              //cpu占用（单核%）
    float memory_usage;           //内存占用比例（%）
    uint64 rss;                    //物理内存
    uint64 shr;                    //共享内存
    uint64 pss;                    //实际物理内存
    uint64 uss;                    //独占物理内存
    uint64 vsize;                  //虚拟内存
    uint64 shr_pool;               //共享内存池占用
    uint64 read_bytes;             //io读取量
    uint64 write_bytes;            //io写入量
} _STRUCT_ALIGNED_ ProcessStat;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SYSTEM_MONITOR_H_
