#ifndef TBOX_PPS_INTERFACE_H
#define TBOX_PPS_INTERFACE_H

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"
#include "vehicle_service_c.h"
#include "camera_perception_parking_slot_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)


typedef struct {
    uint64 secs; /* unsigned long long */
    uint64 nsecs; /* unsigned long long */
}_STRUCT_ALIGNED_ ifly_struct_TimeStamp_T;

typedef struct 
{
    uint8 dpTimeReady; /* unsigned char */
    ifly_struct_TimeStamp_T dpTime;
    ifly_struct_TimeStamp_T mpTime;
}_STRUCT_ALIGNED_ ifly_GnssPpsTime_T;

typedef struct 
{
    MsgHeader       msg_header;
    uint8           Ppstime_valid;
    ifly_GnssPpsTime_T Ppstime;
}_STRUCT_ALIGNED_ McuOutputMsgPps_T;   // 当 Ppstime_valid == 1 为有效


#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // 