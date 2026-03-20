#ifndef _CAN_RAW_ETH_DATA_H_
#define _CAN_RAW_ETH_DATA_H_
/* 备注：该文件暂时仅用于数采回流FDC上CAN原始报文数据：/iflytek/dc/can/raw */
#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define MESSAGE_NUM_MAX (16u) /*一个eth包(1400bytes)最多容纳16个canFd报文*/
#define CAN_DATA_NUM_MAX 64
#define ASC_HEADER_NUM_MAX 512
#define ASC_DATA_NUM_MAX 3096
#pragma pack(4)
typedef struct {
    uint32 canId; /*can报文id*/
    uint8  can_data_size; /*can报文长度*/
    uint8  can_data[CAN_DATA_NUM_MAX]; /*can报文数据*/
    uint8  resv[3]; /*reserved*/
    uint64  timestamp; /*报文收到时的时间戳*/
}_STRUCT_ALIGNED_ CanRawMsg;

typedef struct {
    MsgHeader msg_header;
    uint8  canRawMsg_size; /*报文数量*/
    CanRawMsg  canRawMsg[MESSAGE_NUM_MAX]; /*can报文数组*/
}_STRUCT_ALIGNED_ CanRawEthData;

typedef struct {
    MsgHeader msg_header;   /* 消息发送时间 */
    char asc_header[ASC_HEADER_NUM_MAX];  /* can原始asc报文头信息 */
    uint32 asc_data_size;   /* can原始asc报文内容长度 */
    char asc_data[ASC_DATA_NUM_MAX];   /* can原始asc报文内容 */
}_STRUCT_ALIGNED_ CanAscData;
#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _CAN_RAW_ETH_DATA_H_