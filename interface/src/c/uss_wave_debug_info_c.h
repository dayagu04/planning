// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_USS_WAVE_DEBUG_INFO_H_
#define _IFLYAUTO_USS_WAVE_DEBUG_INFO_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#define USS_WAVE_DEBUG_ECHO_INFO_NUM 10
#define USS_WAVE_DEBUG_OBJ_TYPE_BUF_NUM 10
#define USS_WAVE_DEBUG_SNS_INFO_NUM 12
#define USS_WAVE_DEBUG_RCV_DATA_NUM 20
#define USS_WAVE_DEBUG_START_TIME_BUF_NUM 30
#define USS_WAVE_DEBUG_WIDTH_BUF_NUM 30
#define USS_WAVE_DEBUG_HIGH_BUF_NUM 30
#define USS_WAVE_DEBUG_WV_TIME_NUM 70

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

typedef enum {
  SNS_WORK_MODE_BURST_MODE_IDLE = 0,           // 不发波不收波
  SNS_WORK_MODE_BURST_MODE_DIRECTIONUP = 1,    // 直接发波、上扫频
  SNS_WORK_MODE_BURST_MODE_DIRECTIONDOWN = 2,  // 下扫频
  SNS_WORK_MODE_BURST_MODE_INDIRECTION = 3,    // 收波
} _ENUM_PACKED_ SnsWorkMode;

typedef struct {
  uint8 u8s_uss_echo_type;        // 回波类型 uint8
  uint8 u16s_uss_echo_timestamp;  // 回波时间戳 uint8
} _STRUCT_ALIGNED_ EchoInfo;

typedef struct {
  uint8 u8s_uss_sfl_burst_mode;          // 发波模式 uint8
  uint8 u8s_uss_sfl_meas_mode;           // 工作模式 uint8
  EchoInfo echo_info[USS_WAVE_DEBUG_ECHO_INFO_NUM];  // 回波信息 ECHO_INFO[10]
} _STRUCT_ALIGNED_ USSenDataOneChannel;

typedef struct {
  uint8 data_valid_flag;  // EEPROM参数 uint8
  uint8 void0;            // EEPROM参数 uint8
  uint8 osc_trim;         // EEPROM参数 uint8
  uint8 void1;            // EEPROM参数 uint8
  uint8 customer_bits;    // EEPROM参数 uint8
  uint8 void2;            // EEPROM参数 uint8
  uint8 gain_digital;     // EEPROM参数	数字增益 uint8
  uint8 void3;            // EEPROM参数
  uint8 gain_analog;      // EEPROM参数	模拟增益 uint8
  uint8 void4;            // EEPROM参数 uint8
  uint8 drive_current;    // EEPROM参数	驱动电流 uint8
  uint8 void5;            // EEPROM参数 uint8
  uint8 drive_frequence;  // EEPROM参数	驱动频率 uint8
} _STRUCT_ALIGNED_ SnsCmdEepromData;

typedef struct {
  uint8 rcv_data_value[USS_WAVE_DEBUG_RCV_DATA_NUM];
} _STRUCT_ALIGNED_ RcDataTypeDebug;

typedef struct {
  uint8 sns_data[USS_WAVE_DEBUG_SNS_INFO_NUM];                  // 探头读取参数 uint8[12]
  uint8 cur_bit_index;                              // 探头读取参数 uint8
  uint8 sns_op_cmd_tx_bit_cnt;                      // 探头读取参数 uint8
  uint8 data_bit_index;                             // 探头读取参数 uint8
  uint8 data_cnt;                                   // 探头读取参数 uint8
  uint8 data_addr;                                  // 探头读取参数 uint8
  uint8 data_rx_try_cnt;                            // 探头读取参数 uint8
  uint8 check_sum[USS_WAVE_DEBUG_SNS_INFO_NUM];                 // 探头读取参数 uint8[12]
  uint8 prev_sns_bkwv_time[USS_WAVE_DEBUG_SNS_INFO_NUM];        // 探头读取参数 uint8[12]
  uint8 rcv_data_completed_flag[USS_WAVE_DEBUG_SNS_INFO_NUM];   // 探头读取参数 uint8[12]
  uint8 need_rcv_data_bit_length[USS_WAVE_DEBUG_SNS_INFO_NUM];  // 探头读取参数 uint8[12]
  uint8 rcv_data_bit_index[USS_WAVE_DEBUG_SNS_INFO_NUM];        // 探头读取参数 uint8[12]
  uint8 cmd_processed_state[USS_WAVE_DEBUG_SNS_INFO_NUM];       // 探头读取参数 uint8[12]
  uint8 rcv_data_cmd_type[USS_WAVE_DEBUG_SNS_INFO_NUM];         // 探头读取参数 uint8[12]
  RcDataTypeDebug rcv_data_buf[USS_WAVE_DEBUG_SNS_INFO_NUM];    // 探头读取参数 cRcvDataType[12][20]
} _STRUCT_ALIGNED_ SnsOpInfoType;

typedef struct {
  uint32 sns_bk_wv_start_time_value[USS_WAVE_DEBUG_START_TIME_BUF_NUM];
} _STRUCT_ALIGNED_ SnsBkWvStartTimeBuf;

typedef struct {
  uint32 sns_bk_wv_width_value[USS_WAVE_DEBUG_WIDTH_BUF_NUM];
} _STRUCT_ALIGNED_ SnsBkWvWidthBuf;

typedef struct {
  uint32 sns_bk_wv_high_value[USS_WAVE_DEBUG_HIGH_BUF_NUM];
} _STRUCT_ALIGNED_ SnsBkWvHighBuf;

typedef struct {
  uint32 sns_bk_wv_obj_type_value[USS_WAVE_DEBUG_OBJ_TYPE_BUF_NUM];
} _STRUCT_ALIGNED_ SnsBkWvObjTypeBuf;

typedef struct {
  uint32 echo_type_value[USS_WAVE_DEBUG_ECHO_INFO_NUM];
} _STRUCT_ALIGNED_ EchoTypeBuf;

typedef struct {
  SnsBkWvStartTimeBuf sns_bk_wv_start_time[USS_WAVE_DEBUG_SNS_INFO_NUM];  // wSnsBkWvStartTime[12][30]
  uint32 sns_wv_number[USS_WAVE_DEBUG_SNS_INFO_NUM];                      // cSnsWVNumber[12]
  SnsBkWvWidthBuf sns_bk_wv_width[USS_WAVE_DEBUG_SNS_INFO_NUM];           // wSnsBkWvWidth[12][30]
  SnsBkWvHighBuf sns_bk_wv_high[USS_WAVE_DEBUG_SNS_INFO_NUM];             // wSnsBkWvHigh[12][30]
  uint32 sns_measure_mode[USS_WAVE_DEBUG_SNS_INFO_NUM];                   // cSnsMeasureMode[12]
  SnsBkWvObjTypeBuf sns_bk_wv_obj_type[USS_WAVE_DEBUG_SNS_INFO_NUM];      // wSnsBkWvObjType[12][10]
  uint32 sns_work_mode[USS_WAVE_DEBUG_SNS_INFO_NUM];                      // cSnsWorkMode[12]
  uint32 sns_ring_time[USS_WAVE_DEBUG_SNS_INFO_NUM];                      // wSnsRingTime[12]
  EchoTypeBuf echo_type[USS_WAVE_DEBUG_SNS_INFO_NUM];                     // EchoType[12][10]
} _STRUCT_ALIGNED_ SnsDtBufType;

typedef struct {
  uint16 sns_bk_wv_time_value[USS_WAVE_DEBUG_WV_TIME_NUM];  // 回波时间戳 uint16[70]
} _STRUCT_ALIGNED_ SnsBkWvTimeType;

typedef struct {
  MsgHeader msg_header;                                            // SOC消息发送信息
  SensorMeta sensor_meta;                                          // CAN报文到达MCU信息
  SnsCmdEepromData sns_cmd_eeprom_data_buf[USS_WAVE_DEBUG_SNS_INFO_NUM];       // tSnsCmdEepromData[12]
  SnsCmdEepromData sns_cmd_eepromwrite_data_buf[USS_WAVE_DEBUG_SNS_INFO_NUM];  // tSnsCmdEepromData[12]
  USSenDataOneChannel uss_en_data_one_channel[USS_WAVE_DEBUG_SNS_INFO_NUM];    // USSenDataOneChannel[12]
  SnsOpInfoType sns_command_data;
  SnsBkWvTimeType sns_bk_wv_time[USS_WAVE_DEBUG_SNS_INFO_NUM];   // 回波时间戳 wSnsBkWvTimeType[12][70]
  uint8 sns_input_capture_int_cnt[USS_WAVE_DEBUG_SNS_INFO_NUM];  // 回波cnt计数 uint8[12]
  SnsDtBufType prepare_tobe_compare_data;
  SnsDtBufType sns_source_data;
} _STRUCT_ALIGNED_ UssWaveDebugInfo;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_USS_WAVE_DEBUG_INFO_H_