// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_SINPRO_DATA_H_
#define _IFLYAUTO_SINPRO_DATA_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#define ATTRIBUTE_PACKED __attribute__((packed))
//#pragma pack(1)



#define SYNC_WORD_LEN 4
#define POINT_CLOUD_LENGTH 2048
#define TARGET_OUTPUT_BUFFER_LENGTH 64
#define OBSTACLE_CONTOUR_BUFFER_LENGTH 32
#define OBSTACLE_OUTPUT_BUFFER_LENGTH 192
#define RDMAP_LENGTH  27392
#define RADAR_ERR_CNT_LIMIT 20
#define RADAR_VER_STR_LEN 32
#define RADAR_DUMP_TRACE_SIZE 1536
#define RADAR_LOG_SIZE 1024
#define RADAR_FORMAL_HW_VER_LEN 11
#define RADAR_LOCAL_HW_VER_LEN 5
#define RADAR_HW_PN_LEN 40
#define RADAR_MMIC_CHIPS_LIMIT 2
#define RADAR_DSP_RSV_INFO_LEN 256
#define RADAR_PERCEPTION_DEBUG_DATA_LENGTH 14400
#define RADAR_INVALID_REASON_LIMIT 5
#define RADAR_SUMMARY_ERR_REASON_LIMIT 10

typedef enum
{
    SINPRO_RADAR_OUTPUT_UNDIFINED = 0,
    SINPRO_RADAR_OUTPUT_POINTCLOUD = 1,
    SINPRO_RADAR_OUTPUT_PERCEPTION = 2,
    SINPRO_RADAR_OUTPUT_SYSTEM = 3,
    SINPRO_RADAR_OUTPUT_DEBUG = 4,
} SINPRO_RADAR_DATA_TYPE;

typedef enum
{
    SINPRO_RADAR_STATE_UNDIFINED = 0,
    SINPRO_RADAR_STATE_BOOT = 1,
    SINPRO_RADAR_STATE_INITIAL = 2,
    SINPRO_RADAR_STATE_STANDBY = 3,
    SINPRO_RADAR_STATE_WORKING = 4,
    SINPRO_RADAR_STATE_SILENT = 5,
    SINPRO_RADAR_STATE_SLEEP = 6,
    SINPRO_RADAR_STATE_RECOVERY = 7,
    SINPRO_RADAR_STATE_SAFE = 8,
    SINPRO_RADAR_STATE_REBOOT = 9,
    SINPRO_RADAR_STATE_CALI_CUSTOMER_EOL = 10,
    SINPRO_RADAR_STATE_CALI_MISA = 11,
} SINPRO_RADAR_STATE_TYPE;

typedef enum
{
    SINPRO_RADAR_IM_NO_INTERFERENCE = 0,
    SINPRO_RADAR_IM_UNDER_CONTROL = 1,
    SINPRO_RADAR_IM_DROP_FRAM = 2
} SINPRO_RADAR_IM_TYPE;

typedef enum
{
    SINPRO_RADAR_BW_SHORT = 0,
    SINPRO_RADAR_BW_LONG = 1,
} SINPRO_RADAR_BW_TYPE;

typedef enum
{
    SINPRO_RADAR_SYNC_INIT = 0,
    SINPRO_RADAR_SYNC_ING = 1,
    SINPRO_RADAR_SYNC_OK_WITH_ADC = 2,
    SINPRO_RADAR_SYNC_OK_WITH_DEFAULT = 3,
} SINPRO_RADAR_SYNC_STATUS;

typedef enum
{
    SINPRO_RADAR_SYNC_SHORT = 0,
    SINPRO_RADAR_SYNC_LONG = 1,
} SINPRO_RADAR_SYNC_MODE;

typedef enum
{
    SINPRO_RADAR_FLS_CNT_DTC,
    SINPRO_RADAR_FLS_CNT_MISA_ALERT,
    SINPRO_RADAR_FLS_CNT_MISA_REAL,
    SINPRO_RADAR_FLS_CNT_CAL_REAL,
    SINPRO_RADAR_FLS_CNT_NUM,
} SINPRO_RADAR_FLS_CNT_E;

typedef enum {
  TOS_ADAS_TARGET_NONE = 0x0,
  TOS_ADAS_TARGET_STATIONARY = 0x10,
  TOS_ADAS_TARGET_STATIONARY_OVERLAP_GUARDRAIL = 0x11,
  TOS_ADAS_TARGET_GUARDRAIL = 0x100,
  TOS_ADAS_TARGET_MOVABLE = 0x1000,
  TOS_ADAS_TARGET_LIDAR_MOV = 0x10000,
} TOS_TARGET_TYPE_E;
typedef uint32_t tos_target_type;

typedef enum {
  TOS_CDL_LVL_NO_DANGER = 0,
  TOS_CDL_LVL_ATTENTION = 1,
  TOS_CDL_LVL_PREWARN = 2,
  TOS_CDL_LVL_ACUTEWARN = 3,
  TOS_CDL_LVL_PARTIAL_BRAKE = 4,
  TOS_CDL_LVL_FULL_BRAKE = 5,
  TOS_CDL_LVL_COLLISION_UNAVOIDABLE = 6,
  TOS_CDL_LVL_DEBUG = 7,
} TOS_COLLISION_DANGER_LEVEL_E;
typedef uint8_t tos_cdl ;

typedef struct {
  tos_target_type target_type;
  uint8_t target_index;
  uint16_t target_id;
  float ttc;
  tos_cdl cdl;
} ATTRIBUTE_PACKED Tos_Base_Collision_Info;

typedef struct {
    uint16 sync_word[SYNC_WORD_LEN]; // 0x0102, 0x0304, 0x0506, 0x0708
    uint16 version; // high 8-bit: major version, low 8-bit: minor version
    uint8 data_type;
    uint64 frame_cnt;
    uint64 frame_ts_ns; // frame timestamp
    uint64 publish_ts_ns; // publish (send) timestamp
} ATTRIBUTE_PACKED Sinpro_Radar_Header;

typedef struct {
    uint64 vehicle_info_ts_ns;
    float32 vehicle_speed_kmph;
    float32 yaw_rate_degps;
    uint8 gear_pos;
    float32 steer_whl_ang_deg;
    uint8 whl_spd_pulse[4]; // 0-front left, 1-front right, 2-rear left, 3-rear right
    uint8 whl_spd_dir[4]; // wheel speed direction
    uint8 wiper_sts;
    float32 vehicle_acceleration_g;
    uint64 vehicle_speed_ts_ns;
    uint64 yaw_rate_ts_ns;
    uint64 exposure_mid_ts_ns;
    float32 vehicle_speed_raw_kmph;
    float32 yaw_rate_raw_degps;
    float32 vehicle_acceleration_raw_g;
} ATTRIBUTE_PACKED Vehicle_Input_Info;

typedef struct {
    int32 susp_level_height_mm[4]; // FL FR RL RR
    uint8 susp_valve_state[4]; // FL FR RL RR, 0x0:no command, 0x1: level command
    uint8 susp_cur_level;
    int32 susp_comp_z; // mm
} ATTRIBUTE_PACKED Vehicle_Suspension_Info;

typedef struct {
    uint16 brand_info;
    uint16 vehicle_length_mm;
    uint16 vehicle_width_mm;
    uint16 wheel_base_mm;
    int16 radar_rear_x_mm;
    int16 radar_rear_y_mm;
    int16 radar_ground_z_mm;
    int16 head_front_x_mm;
    uint8 mount_direction;
} ATTRIBUTE_PACKED Vehicle_Type_Info;

typedef struct {
    uint8 cali_state;
    float32 yaw_cali_deg;
    float32 pitch_cali_deg;
    float32 roll_cali_deg;
} ATTRIBUTE_PACKED Sinpro_Radar_Cali_Info;

typedef struct {
    uint16 matched_obj_id;
    float32 rng_m;
    float32 azi_deg;
    float32 ele_deg;
    uint8 motion_status;

    float32 dpl_raw_mps;
    float32 dpl_unambi_mps;

    uint8 bandwidth_mode;
    float32 SNR_dB;
    float32 RCS_dBsm;

    uint8 IsPeak;
    uint8 IsGhost;
    uint8 azimuth_order;
} ATTRIBUTE_PACKED Pointcloud_Unit;

typedef struct {
    uint8 im_flag_sr;//SINPRO_RADAR_IM_TYPE
    uint8 im_flag_lr;//SINPRO_RADAR_IM_TYPE
    uint16 im_chirp_num_sr;
    uint32 im_sample_num_sr;
    uint32 im_pow_level_sr;
    float32 im_snr_loss_sr;
    uint16 im_chirp_num_lr;
    uint32 im_sample_num_lr;
    uint32 im_pow_level_lr;
    float32 im_snr_loss_lr;
} ATTRIBUTE_PACKED Im_Info;

typedef struct {
    uint8 ch_pow_tx[6];
    uint8 ch_pow_rx[8];
} ATTRIBUTE_PACKED Ch_Pow;

typedef struct {
    uint16 pointcloud_num;
    float32 short_dpl_unambi_scale_mps;
    float32 long_dpl_unambi_scale_mps;
    Im_Info im_info;
    Ch_Pow ch_pow;
    Pointcloud_Unit pointcloud_unit[POINT_CLOUD_LENGTH];
} ATTRIBUTE_PACKED Sinpro_Radar_Pointcloud;

typedef struct {
    uint16 id;
    uint16 life_counter;

    uint16 observed_counter;
    uint8 observed_status;

    float32 pos_x_m;
    float32 pos_y_m;
    float32 pos_z_m;
    float32 vel_x_mps;
    float32 vel_y_mps;
    float32 acc_x_mps2;
    float32 acc_y_mps2;

    float32 std_pos_x_m;
    float32 std_pos_y_m;
    float32 std_vel_x_mps;
    float32 std_vel_y_mps;
    float32 std_acc_x_mps2;
    float32 std_acc_y_mps2;

    float32 box_length_m;
    float32 box_width_m;
    float32 box_height_m;

    float32 heading_deg; // unit deg
    float32 yawrate_degps; // uint deg/s

    uint8 existence_prob; // [0,100]
    uint8 obstacle_prob;  // [0,100]

    uint8 moving_status;

    uint8 object_classification;
    uint8 classification_prob; // [0,100]
    int8 object_rcs; // resolution: 1 dBsm
    uint16 pcld_obs_num;

    uint8 ghost_estm;
    float32 pcld_obs_edge_pos_x_m[3];
    float32 pcld_obs_edge_pos_y_m[3];
} ATTRIBUTE_PACKED Target_Info;

typedef struct {
    uint16 target_num;
    Target_Info target[TARGET_OUTPUT_BUFFER_LENGTH];
} ATTRIBUTE_PACKED Sinpro_Radar_Target_Opt;

typedef struct {
    uint16 id;
    uint16 life_counter;
    uint8 existence_prob;
    int16 box_edge_x_m[OBSTACLE_CONTOUR_BUFFER_LENGTH];//0.02m/bin
    int16 box_edge_y_m[OBSTACLE_CONTOUR_BUFFER_LENGTH];//0.02m/bin
    uint8 total_edge_num;
    float32 vertex_x_m[4];
    float32 vertex_y_m[4];
    float32 pos_z_m;
    float32 box_height_m;
    uint8 is_passable;
    uint16 pcld_obs_num;
    uint8 obstacle_classification;
    uint8 moving_status;
    float32 radial_absolute_velocity;
} ATTRIBUTE_PACKED Obstacle_Info;

typedef struct {
    uint16 obstacle_num;
    Obstacle_Info obstacle[OBSTACLE_OUTPUT_BUFFER_LENGTH];
} ATTRIBUTE_PACKED Sinpro_Radar_Obstacle_Opt;

typedef struct {
    uint8 valid;
    float32 coef_3;
    float32 coef_2;
    float32 coef_1;
    float32 coef_0;
    float32 start_x_m;
    float32 end_x_m;
} ATTRIBUTE_PACKED Tag_Guardrail_Info;

typedef struct {
    Tag_Guardrail_Info left;
    Tag_Guardrail_Info right;
} ATTRIBUTE_PACKED Sinpro_Radar_Guardrail;

typedef struct {
    uint16 err[RADAR_ERR_CNT_LIMIT];
} ATTRIBUTE_PACKED Sinpro_Radar_Active_Error;

typedef struct {
    uint16 val[RADAR_SUMMARY_ERR_REASON_LIMIT];
} ATTRIBUTE_PACKED Sinpro_Summary_Err_Reason;

typedef struct {
    uint16 blindness;
    uint16 internal_fault_0;
    uint16 alignment_incomplete;
    uint16 alignment_out_of_range;
    uint16 voltage_above_threshold;
    uint16 voltage_below_threshold;
    uint16 temp_high_fault;
    uint16 loss_comm_with_adc;
    uint16 invalid_data_from_adc;
    uint16 loss_comm_with_zone;
    uint16 invalid_data_from_zone;
    uint16 internal_fault_1;
    uint16 internal_fault_2;
    uint16 radar_data_invalid;
    uint16 radar_version_mismatch;
    uint16 rsv[5];
} ATTRIBUTE_PACKED Sinpro_Radar_Summary_Error;

typedef struct {
    char radar_sw_version[RADAR_VER_STR_LEN];
    char sdk_sw_version[RADAR_VER_STR_LEN];
} ATTRIBUTE_PACKED Sinpro_Sw_Version;

typedef struct {
    uint8 rdmap_short[RDMAP_LENGTH];
    uint8 rdmap_long[RDMAP_LENGTH];
} ATTRIBUTE_PACKED Sinpro_Radar_Rd_Map;

typedef struct {
    uint16 blindness_prob;
} ATTRIBUTE_PACKED Sinpro_Radar_Blindness_Data;

typedef struct {
    uint16 is_useful;
    uint8 dump_reg[RADAR_DUMP_TRACE_SIZE];
} ATTRIBUTE_PACKED Sinpro_Radar_Dump_Data;

typedef struct {
    uint8 ptp_sync_status;
    uint8 frame_sync_status;
    uint8 chirp_sync_mode;
    uint32 target_time_us;
    uint32 short_chirp_center_us;
    uint32 long_chirp_center_us;
    uint32 sync_step_us;
    uint32 boot_to_sync_time_us;
    uint32 sync_cost_time_us;
    uint64 sync_start_time_us;
} ATTRIBUTE_PACKED Sinpro_Radar_Sync_Data;

typedef struct {
    uint16 is_useful;
    char data[RADAR_LOG_SIZE];
} ATTRIBUTE_PACKED Sinpro_Radar_Log;

typedef struct {
    uint64 radar_local_time_us;
    uint8 frame_type;
    uint16 useful_len; // byte
    uint8 rsv[253];
} ATTRIBUTE_PACKED Sinpro_Radar_Append_Info;

typedef struct {
    uint16 rsv_data;
} ATTRIBUTE_PACKED Sinpro_Drop_Detect;

typedef struct {
    uint8 formal_hw_ver[RADAR_FORMAL_HW_VER_LEN];
    uint8 local_hw_ver[RADAR_LOCAL_HW_VER_LEN];
    uint8 radar_hw_pn[RADAR_HW_PN_LEN];
} ATTRIBUTE_PACKED Sinpro_Hw_Info;

typedef struct {
    uint16 vol_input;
    uint16 vol_SBC_4V1;
    uint16 vol_SBC_3V3;
    uint16 vol_SBC_1V8;
    uint16 MMIC_2V3[RADAR_MMIC_CHIPS_LIMIT];
} ATTRIBUTE_PACKED Sinpro_Vol_Info;

typedef struct {
    uint8 temp_SOC;
    uint8 temp_SBC;
    uint8 temp_PMIC;
    uint8 temp_MMIC;
} ATTRIBUTE_PACKED Sinpro_Temp_Info;

typedef struct {
    uint8 CPU_load_M_core; // unit: %
    uint8 CPU_load_A_core; // unit: %
    uint8 stack_used_max; // unit: %
    uint8 stack_ID;
} ATTRIBUTE_PACKED Sinpro_Resource_Info;

typedef struct {
    uint8 is_useful; // 0-unuseful, 1-useful
    Sinpro_Vol_Info vol_info;
    Sinpro_Temp_Info temp_info;
    Sinpro_Resource_Info resource_info;
} ATTRIBUTE_PACKED Sinpro_Key_Monitor_Info;

typedef struct {
    sint64 gPTP_TS_deviation_ns; // SoC_TS + delay - Radar_TS
    uint8 power_off_act_status; // 0-not ready, 1-ready, 2-timeout
} ATTRIBUTE_PACKED Sinpro_Key_Module_Status;

typedef struct {
    uint16 useful_len; // byte
    uint8 data[RADAR_PERCEPTION_DEBUG_DATA_LENGTH];
} ATTRIBUTE_PACKED Sinpro_Perc_Debug_Data;

typedef struct {
    uint16 err[RADAR_INVALID_REASON_LIMIT];
} ATTRIBUTE_PACKED Sinpro_Invalid_Reason;

typedef struct {
    uint32 cnt_and_flag;
} ATTRIBUTE_PACKED Sinpro_Fls_Cnt_Unit;

typedef struct {
    Sinpro_Fls_Cnt_Unit fls_count[4]; // flash block write counters
} ATTRIBUTE_PACKED Sinpro_Fls_Cnt;

typedef struct {
    float32 yaw_array[5];
    float32 pitch_array[5];
    uint8 yaw_final_judge_flag;
    uint8 pitch_final_judge_flag;
} ATTRIBUTE_PACKED Sinpro_Misa_Alert_Mem;

typedef struct {
    Sinpro_Radar_Header radar_header;
    Sinpro_Radar_Pointcloud pointcloud;
    Vehicle_Input_Info vehicle_input_info;
    Sinpro_Radar_Append_Info append_info;
    Vehicle_Suspension_Info suspension_info;
    Vehicle_Type_Info vehicle_type_info;
    Sinpro_Radar_Cali_Info radar_cali_info;
    Sinpro_Invalid_Reason invalid_reason;
    uint8 is_valid; // 0-invalid, 1-valid
    uint8 rsv;
} ATTRIBUTE_PACKED Sinpro_Radar_Pointcloud_Data;

typedef struct {
    Sinpro_Radar_Header radar_header;
    Sinpro_Radar_Target_Opt target_opt;
    Sinpro_Radar_Obstacle_Opt obstacle_opt;
    Sinpro_Radar_Guardrail guardrail;
    Vehicle_Input_Info vehicle_input_info;
    Sinpro_Perc_Debug_Data perc_debug_data; // add in 0.9.0 protocol
    Sinpro_Invalid_Reason invalid_reason;
    uint8 is_valid; // 0-invalid, 1-valid
} ATTRIBUTE_PACKED Sinpro_Radar_Perception_Data;

typedef struct {
    Sinpro_Radar_Header radar_header;
    uint8 state;
    Sinpro_Radar_Active_Error active_error;
    Vehicle_Type_Info vehicle_type_info;
    Sinpro_Radar_Cali_Info radar_cali_info;
    Sinpro_Sw_Version sw_version;
    Sinpro_Radar_Dump_Data dump_data;
    Sinpro_Radar_Blindness_Data blindness_opt;
    Sinpro_Radar_Sync_Data sync_data;
    Sinpro_Radar_Summary_Error active_error_summary;
    Sinpro_Radar_Log radar_log;
    Sinpro_Drop_Detect drop_detect_data;
    Sinpro_Hw_Info hw_version;
    Sinpro_Key_Monitor_Info key_monitor_info;
    Sinpro_Key_Module_Status key_module_status;
    Sinpro_Fls_Cnt fls_cnt;
    Sinpro_Misa_Alert_Mem misa_alert_mem;
    Sinpro_Summary_Err_Reason err_reason;
    Sinpro_Invalid_Reason invalid_reason;
    uint8 is_valid; // 0-invalid, 1-valid
} ATTRIBUTE_PACKED Sinpro_Radar_System_Data;

typedef struct {
    Sinpro_Radar_Header radar_header;
    Sinpro_Radar_Rd_Map rd_map;
    Sinpro_Invalid_Reason invalid_reason;
    uint8 is_valid; // 0-invalid, 1-valid
} ATTRIBUTE_PACKED Sinpro_Radar_Debug_Data;

typedef struct ATTRIBUTE_PACKED
{
    Sinpro_Radar_Header header;
    Tos_Base_Collision_Info collision_info_so;
    Tos_Base_Collision_Info collision_info_mo;
} ATTRIBUTE_PACKED Sinpro_Radar_Aeb_Data;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_SENSOR_IMAGE_H_
