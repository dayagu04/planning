// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_PHM_H_
#define _IFLYAUTO_PHM_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "common_c.h"

#ifdef __cplusplus
namespace iflyauto {
#endif

#pragma pack(4)

// 报点类型枚举
typedef enum {
        main_flow_start = 0x00,//主处理流程起始点
        main_flow_selfdefine = 0x01,//主处理流程用户自定义点
        main_flow_end = 0x02,//主处理流程结束点
        service_init_ready = 0x03,//进程初始化成功
        service_init_fail = 0x04 //进程初始化失败
}_ENUM_PACKED_ MainFlowDotpoint;

// asw各模块的AppName后缀需与manifest.yaml中的components.name保持一致，否则模板引擎无法生成对应模块的枚举值
typedef enum {
        unknown_app_name = 0x00,
        asw_around_view_camera_perception = 0x01,
        asw_around_view_camera_perception_parking_feature = 0x02,
        asw_calibration = 0x03,
        asw_control = 0x04,
        asw_ehp = 0x05,
        asw_ehr = 0x06,
        asw_factory_calibration = 0x07,
        asw_finite_state_manager = 0x08,
        asw_function_data_loop = 0x09,
        asw_function_data_loop_a = 0x0a,
        asw_localization = 0x0b,
        asw_mega = 0x0c,
        asw_obstacle_fusion = 0x0d,
        asw_panorama_view_camera_perception = 0x0e,
        asw_panorama_view_camera_perception_laneline = 0x0f,
        asw_planning = 0x10,
        asw_static_fusion = 0x11,                    
                        
        bsw_camera_node_around_view = 0x20,
        bsw_camera_node_panorama_view = 0x21,
        bsw_com_service_pnc_node = 0x22,
        bsw_gnss_service_node =0x23,
        bsw_hmi_service = 0x24,
        bsw_ota_node = 0x25,
        bsw_sdmap = 0x26,
        bsw_soc_a_communication = 0x27,
        bsw_soc_b_communication = 0x28,
        bsw_radar_4d_service = 0x29
}_ENUM_PACKED_ AppNameEnum;

// 报点消息结构体，用于各模块给PHM报点
typedef struct {
        AppNameEnum app_name;                //填充AppNameEnum枚举类型
        uint32_t check_point_id;             //程序运行主线程的checkpointId,根据应用的主线程自定义checkpointid，默认从1000开始
        MainFlowDotpoint report_point;       //报点位置 
        uint64_t report_time_stamp;          //数据面时间戳
} _STRUCT_ALIGNED_ IflyPhmReport;

#pragma pack()
#ifdef __cplusplus
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_PHM_H_