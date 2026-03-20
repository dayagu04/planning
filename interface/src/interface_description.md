# 简述
本文用于综述智驾系统的接口信息。适用于智驾相关的所有研发人员。

## 目录说明
1. c目录：存放接口定义头文件，soc/mcu共用
2. proto目录：存放proto源文件，暂时未转成c的接口定义
3. mcu_base_interface目录：存放mcu上算法用到的数据结构定义
4. interface_version目录：存放接口版本号宏定义
5. private目录：存放算法模块独立proto定义，如debug信息，供工具链使用


# 模块和接口描述

## 公共结构体
**用途：** 被多个结构体共用的结构体。
**头文件：** common_c.h
**proto：** common.proto

## adas
+ **接口：** adas功能输出
  + **头文件：** adas_function_c.h
  + **topic：** /iflytek/adas_function/adas
  + **结构体：** ADASFunctionOutputInfo
  + **频率：** 50Hz
+ **接口：** adas调试信息
  + **topic：** /iflytek/adas_function/debug_info
  + **头文件** adas_function_debug_c.h
  + **结构体：** ADASFunctionDebugOutputInfo
  + **频率：** 50Hz

## calibration
### calibration(ace+post_sale_calibration)
+ **接口：** 标定调试信息
  + **topic：** /iflytek/calibration/debug_info
  + **proto：** struct_container.proto
  + **结构体：** StructContainer
  + **频率：** 待定
+ **接口：** 标定请求接口(C/S)，售后标定作为服务端
  + **topic：** /iflytek/post_sale_calibration/activation
  + **头文件：** post_sale_calibration_c.h
  + **结构体：** PostSaleCalibActiveInfo/PostSaleCalibActiveInfoResult
  + **频率：** 不固定
+ **接口：** 标定结果查询接口(C/S)，售后标定作为服务端
  + **topic：** /iflytek/post_sale_calibration/result
  + **头文件：** post_sale_calibration_c.h
  + **结构体：** PostSaleCalibGetCalibResReqInfo/PostSaleCalibResults
  + **频率：** 不固定
+ **接口：** 标定终止接口(C/S)，售后标定作为服务端
  + **topic：** /iflytek/post_sale_calibration/stop
  + **头文件：** post_sale_calibration_c.h
  + **结构体：** PostSaleCalibStopReqInfo/PostSaleCalibStopResult
  + **频率：** 不固定
+ **接口：** 标定状态(在线/售后)
  + **topic：** /iflytek/calibration/calib_info
  + **头文件：** calib_info_c.h
  + **结构体：** CalibInfo
  + **频率：** 标定算法状态变化触发
### factory_calibration
+ **接口：** 标定请求接口(C/S)，工厂标定作为服务端
  + **topic：** /iflytek/factory_calibration/activation
  + **头文件：** factory_calib_c.h
  + **结构体：** FactoryCalibActiveInfo/FactoryCalibActiveInfoResult
  + **频率：** 不固定
+ **接口：** 标定结果查询接口(C/S)，工厂标定作为服务端
  + **topic：** /iflytek/factory_calibration/result
  + **头文件：** factory_calib_c.h
  + **结构体：** FactoryCalibGetCalibResReqInfo/FactoryCalibResults
  + **频率：** 不固定

## control
+ **接口：** control模块输出
  + **topic：** /iflytek/control/control_command
  + **头文件：** control_command_c.h
  + **结构体：** ControlOutput
  + **频率：** 50Hz
+ **接口：** control模块调试信息
  + **topic：** /iflytek/control/debug_info
  + **proto：** struct_container.proto
  + **结构体：** StructContainer
  + **频率：** 50Hz

## fm_service
+ **接口：** a面故障信息(传给工控机)
  + **topic：** /iflytek/alarm_info/fm_a_service
  + **头文件：** fm_info_c.h
  + **结构体：** FmInfo
  + **频率：** 1Hz
+ **接口：** b面故障信息(传给工控机)
  + **topic：** /iflytek/alarm_info/fm_b_service
  + **头文件：** fm_info_c.h
  + **结构体：** FmInfo
  + **频率：** 1Hz
+ **接口：** 故障降级接口(fm_a_service传给规划、状态机)
  + **topic：** /iflytek/degrade_function/fm_a_service
  + **头文件：** degraded_driving_function_c.h
  + **结构体：** DegradedDrivingFunction
  + **频率：** 10Hz
+ **接口：** 故障降级接口(fm_a_service传给mcu)
  + **topic：** /iflytek/degrade_function/fm_a_service
  + **头文件：** degraded_driving_function_c.h
  + **结构体：** DegradedDrivingFunctionMcu
  + **频率：** 10Hz
+ **接口：** 哨兵模式接口(fm_a_service传给mcu)
  + **topic：** /iflytek/fm_a_mcu_sentinel
  + **头文件：** sentinel_mode_c.h
  + **结构体：** SENTINEL_Soc2Mcu
  + **频率：** 10Hz
+ **接口：** 哨兵模式接口(fm_a_service传给环视相机)
  + **topic：** /iflytek/fm_a_camera_sentinel
  + **频率：** 10Hz
+ **接口：** 相机热插拔信号接口(fm_a_service传给环视相机)
  + **topic：** IflytekCameraFault
  + **频率：** 不定帧率
+ **接口：** 各模块故障上报接口，上报给fm_service
  + **topic：** /iflytek/alarm_info/模块名
    + 示例-状态机：/iflytek/alarm_info/finite_state_manager
    + 示例-片间通信：/iflytek/alarm_info/com_service_pnc_node
  + **头文件：** 使用平台提供的接口头文件
  + **结构体：** 使用平台提供的结构体
  + **上报时机：** 故障发生时上报
+ **接口：** MCU故障上报汇总接口
  + **topic：** /iflytek/alarm_info/mcu
  + **头文件：** 底软内部维护
  + **结构体：** 底软内部维护
  + **上报时机：** 底软自行设计
+ **接口：** SOC故障上报汇总接口(适用于存在多片soc的情况，topic视情况扩展)
  + **topic：** /iflytek/alarm_info/soc
  + **头文件：** 底软内部维护
  + **结构体：** 底软内部维护
  + **上报时机：** 底软自行设计
+ **接口：** 故障数据回传
  + **topic：** /iflytek/dtc_info/fm_a_service
  + **头文件：** dtc_code_c.h
  + **结构体：** IFLYAllDtcState
  + **频率：** 无固定帧率
+ 

## function_data_loop_exec
+ **接口：** 数据回流状态(B面)
  + **topic：** /iflytek/fdl/upload_state
  + **头文件：** fdl_upload_state_c.h
  + **结构体：** FdlUploadState
  + **频率：** 不定频率
+ **接口：** 数据回流状态(B面)
  + **topic：** /iflytek/fdl/trigger
  + **头文件：** fdl_trigger_c.h
  + **结构体：** FdlTriggerState
  + **频率：** 不定频率

## function_data_loop_a_exec
+ **接口：** 数据回流状态(A面)
  + **topic：** /iflytek/fdl/trigger_ack
  + **头文件：** fdl_trigger_c.h
  + **结构体：** FdlTriggerState
  + **频率：** 不定频率

## fusion
### obstacle fusion
+ **接口：** 障碍物融合输出
  + **topic：** /iflytek/fusion/objects
  + **头文件：** fusion_objects_c.h
  + **结构体：** FusionObjectsInfo
  + **频率：** 20Hz
+ **接口：** 障碍物融合简易输出(降负载)
  + **topic：** /iflytek/fusion/objects_compress
  + **头文件：** fusion_objects_compress_c.h
  + **结构体：** FusionObjectsInfoCompress
  + **频率：** 20Hz
+ **接口：** 通用障碍物融合输出
  + **topic：** /iflytek/fusion/occupancy/objects
  + **头文件：** fusion_occupancy_objects_c.h
  + **结构体：** FusionOccupancyObjectsInfo
  + **频率：** 20Hz
+ **接口：** 通用障碍物空间车位输出
  + **topic：** /iflytek/fusion/spatial_parking_slot
  + **头文件：** fusion_spatial_parking_slot_c.h
  + **结构体：** FusionSpatialParkingSlot
  + **频率：** 20Hz
+ **接口：** 超声波感知结果
  + **topic：** /iflytek/fusion/uss_perception_info
  + **头文件：** uss_percept_info_c.h
  + **结构体：** UssPerceptInfo
  + **频率：** 20Hz
### static fusion
+ **接口：** 车道线融合输出
  + **topic：** /iflytek/fusion/road_fusion
  + **头文件：** fusion_road_c.h
  + **结构体：** RoadInfo
  + **频率：** 20Hz
+ **接口：** 车道线融合简易输出(降负载)
  + **topic：** /iflytek/fusion/road_fusion_compress
  + **头文件：** fusion_road_compress_c.h
  + **结构体：** RoadInfoCompress
  + **频率：** 20Hz
+ **接口：** 车位融合输出
  + **topic：** /iflytek/fusion/parking_slot
  + **头文件：** fusion_parking_slot_c.h
  + **结构体：** ParkingFusionInfo
  + **频率：** 20Hz
+ **接口：** 接地线融合输出
  + **topic：** /iflytek/fusion/ground_line
  + **头文件：** fusion_groundline_c.h
  + **结构体：** FusionGroundLineInfo
  + **频率：** 20Hz
+ **接口：** 减速带融合输出
  + **topic：** /iflytek/fusion/speed_bump
  + **头文件：** fusion_deceler_c.h
  + **结构体：** FusionDecelerInfo
  + **频率：** 20Hz

## hmi_service
+ **接口：** 输出给fsm的hmi信息
  + **头文件：** hmi_inner_c.h
  + **topic：** /iflytek/hmi/inner
  + **结构体：** HmiInner
  + **频率：** 20Hz
+ **接口：** 工厂模式输入信息
  + **topic：** /iflytek/hmi/factory_mode
  + **头文件：** factory_calibration_c.h
  + **结构体：** FactoryCalibCarModeInfo
  + **频率：** 不固定
+ **接口：** 工程模式输入信息
  + **topic：** /iflytek/engineer/inner
  + **头文件：** engineer_mode_c.h
  + **结构体：** Engineer_Outer
  + **频率：** 20Hz
### CHERY_E0Y_MDC510
+ **数据流：** 底软内部的接口信息，E0Y车辆专用。
  + [com_service_pnc_exec节点]的[IflytekHmiMcuToSocBsw]流向[hmi_service_exec节点]
  + [hmi_service_exec节点]的[IflytekHmiSocToMcuBsw]流向[com_service_pnc_exec节点]
  + com_service_pnc_exec
    + PPort: IflytekHmiMcuToSocBsw
    + RPort: IflytekHmiSocToMcuBsw
  + hmi_service_exec
    + PPort: IflytekHmiSocToMcuBsw
    + RPort: IflytekHmiMcuToSocBsw
+ **接口：** CHERY_E0Y_MDC510的HMI上行数据(mcu至soc) (数采用)
  + **头文件：** hmi_service_c.h
  + **topic：** /iflytek/hmi/mcu_to_soc_bsw/chery_e0y_mdc510
  + **结构体：** HmiMcuToSocBsw_CHERY_E0Y_MDC510
  + **频率：** 20Hz
+ **接口：** CHERY_E0Y_MDC510的HMI下行数据(soc至mcu) (数采用)
  + **头文件：** hmi_service_c.h
  + **topic：** /iflytek/hmi/soc_to_mcu_bsw/chery_e0y_mdc510
  + **结构体：** HmiSocToMcuBsw_CHERY_E0Y_MDC510
  + **频率：** 20Hz
### CHERY_T26
+ + **接口：** CHERY_T26的HMI上行数据(mcu至soc)
  + **头文件：** hmi_service_c.h
  + **topic：** /iflytek/hmi/mcu_to_soc_bsw/chery_t26_fdc
  + **结构体：** HmiMcuToSocBsw_CHERY_T26
  + **频率：** 20Hz
+ **接口：** CHERY_T26的HMI下行数据(soc至mcu)
  + **头文件：** hmi_service_c.h
  + **topic：** /iflytek/hmi/soc_to_mcu_bsw/chery_t26_fdc
  + **结构体：** HmiSocToMcuBsw_CHERY_T26
  + **频率：** 20Hz

## localization
+ **接口：** 定位信息
  + **topic：** /iflytek/localization/egomotion
  + **头文件：** ifly_localization_c.h
  + **结构体：** IFLYLocalization
  + **频率：** 100Hz
+ **接口：** 语义信息
  + **topic：** /iflytek/localization/semantic_info
  + **头文件：** ifly_parking_map_c.h
  + **结构体：** SemanticInfo
  + **频率：** 10Hz

## map
### ehr
+ **接口：** 导航地图信息
  + **topic：** /iflytek/ehr/static_map
  + **proto：** ehr.proto
  + **结构体：** StaticMap
  + **频率：** 1Hz
+ **接口：** 地图的位置消息
  + **topic：** /iflytek/ehr/position
  + **头文件：** ehr_c.h
  + **结构体：** Position
  + **频率：** 20Hz
+ **接口：** 原始地图信息
  + **topic：** /iflytek/ehr/origin_data
  + **proto：** ehr.proto
  + **结构体：** OriginData
  + **频率：** 20Hz
### ehr_sdmap
+ **接口：** SD地图信息
  + **topic：** /iflytek/ehr/sdmap_info
  + **proto：** ehr_sdmap.proto
  + **结构体：** SdMap
  + **频率：** 1Hz
+ **接口：** SDpro地图信息
  + **topic：** /iflytek/ehr/sdpromap_info
  + **proto：** map_data.proto
  + **结构体：** MapData
  + **频率：** 1Hz
+ **接口：** EHR 调试信息
  + **topic：** /iflytek/ehr/debuginfo
  + **proto：** struct_container.proto
  + **结构体：** StructContainer
  + **频率：** 非固定频率
### ehp
+ **接口：** hpp地图信息(地图使用)
  + **topic：：：** /iflytek/ehp/parking_map
  + **proto：** ifly_parking_map.proto
  + **结构体：** ParkingInfo
  + **频率：** 1Hz
+ **接口：** hpp地图信息(建图文件命名使用)
  + **topic：** /iflytek/ehp/parking_map_file_info
  + **头文件：** ifly_parking_map_c.h
  + **结构体：** ParkingMapFileInfo
  + **频率：** 1Hz
+ **接口：** hpp地图信息(可视化交互使用)
  + **topic：** /iflytek/ehp/map_manager
  + **头文件：** ifly_parking_map_c.h
  + **结构体：** EhpOutput
  + **频率：** 触发式，不计帧率
### mega
+ **接口：** 建图信息(新车位融合使用)
  + **topic：** /iflytek/mega/local_map
  + **头文件：** ifly_parking_map_c.h
  + **结构体：** ParkingInfo
  + **频率：** 20Hz
+ **接口：** 建图状态信息
  + **topic：** /iflytek/mega/mapping_status
  + **头文件：** ifly_mapping_status_c.h
  + **结构体：** MappingStatusInfo
  + **频率：** 50Hz
### sdmap
+ **接口：** 算路信息(地图app的算路json消息发送给ehr)
  + **topic：** /iflytek/sdmap/sdmap_info
  + **结构体：** String字符串
  + **频率：** 0.2Hz

## monitoring(预留待定)

## perception
### around view camera perception
+ **接口：** 视觉感知车位线
  + **topic：** /iflytek/camera_perception/parking_slot_list
  + **头文件：** camera_perception_parking_slot_c.h
  + **结构体：** ParkingSlotSelectInfo
  + **频率：** 10Hz
+ **接口：** 视觉感知减速带
  + **topic：** /iflytek/camera_perception/deceler
  + **头文件：** camera_perception_deceler_c.h
  + **结构体：** DecelerPerceptionInfo
  + **频率：** 10Hz
+ **接口：** 视觉感知接地线
  + **topic：** /iflytek/camera_perception/ground_line
  + **头文件：** camera_perception_groundline_c.h
  + **结构体：** GroundLinePerceptionInfo
  + **频率：** 10Hz
+ **接口：** 环视感知车道线(泊车用)
  + **topic：** /iflytek/camera_perception/parking_lane_line
  + **头文件：** camera_perception_lane_lines_c.h
  + **结构体：** LaneLineSet
  + **频率：** 10Hz
+ **接口：** 环视通用障碍物信息(2D轮廓点)
  + **topic：** /iflytek/camera_perception/occupancy_objects
  + **头文件：** camera_perception_occupancy_objects_c.h
  + **结构体：** CameraPerceptionOccObjectsInfo
  + **频率：** 10Hz
+ **接口：** 3D通用障碍物信息(3D体素,用于可视化显示)
  + **topic：** /iflytek/camera_perception/3d_occupancy_objects
  + **proto：** camera_perception_3d_occupancy_objects.proto
  + **结构体：** CameraPerceptionObjectsInfo3D
  + **频率：** 10Hz
+ **接口：** 3D-box障碍物信息(3D-BOX)
  + **topic：** /iflytek/camera_perception/3d_general_objects
  + **头文件：** camera_perception_objects_c.h
  + **结构体：** CameraPerceptionObjectsInfo
  + **频率：** 10Hz
+ **接口：** 环视图像脱敏区域坐标信息
  + **topic：** /iflytek/camera_perception/around_view_2d_bounding_boxes
  + **头文件：** camera_perception_objects_c.h
  + **结构体：** CameraPerceptionAllView2DBoundingBoxes
  + **频率：** 10Hz
+ **接口：** 视觉感知自用感知诊断调试信息
  + **topic：**
    + /iflytek/camera_perception/parking_slot_list_diagnostic_info           车位与限位器信息
    + /iflytek/camera_perception/deceler_diagnostic_info                     减速带
    + /iflytek/camera_perception/ground_line_diagnostic_info                 接地线
    + /iflytek/camera_perception/occupancy_objects_diagnostic_info           通用障碍物信息(2D轮廓点)
    + /iflytek/camera_perception/3d_occupancy_objects_diagnostic_info        通用障碍物信息(3D体素)
    + /iflytek/camera_perception/3d_general_objects_diagnostic_info          3D-box障碍物信息
  + **proto：** camera_perception_diagnostic_info.proto
  + **结构体：** CameraPerceptionDiagnosticInfo
  + **频率：** 各项目根据效果和负载而定
### front view camera perception / panorama view camera perception
+ **接口：** 视觉感知障碍物
  + **topic：** /iflytek/camera_perception/objects
  + **头文件：** camera_perception_objects_c.h
  + **结构体：** CameraPerceptionObjectsInfo
  + **频率：** 20Hz,10Hz(各项目视效果和负载而定)
+ **接口：** 视觉感知特征
  + **topic：** /iflytek/camera_perception/feature
  + **头文件：** camera_perception_feature_c.h
  + **结构体：** CameraPerceptionFeatureInfo
  + **频率：** 10Hz
+ **接口：** 视觉感知车道线
  + **topic：** /iflytek/camera_perception/lane_lines
  + **头文件：** camera_perception_lane_lines_c.h
  + **结构体：** LaneLineSet
  + **频率：** 25Hz
+ **接口：** 视觉感知可变车道信息
  + **topic：** /iflytek/camera_perception/lane_topo
  + **头文件：** camera_perception_lane_lines_c.h
  + **结构体：** LaneLineSet
  + **频率：** 10Hz
+ **接口：** 视觉感知交通标志信息(信号灯/限速标志牌/辅助标志牌)
  + **topic：** /iflytek/camera_perception/traffic_sign_recognition
  + **头文件：** camera_perception_tsr_c.h
  + **结构体：** CameraPerceptionTsrInfo
  + **频率：** 10Hz
+ **接口：** 视觉感知交通场景信息
  + **topic：** /iflytek/camera_perception/scene
  + **头文件：** camera_perception_scene_c.h
  + **结构体：** CameraPerceptionScene
  + **频率：** 10Hz
+ **接口：** 障碍物预测结果(感知与预测模型合并后，预测结果由感知节点输出)
  + **topic：** /iflytek/prediction/prediction_result
  + **头文件：** prediction_c.h
  + **结构体：** PredictionResult
  + **频率：** 10Hz
+ **接口：** 视觉感知占用栅格信息
  + **topic：** /iflytek/camera_perception/occ_grid
  + **头文件：** camera_perception_occupancy_grid_c.h
  + **结构体：** CameraPerceptionOccGridInfo
  + **频率：** 10Hz
+ **接口：** 视觉感知可行驶区域信息
  + **topic：** /iflytek/camera_perception/drivable_space_grid
  + **头文件：** camera_perception_occupancy_grid_c.h
  + **结构体：** CameraPerceptionDrivableSpaceGridInfo
  + **频率：** 10Hz
+ **接口：** 视觉感知车道线调试信息
  + **topic：** /iflytek/camera_perception/lane_lines_debug_info
  + **头文件：** camera_perception_lane_lines_c.h
  + **结构体：** LaneLineSet
  + **频率：** 10Hz
+ **接口：** 视觉感知可变车道调试信息
  + **topic：** /iflytek/camera_perception/lane_topo_debug_info
  + **头文件：** camera_perception_lane_lines_c.h
  + **结构体：** LaneLineSet
  + **频率：** 10Hz
+ **接口：** 周视图像脱敏区域坐标信息
  + **topic：** /iflytek/camera_perception/panorama_view_2d_bounding_boxes
  + **头文件：** camera_perception_objects_c.h
  + **结构体：** CameraPerceptionAllView2DBoundingBoxes
  + **频率：** 10Hz
+ **接口：** 视觉感知自用感知诊断调试信息
  + **topic：**
    + /iflytek/camera_perception/objects_diagnostic_info                     障碍物
    + /iflytek/camera_perception/lane_lines_diagnostic_info                  车道线
    + /iflytek/camera_perception/lane_topo_diagnostic_info                   可变车道
    + /iflytek/camera_perception/traffic_sign_recognition_diagnostic_info    道路交通标识
  + **proto：** camera_perception_diagnostic_info.proto
  + **结构体：** CameraPerceptionDiagnosticInfo
  + **频率：** 各项目根据效果和负载而定
### mobileye(用于效果对比)
+ **接口：** mobileye感知障碍物
  + **topic：** /mobileye/camera_perception/objects
  + **头文件：** camera_perception_objects_c.h
  + **结构体：** CameraPerceptionObjectsInfo
  + **频率：** 33Hz
+ **接口：** mobileye感知车道线
  + **topic：** /mobileye/camera_perception/lane_lines
  + **头文件：** camera_perception_lane_lines_c.h
  + **结构体：** LaneLineSet
  + **频率：** 33Hz

### radar perception
+ **接口：** 3D毫米波雷达障碍物
  + **topic：**
    + 前雷达 /iflytek/radar_fm_perception_info
    + 前左雷达 /iflytek/radar_fl_perception_info
    + 前右雷达 /iflytek/radar_fr_perception_info
    + 后左雷达 /iflytek/radar_rl_perception_info
    + 后幼雷达 /iflytek/radar_rr_perception_info
  + **头文件：** radar_perception_objects_c.h
  + **结构体：** RadarPerceptionObjectsInfo
  + **频率：** 20Hz
+ **接口：** 4D毫米波雷达动态障碍物
  + **topic：** /iflytek/radar_fm_perception/4d_target
  + **头文件：** radar_perception_objects_c.h
  + **结构体：** RadarPerceptionObjectsInfo
  + **频率：** 15Hz
+ **接口：** 4D毫米波雷达静态障碍物
  + **topic：** /iflytek/radar_fm_perception/4d_obstacle
  + **头文件：** radar_perception_objects_c.h
  + **结构体：** RadarPerceptionObjectsInfo
  + **频率：** 15Hz
+ **接口：** 4D毫米波雷达点云
  + **topic：** /iflytek/radar_fm_perception/4d_point_cloud
  + **头文件：** sensor_radar_c.h
  + **结构体：** RadarPointCloud
  + **频率：** 15Hz
+ **接口：** 4D毫米波雷达数采用(仅做记录)
  + **头文件：** sensor_sinpro_radar.h
  + **描述：** 
    + /iflytek/sinpro_radar_point_cloud     Sinpro_Radar_Pointcloud_Data  点云      15Hz
    + /iflytek/sinpro_radar_debug_data      Sinpro_Radar_Debug_Data       调试信息  15Hz
    + /iflytek/sinpro_radar_system_data     Sinpro_Radar_System_Data      系统信息  15Hz
    + /iflytek/sinpro_radar_perception_data Sinpro_Radar_Perception_Data  感知数据  15Hz

### uss driver
+ **接口：** 超声波驱动
  + **topic：** /iflytek/uss/usswave_info
  + **头文件：** uss_wave_info_c.h
  + **结构体：** UssPdcIccSendDataType
  + **频率：** 50Hz

## planning
+ **接口：** 路径规划结果
  + **topic：** /iflytek/planning/plan
  + **头文件：** planning_plan_c.h
  + **结构体：** PlanningOutput
  + **频率：** 10Hz
+ **接口：** 规划给可视化的状态
  + **topic：** /iflytek/planning/hmi
  + **头文件：** planning_hmi_c.h
  + **结构体：** PlanningHMIOutputInfoStr
  + **频率：** 10Hz
+ **接口：** 规划调试信息
  + **topic：** /iflytek/planning/debug_info
  + **proto：** struct_container.proto
  + **结构体：** StructContainer
  + **频率：** 10Hz

## sensor
+ **接口：** 前视相机图片信息
  + **topic：** /iflytek/sensor/camera/front/image
  + **头文件：** sensor_image_c.h
  + **结构体：** Front_Camera_Image_Info
  + **频率：** MDC平台研发车频率为10Hz; MDC平台数采车频率为20Hz; FDC平台频率为20Hz
+ **接口：** h265图
  + **topic：**
    + 前视120(未脱敏) /iflytek/sensor/camera/front_120_full_resolution_clear_h265
    + 前视120 /iflytek/sensor/camera/front_120_full_resolution_h265
    + 前视30° /iflytek/sensor/camera/front_30_full_resolution_h265
    + 后视 /iflytek/sensor/camera/rear_full_resolution_h265
    + 侧前左 /iflytek/sensor/camera/side_fl_full_resolution_h265
    + 侧前右 /iflytek/sensor/camera/side_fr_full_resolution_h265
    + 侧后右 /iflytek/sensor/camera/side_rl_full_resolution_h265
    + 侧后左 /iflytek/sensor/camera/side_rr_full_resolution_h265
    + 环视右 /iflytek/sensor/camera/surround/right_full_resolution_h265
    + 环视前 /iflytek/sensor/camera/surround/front_full_resolution_h265
    + 环视左 /iflytek/sensor/camera/surround/left_full_resolution_h265
    + 环视后 /iflytek/sensor/camera/surround/rear_full_resolution_h265
  + **头文件：** sensor_image_c.h
  + **结构体：** Camera_Image_Info
  + **频率：** 10Hz
+ **接口：** 环视相机原图信息(做拼接的图，适用于环视视相机所有路一并发送的情况，或者仅发送拼接图的情况)
  + **topic：** /iflytek/sensor/camera/surround/image
  + **头文件：** sensor_image_c.h
  + **结构体：** Surround_Camera_Image_Info
  + **频率：** MDC平台研发车频率为10Hz; MDC平台数采车频率为20Hz; FDC平台频率为20Hz
+ **接口：** 环视相机原图信息(做拼接的图，适用于环视相机原图需要各路逐一发送的情况)
  + **topic：**
    + 环视右 /iflytek/sensor/camera/surround/image/right
    + 环视前 /iflytek/sensor/camera/surround/image/front
    + 环视左 /iflytek/sensor/camera/surround/image/left
    + 环视后 /iflytek/sensor/camera/surround/image/rear
  + **头文件：** sensor_image_c.h
  + **结构体：** Camera_Image_Info
  + **频率：** MDC平台研发车频率为10Hz; MDC平台数采车频率为20Hz; FDC平台频率为20Hz
+ **接口：** 环视相机重采样图片信息(做障碍物检测，适用于环视相机图片需要各路逐一发送的情况)
  + **topic：**
    + 环视右 /iflytek/sensor/camera/surround/resize_image/right
    + 环视前 /iflytek/sensor/camera/surround/resize_image/front
    + 环视左 /iflytek/sensor/camera/surround/resize_image/left
    + 环视后 /iflytek/sensor/camera/surround/resize_image/rear
  + **头文件：** sensor_image_c.h
  + **结构体：** Camera_Image_Info
  + **频率：** MDC平台研发车频率为10Hz; MDC平台数采车频率为20Hz;
+ **接口：** 环视相机哨兵模式接口 (环视相机传给fm_a，暂未使用)
  + **topic：** /iflytek/sensor/rtsp_status
  + **频率：** 10Hz
+ **接口：** 环视相机重启请求接口 (环视相机传给fm_a，仅域内通信，无需数采)
  + **topic：** IflytekCameraState
  + **频率：** 不定帧率
  
+ **接口：** 周视相机图片信息(适用于周视相机图片所有路一并发送的情况)
  + **topic：** /iflytek/sensor/camera/panorama/image
  + **头文件：** sensor_image_c.h
  + **结构体：** Panorama_Camera_Image_Info
  + **频率：** MDC平台研发车频率为10Hz; MDC平台数采车频率为20Hz;FDC平台频率为20Hz
+ **接口：** 周视相机图片独立信息(适用于周视相机图片需要各路逐一发送的情况)
  + **topic：**
    + 前视30° /iflytek/sensor/camera/front_30
    + 前视120° /iflytek/sensor/camera/front_120
    + 前视120°鹰眼视角 /iflytek/sensor/camera/front_120_eagle
    + 后视 /iflytek/sensor/camera/rear
    + 侧前左 /iflytek/sensor/camera/side_fl
    + 侧前右 /iflytek/sensor/camera/side_fr
    + 侧后右 /iflytek/sensor/camera/side_rl
    + 侧后左 /iflytek/sensor/camera/side_rr
  + **头文件：** sensor_image_c.h
  + **结构体：** Camera_Image_Info
  + **频率：** MDC平台研发车频率为10Hz; MDC平台数采车频率为20Hz; FDC平台频率为20Hz
+ **接口：** 相机数采
  + **topic：**
    + 前视30° /iflytek/dc/camera/front_30
    + 前视120°/iflytek/dc/camera/front_120
    + 后视 /iflytek/dc/camera/rear
    + 侧前左 /iflytek/dc/camera/side_fl
    + 侧前右 /iflytek/dc/camera/side_fr
    + 侧后右 /iflytek/dc/camera/side_rl
    + 侧后左 /iflytek/dc/camera/side_rr
    + 环视右 /iflytek/dc/camera/surround_right
    + 环视前 /iflytek/dc/camera/surround_front
    + 环视左 /iflytek/dc/camera/surround_left
    + 环视后 /iflytek/dc/camera/surround_rear
  + **proto：** sensor_image.proto
  + **结构体：** Camera_Image_Info
  + **频率：** MDC平台研发车频率为10Hz; MDC平台数采车频率为20Hz; FDC平台频率为20Hz
+ **接口：** gnss信息
  + **topic：** /iflytek/sensor/gnss
  + **头文件：** sensor_gnss_c.h
  + **结构体：** IFLYGnss
  + **频率：** 1Hz
+ **接口：** tbox信息
  + **topic：** /iflytek/sensor/gnss_tbox
  + **头文件：** sensor_gnss_c.h
  + **结构体：** IFLYGnss
  + **频率：** 1Hz
+ **接口：** imu信息
  + **topic：** /iflytek/sensor/imu
  + **头文件：** sensor_imu_c.h
  + **结构体：** IFLYIMU
  + **频率：** 100Hz
+ **接口：** lidar信息
  + **topic：** /iflytek/sensor/lidar/rsp128
  + **头文件：** sensor_lidar_c.h
  + **结构体：** LidarMsgInfo
  + **频率：** 10Hz

## state_machine
+ **接口：** 功能状态机输出
  + **topic：** /iflytek/fsm/soc_state
  + **头文件：** func_state_machine_c.h
  + **结构体：** FuncStateMachine
  + **频率：** 20Hz
+ **接口：** 系统状态机输出(暂无)
  + **头文件：** fsm_mcu_out_c.h
  + **结构体：** FsmMcuOut
+ **接口：** hmi的输出到仪表、可视化的信号
  + **topic：** /iflytek/fsm/hmi_soc_outer
  + **头文件：** hmi_soc_outer_c.h
  + **结构体：** HmiSocOuter
  + **频率：** 20Hz
+ **接口：** MCU上的hmi的输出，到mcu的hmi_service
  + **topic：** 无(不经过片间通过，soc需要的信息由hmi_service传到soc)
  + **头文件：** hmi_mcu_out_c.h
  + **结构体：** HmiMcuOut
  + **频率：** 50Hz

## vehicle service
+ **接口：** 自车信息
  + **topic：** /iflytek/vehicle_service
  + **头文件：** vehicle_service_c.h
  + **结构体：** VehicleServiceOutputInfo
  + **频率：** 50Hz

## 其他
+ **接口：** 系统版本号输出
  + **topic：** /iflytek/system/version
  + **头文件：** system_version_c.h
  + **结构体：** SystemVersion
  + **频率：** 无需固定，随发布模块而定(当前是静态融合发布)
+ **接口：** 域控时间戳信息
  + **topic：** /iflytek/other/time_synchronization
  + **头文件：** time_synchronization_c.h
  + **结构体：** TimeSynchronizationInfo
  + **频率：** 1Hz
+ **接口：** can原始数据输出(数采)
 + **topic：** /iflytek/dc/can/raw
 + **头文件：** can_raw_eth_data_c.h
 + **结构体：** CanRawEthData
 + **频率：** 无需固定，有输出即发送
+ **接口：** 片间通信精简
 + **头文件：** icc_c.h
 + **结构体：** ControlOutputOnMcu、FuncStateMachineMcu
 + **频率：** 50hz

## 其他文件
1. proto/common_vehicle_param.proto 车辆相关的公共配置参数定义文件
2. c/common_platform_type_soc.h SOC平台C类型定义文件
3. c/struct_container.hpp proto结构体包装器工具头文件
4. mcu_base_interface/define_common.h MCU宏定义

## topic名称转换关系
1. 默认：未做特殊说明的平台，默认使用上文topic，不做转换。
2. MDC510平台：
   1. 数采topic：默认。
   2. CM通信topic：topic名中所有/和_的左边第一个字母大写，再删除所有/和_。(单向转换)


# 接口数据结构及其含义
详见各proto和头文件。


# 片间通信topic汇总(topic名称以上方文档为准，此处仅做记录)

## S811 / t26
### SOC -> MCU
| topic | 头文件 | 备注 |
| ---- | ---- | ---- |
| /iflytek/control/control_command | control_command_c.h | |
| /iflytek/fusion/objects_compress | fusion_objects_compress_c.h | |
| /iflytek/fusion/road_fusion_compress | fusion_road_compress_c.h | |
| /iflytek/fsm/soc_state | func_state_machine_c.h | |
| /iflytek/hmi/soc_to_mcu_bsw/chery_t26_fdc | hmi_service_c.h | T26 hmi service结构体HmiSocToMcuBsw_CHERY_T26 |
| 暂无 | hmi_service_c.h | S811 hmi service结构体(暂无) |
### MCU -> SOC
| topic | 头文件 | 备注 |
| ---- | ---- | ---- |
| /iflytek/adas_function/debug_info | adas_function_debug_c.h | |
| /iflytek/radar_fm_perception_info | radar_perception_objects_c.h | |
| /iflytek/radar_fl_perception_info | radar_perception_objects_c.h | |
| /iflytek/radar_fr_perception_info | radar_perception_objects_c.h | |
| /iflytek/radar_rl_perception_info	| radar_perception_objects_c.h | |
| /iflytek/radar_rr_perception_info	| radar_perception_objects_c.h | |
| /iflytek/sensor/gnss | sensor_gnss_c.h | |
| /iflytek/sensor/imu | sensor_imu_c.h | |
| /iflytek/uss/usswave_info | uss_wave_info_c.h | |
| /iflytek/vehicle_service | vehicle_service_c.h | |
| /iflytek/hmi/mcu_to_soc_bsw/chery_t26_fdc | hmi_service_c.h | 结构体HmiMcuToSocBsw_CHERY_T26 |
| 暂无 | hmi_service_c.h | S811 hmi service结构体(暂无) |

## E0Y
### SOC -> MCU
| topic | 头文件 | 备注 |
| ---- | ---- | ---- |
| /iflytek/control/control_command | control_command_c.h | |
| /iflytek/fusion/objects_compress | fusion_objects_compress_c.h | |
| /iflytek/fusion/road_fusion_compress | fusion_road_compress_c.h | |
| /iflytek/fsm/soc_state | func_state_machine_c.h | |
| /iflytek/hmi/soc_to_mcu_bsw/chery_e0y_mdc510 | hmi_service_c.h | 结构体HmiSocToMcuBsw_CHERY_E0Y_MDC510 |
### MCU -> SOC
| topic | 头文件 | 备注 |
| ---- | ---- | ---- |
| /iflytek/adas_function/debug_info | adas_function_debug_c.h | |
| /iflytek/radar_fm_perception_info | radar_perception_objects_c.h | |
| /iflytek/radar_fl_perception_info | radar_perception_objects_c.h | |
| /iflytek/radar_fr_perception_info | radar_perception_objects_c.h | |
| /iflytek/radar_rl_perception_info	| radar_perception_objects_c.h | |
| /iflytek/radar_rr_perception_info	| radar_perception_objects_c.h | |
| /iflytek/sensor/imu | sensor_imu_c.h | |
| /iflytek/uss/usswave_info | uss_wave_info_c.h | |
| /iflytek/vehicle_service | vehicle_service_c.h | |
| /iflytek/alarm_info/mcu | 底软内部维护 | 用于诊断消息汇总 |
| /iflytek/hmi/mcu_to_soc_bsw/chery_e0y_mdc510 | hmi_service_c.h | 结构体HmiMcuToSocBsw_CHERY_E0Y_MDC510 |
| /iflytek/mcu_fm_a_sentinel | sentinel_mode_c.h | 结构体SENTINEL_Mcu2Soc |

# MCU部分

1. adas_function
2. 底软
   1. 底软基础模块
   2. hmi_on_mcu
   3. parking_distance_control
   4. radar_perception
   5. uss_driver
   6. vehicle_service

