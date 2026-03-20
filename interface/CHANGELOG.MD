# 发版分支：main_BESTUNE_E541_MDC510
# 版本号：03.01版本
# 发版日期：2026.02.24
一、接口变更
  1. 【common】输入信息源的枚举类型新增adas_function枚举字段
二、连线变更
  1. 新增片间通讯到状态机的连线(IflytekAdasFunctionAdas)
三、其他改动
  1. mainfest.yaml适配新增片间通讯到状态机的连线

# 发版分支：main_BESTUNE_E541_MDC510
# 版本号：02.15版本
# 发版日期：2026.02.09
一、接口变更
  1. 【vs】拆分驾驶员状态、门锁状态的接口字段
  2. 【建图】新增自车所在车位id的接口字段
  3. 【静态融合】新增自车所在车位id、自车是否在车位内的接口字段
  4. 【planning_hmi】新增窄路通行行驶暂停原因的接口字段
  5. 【规划】新增窄路通行规划状态的接口字段
  6. 【规划】新增泊车规划失败原因、泊车档位变化状态的接口字段
  7. 【故障降级】新增PA、NRA及adas等功能的功能状态与故障等级接口字段
  8. 【片间通讯】新增所有功能的功能状态接口字段
  9. 【状态机】新增功能降级开关反馈的接口字段
  10. 【hmi】新增显示功能降级开关反馈的接口字段
二、连线变更
  1. 新增建图到静态融合的连线(IflytekMegaMappingStatus)
三、其他改动
  1. mainfest.yaml适配新增建图到静态融合的连线
  2. 故障发布功能中新增故障注入功能

# 发版分支：main_CHERY_M32T_MDC510
# 版本号：00.05.11版本
# 发版日期：2026.02.05
一、接口变更
  1. 【障碍物融合】新增行泊车感知模式的接口字段
  2. 【状态机】修改RPA文言的部分枚举值名称
二、连线变更
三、其他改动

# 发版分支：main_BESTUNE_E541_MDC510
# 版本号：02.08版本
# 发版日期：2026.02.02
一、接口变更
  1. 【障碍物融合】新增行泊车感知模式的接口字段
  2. 【状态机】新增小蓝灯软开关、小蓝灯请求信息的接口字段
  3. 【hmi】新增小蓝灯主开关、小蓝灯开关状态反馈、通用信号输入的接口字段
二、连线变更
三、其他改动

# 发版分支：main_BESTUNE_E541_MDC510
# 版本号：02.01版本
# 发版日期：2026.01.26
一、接口变更
  1. 【hmi】新增记忆泊车目标车位id及三步引导开始命令的接口字段
  2. 【状态机】新增记忆泊车IVI显示相关的接口字段
  3. 【状态机】新增窄路通行信息的接口字段
  4. 【planning_hmi】新增窄路通行行驶距离满足的接口字段
  5. 【planning_hmi】新增变道原因枚举和选道方向的接口字段
  6. 【gnss】提升海拔高度、定位经纬度字段的精度，新增gnss消息来源的接口字段
二、连线变更
  1. 新增ehp到hmi_service的连线(IflytekEhpParkingMapToHmi)
  2. 新增gnss到sdmap的连线(IflytekSensorGnss)
  3. 新增com_pnc到fdl的连线(IflytekCanTxInfo )
三、其他改动
  1. mainfest.yaml适配新增ehp到hmi_service、gnss到sdmap、com_pnc到fdl的连线

# 发版分支：main_BESTUNE_E541_MDC510
# 版本号：01.25的E541版本交付
# 发版日期：2026.01.19
一、接口变更
  1. 【ehp】地图文件最大数量由10改为100
  2. 【ehp】从EhpStatus中拆分出EhpMatchingStatus以及EhpMapStatus表征匹配状态以及静默建图时的地图状态
二、连线变更
三、其他改动
  1. ehp节点的运行帧率由1hz改为10hz

# 发版分支：main_BESTUNE_E541_MDC510
# 版本号：01.15的E541版本交付
# 发版日期：2026.01.12
一、接口变更
  1. 【planning_hmi】新增避让刹停文言的接口字段
二、连线变更
  1. 新增融合occ障碍物到状态机的连线
三、其他改动

# 发版分支：main/BESTUNE_E541_MDC510_v2.11
# 版本号：01.10的E541版本交付
# 发版日期：2026.01.05
一、接口变更
  hpp功能相关:
  1. 【状态机】新增状态机后台状态BackgroundState的接口字段
  2. 【状态机】新增可视化模拟消星按钮的接口字段
  3. 【状态机】、【hmi_soc_outer】hpp的preactive状态拆分为preactive_driving和preactive_parking接口字段
  4. 【静态融合】新增静态融合记忆库位序号memorized_slot_id的接口字段
  5. 【hmi】新增是否需要消星的接口字段
  6. 【定位】增加gps_snr、gps_signal_available的接口字段
  7. 【ehp】新增表征消星点与地图匹配状态的接口字段
  8. 【planning_hmi】新增规划失败原因的接口字段
  9. 【planning_hmi】修改记忆泊车目标停车位是否已被占用的接口字段名称
  10. 【规划】新增HppPlanningStatus枚举类型的接口字段
  11. 【common3.proto】新增《common3.proto》接口定义，使用proto3语法，定义了与common.proto中相同的messages
  12. 【common3.proto】将Common3.proto中的FloorId类型从uint32t修改为int32t
  13. 【ifly_parking_map.proto】已删除该文件，新增parking_map.proto替代
  14. 【parking_map.proto】 新增《parking_map.proto》接口定义，使用proto3语法，定义了与ifly_parking_map.proto类似的messages
  15. 【parking_map.proto】语法升级至proto3，改为import 《common3.proto》
  16. 【parking_map.proto】将parking_map.proto命名空间从autopilot修改为 IFLYParkingMap
  17. 【parking_map.proto】移除parking_map.proto中定义的TileId，UrId，FloorId
  18. 【ehr.proto】语法升级至proto3， 改为import 《common3.proto》，新增目标停车位信息的接口字段
  meb功能相关:
  19. 【hmi】新增MEB功能状态的接口字段
  20. 【planning_hmi】新增MEB功能触发方向的接口字段
  超车变道确认功能相关：
  21. 【状态机】新增超车变道确认的接口字段
  周视感知模型拆分相关：
  22. 【周视感知】新增障碍物特征、车道线特征的接口定义
二、连线变更
  1. 适配新增障碍物特征、车道线特征以及改动可行驶区域topic发布节点的连线
三、其他改动
  1. 适配ehp接收定位语义信息的mainfest.yaml配置
  2. mainfest.yaml适配新增的障碍物特征、车道线特征
  3. mainfest.yaml适配可行驶区域topic发布节点的改动
  4. 删除周视感知配置文件中的绑核策略 (统一通过系统集成线程配置文件配置)

# interface2.11_pre_1231.8210
一、接口变更
二、连线变更
三、其他改动
  1. B面数据回流改为底软功能组

# interface2.11_pre_1230.8184
一、接口变更
  1. 【vs】新增车辆模式、驾驶模式相关的接口字段(仅E541使用)
二、连线变更
  1. 新增定位到hmi的连线
  2. 删除环视相机、fm_a的部分无用连线
三、其他改动

# interface2.11_pre_1229.8149
一、接口变更
二、连线变更
三、其他改动
  1. 移除诊断功能中的无用线程，修复cpu占用高的问题

# interface2.11_pre_1226
一、接口变更
  1. 【vs】新增方向盘转向扭矩的接口字段
二、连线变更
三、其他改动
  1. 修复进程退出超时问题
  2. 修改环视感知、周视车道线感知进程的最长退出时间

# interface2.11_pre_1222.7959
一、接口变更
  1. 【状态机】、【hmi】新增RPA功能相关接口字段
二、连线变更
三、其他改动
  1. 修复临终遗言日志存放路径
  2. 新增各节点CM日志的打印

# interface2.11_pre_1218.7846
一、接口变更
  1. 【静态融合】融合车道的最大数量增加至8
二、连线变更
三、其他改动
  1. 修复工程模式topic数据的回流链路
  2. 修复临终遗言接口引入的静态代码缺陷

# interface2.11_pre_1216.7801
一、接口变更
  1. 【规划】新增路口位置状态与红绿灯提醒的接口字段
二、连线变更
三、其他改动

# interface2.11_pre_1210.7642
一、接口变更
二、连线变更
三、其他改动
  1. 新增系统监控信息接口的ros转包适配
  2. 适配mbuf_pool处理逻辑迁移至各模块内部

# interface2.11_pre_1205.7564
一、接口变更
  1. 【hmi】、【状态机】新增泊车声音提醒接口字段(仅E541使用)
  2. 【ehr】新增雄狮地图信息的proto接口定义
二、连线变更
  1. A面系统监控消息转发至B面数据回流
三、其他改动
  1. 自研临终遗言接口替换华为临终遗言接口
  2. 修复华为日志崩溃问题
  
# interface2.11_pre_1202.7486
一、接口变更
  1. 【环视感知】新增proto3版本的3d_occ接口定义
二、连线变更
  1. 新增在线标定到fm_a的连线
三、其他改动
  1. 修复故障注入coredump问题

# interface2.11_pre_1128.7410
一、接口变更
  1. 【规划】新增施工区域提醒的接口字段
  2. 【磁盘管理】新增近端升级状态的接口字段
  3. 【相机】新增相机错误状态的接口定义(仅E541使用)
  4. 【ota】新增ota日志上云的接口定义
二、连线变更
  1. 新增phm_a/phm_b到fm_a、fm_a到ota的连线
  2. 新增环视相机到com_pnc连线
  3. 新增ota到fdl的连线
  4. ehp架构调整，由soc-b迁移至soc-a
三、其他改动
  1. 修改fdl接收到的alarm_info_fm_a_service的消息类型
  2. common_swc_lib中新增临终遗言的接口调用

# interface2.11_pre_1125.7268
一、接口变更
  1. 【定位】新增特征点检查时间戳的接口字段
  2. 【状态机】、【hmi】、【规划】新增窄路通行功能的接口字段
二、连线变更
  1. 新增在线标定debug信息到fdl的连线
三、其他改动
  1. 为所有节点新增/usr/bin等路径的环境变量

# interface2.11_pre_1121.7141
一、接口变更
  1. 【状态机】新增rads功能开始按钮、恢复按钮、发给hmi的轨迹点数量接口字段
  2. 【系统监控】新增系统负载监控的接口定义
二、连线变更
  1. 新增系统监控节点及该节点到数据回流的连线
  2. 新增在线标定、数据回流到健康管理节点的连线
三、其他改动
  1. 为fdl新增售后标定的功能状态

# interface2.11_pre_1118.6984
一、接口变更
  1. 【片间通信】新增删减msg_meta字段的状态机与控制消息的接口定义，降低片间通信传输的数据量
  2. 【规划】新增描述自车到cutin目标物碰撞时间的cutin_ttc接口字段
  3. 【数据回流】新增打包起始时间的接口字段
二、连线变更
三、其他改动
  1. 更新所有通信连线的qos配置，以优化通信负载
  2. 为数据回流节点新增环境变量的定义(仅对E541生效)

# interface2.11_pre_1114.6921
一、接口变更
  1. 【hmi】、【哨兵模式】删减消息结构体的接口字段，降低片间通信传输的数据量
二、连线变更
三、其他改动
  1. 更新所有节点的通信传输配置，以减少通信负载

# interface2.11_pre_1111.6797
一、接口变更
  1. 【数据回流】新增车云链路检测信息的接口定义
  2. 【磁盘管理】新增磁盘管理信息的接口定义
  3. 【状态机】、【hmi】新增二级告警提示与E541_DBC_6.0相关接口字段
  4. 【规划】新增泊入/泊出推荐接口字段
二、连线变更
  1. 新增fdl到fm_a的车云链路检测连线
  2. 新增磁盘管理相关的连线
三、其他改动
  1. 更新接口中通信传输类型，从string改成vector以减少内存拷贝

# interface2.11_pre_1110.6746
一、接口变更
二、连线变更
三、其他改动
  1. 修改NDSHM使用small qos，并且修改队列深度为2

# interface2.11_pre_1107.6688
一、接口变更
  1. 【状态机】修改行泊切换功能的状态枚举值
二、连线变更
三、其他改动
  1. 更新接口中通信传输类型，从string改成vector以减少内存拷贝

# interface2.11_pre_1104.6564
一、接口变更
  1. 【故障降级】新增前环视故障枚举值
  2. 【状态机】、【障碍物融合】、【规划】新增一键贴边功能相关接口字段
  3. 【状态机】新增RADS相关枚举值
  4. 【状态机】、【规划】新增窄车位标志接口字段
二、连线变更
三、其他改动
  1. 部分算法模块的Mainfest.json中新增状态，以适配售后标定功能
  2. 适配部分新接口的转包代码

# interface2.11_pre_1103.6503
一、接口变更
二、连线变更
三、其他改动
  1. common_swc_lib相关代码中修改线程队列栈的深度

# interface2.11_pre_1031.6486
一、接口变更
  1. 【感知】新增地面标识track_id接口字段
  2. 【规划】新增ldp脱手报警逻辑接口字段
  3. 【环视相机】新增相机热插拔接口定义
二、连线变更
  1. 重画tboxtime相关连线，修复该消息无法传输的问题
  2. 修复planning告警信息topic的命名
  3. 新增相机热插拔相关连线
  4. 新增数据回流到fm_a的连线，删除到fm_b的连线
三、其他改动
  1. 算法模块的Mainfest.json中新增WorkingNoCam状态，以适配相机热插拔功能

# interface2.11_pre_1024.6237
一、接口变更
  1. 【状态机】、【hmi】新增智驾小蓝灯状态
  2. 【片间通信】增加car_mode枚举值、泊车EPB干预、泊车挡位干预、PDC系统故障状态
  3. 【静态融合】新增路沿信息
  4. 【环视感知】修改3d_occ的proto定义，缩减数据量
  5. 【健康管理】新增健康管理报点消息定义
二、连线变更
  1. 新增3d_occ、健康管理功能相关连线
三、其他改动
  1. 删除alarm诊断功能中的部分打印
  2. 合入缺失的ros消息定义(仅工具链使用)

#### 已知问题

#### 兼容性
不向前兼容

# interface2.11_pre_1021.6122
一、连线变更
  1. 新增故障降级、哨兵模式、工程模式、图像脱敏等功能相关topic到fdl的连线，修复数据未回流的问题

#### 已知问题

#### 兼容性
不向前兼容

# interface2.11_pre_1018.6038
一、接口变更
  1. 【状态机】新增acc_status枚举值，对齐奇瑞需求差异项
  2. 【规划】新增LandingPoint有效性字段，优化变道框显示效果
二、连线变更
  1. 新增后视相机到fdl的连线，修复后视图像数据未回流的问题
三、其他改动
  1. 增加故障注入功能，其他模块无需适配

#### 已知问题

#### 兼容性
不向前兼容

# interface2.10_patch1
1. [修改代码] (planning_plan_c.h) 修复ready_to_go变量带初值问题
2. [修改代码] (manifest.yaml) 修复IflytekEhrStaticMap消息帧率文本错误
3. [修改代码] (src/project/m32t_reev_mdc510/bsw/remotecontrol_bsw_soc2mcu.h) DTC_internal_number数组扩展到255，不转包，底软内部使用

#### 已知问题
1. 部分结构体使用bool变量，C结构中无法使用，下版本将修改成boolean。double等也将同步到规范的float64。

#### 兼容性
不向前兼容


# interface2.10
1. [修改代码] (camera_perception_feature_c.h) 前融合方案变更AB面特征
2. [修改代码] (camera_perception_groundline_c.h) 接地线数量增加到576个
3. [修改代码] (camera_perception_parking_slot_c.h) 增加车位材质
4. [修改代码] (common_c.h) 
   1. 增加障碍物类型
   2. 增加车位材质枚举
   3. 增加减速虚/实线
   4. 增加掉头编制
   5. 车道类型增加对向车道和自车压导流带
5. [修改代码] (factory_calib_c.h) 工厂标定调FactoryCalibResults
6. [修改代码] (func_state_machine_c.h) 增加客户标定模式
7. [修改代码] (fusion_road_c.h) 增加车道数量信息
8. [修改代码] (hmi_inner_c.h) 增加rpa字段
9. [修改代码] (hmi_service_c.h) 增加rpa字段
10. [修改代码] (hmi_soc_outer_c.h) 
    1.  增加起步提示
    2.  增加rpa字段
    3.  增加rads激活提示
11. [修改代码] (ifly_parking_map_c.h) 增加localmap时间，用于输出插值时间
12. [修改代码] (planning_hmi_c.h) 增加起步提示
13. [修改代码] (planning_plan_c.h) 增加路口转向提示
14. [修改代码] (uss_wave_info_c.h) 增加uss高度宽度信息
15. [修改代码] (vehicle_service_c.h) 增加仪表信息
16. [新增代码] (calib_info_c.h) 增加标定结果输出信息
17. [新增代码] (camera_perception_scene_c.h) 增加交通场景信息（暂不连线）
18. [新增代码] (dtc_code_c.h) 增加dtc状态信息
19. [新增代码] (fdl_upload_state_c.h) 增加数据回传状态
20. [新增代码] (post_sale_calibration_c.h) 增加售后标定信息
21. [新增代码] (sensor_radar_c.h) 新增4d雷达点云
22. [新增代码] (legacy\4d_radar_for_data_collection\sensor_sinpro_radar.h) 新增4d雷达原始点云用于数采
23. [新增代码] (project\m32t_reev_mdc510\bsw\remotecontrol_bsw_soc2mcu.h) 新增项目所需底软远程诊断接口，只存文件
24. [新增代码] (project\m32t_reev_mdc510\bsw\tbox_Pps_Interface.h) 新增项目所需gptp接口，只存文件
25. [删除代码] (calib_button_c.h) 标定模块重新设计，删除冲突的旧文档
26. [删除代码] (calib_calc_result_c.h) 标定模块重新设计，删除冲突的旧文档
27. [代码不统计] (mdc) 
    1. 4D radar 感知方案
        1. 新增模块 radar_4d_service_exec(B面)
        2. 新增连线 radar_4d_service_exec(/iflytek/radar_fm_perception/4d_target) -> obstacle_fusion_exec
        3. 新增连线 radar_4d_service_exec(/iflytek/radar_fm_perception/4d_target) -> panorama_view_camera_perception_exec
        4. 新增连线 radar_4d_service_exec(/iflytek/radar_fm_perception/4d_obstacle) -> obstacle_fusion_exec
        5. 新增连线 radar_4d_service_exec(/iflytek/radar_fm_perception/4d_obstacle) -> panorama_view_camera_perception_exec
        6. 新增连线 radar_4d_service_exec(/iflytek/radar_fm_perception/4d_point_cloud) -> panorama_view_camera_perception_exec
        7. 新增连线 localization_exec(/iflytek/localization/egomotion) -> panorama_view_camera_perception_exec
        8. 新增连线 panorama_view_camera_perception_laneline_exec(/iflytek/prediction/prediction_result) -> soc_a_communication_exec/soc_b_communication_exec -> planning_exec
        9. 删除模块 prediction_exec
        10. 新增诊断连线 fm_b_service_exec(IflytekAlarmInfoRadar4dService) -> radar_4d_service_exec
        11. 新增连线 panorama_view_camera_perception_laneline_exec(/iflytek/camera_perception/objects) -> soc_a_communication_exec/soc_b_communication_exec -> obstacle_fusion_exec
        12. 新增连线 panorama_view_camera_perception_laneline_exec(/iflytek/camera_perception/objects) -> ace_exec
    2.  gnss接入tbox时间
        1.  新增连线 com_service_pnc_node_exec(/iflytek/sensor/tbox_time) -> gnss_service_node_exec
    3.  fm_a_service_exec接入状态机
        1.  新增连线 soc_b_communication_exec(/iflytek/fsm/soc_state) -> fm_a_service_exec
    4.  ace模块修改名称为calibration模块，并接入售后标定
        1.  修改模块 ace_exec -> calibration_exec
        2.  修改诊断连线 IflytekAlarmInfoAce 改名 IflytekAlarmInfoCalibration
        3.  新增连线 calibration_exec(/iflytek/calibration/calib_info) -> soc_a_communication_exec/soc_b_communication_exec -> finite_state_manager_exec
        4.  新增连线 calibration_exec(/iflytek/post_sale_calibration/activation) -> fm_a_service_exec
        5.  新增连线 calibration_exec(/iflytek/post_sale_calibration/stop) -> fm_a_service_exec
        6.  新增连线 calibration_exec(/iflytek/post_sale_calibration/result) -> fm_a_service_exec
        7.  新增连线 soc_b_communication_exec(/iflytek/localization/egomotion) -> calibration_exec
        8.  新增连线 camera_node_panorama_view_exec(/iflytek/sensor/camera/front30) -> soc_b_communication_exec/soc_a_communication_exec -> calibration_exec
        9.  新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/right_full_resolution_h265) -> calibration_exec
        10. 新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/front_full_resolution_h265) -> calibration_exec
        11. 新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/left_full_resolution_h265) -> calibration_exec
        12. 新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/rear_full_resolution_h265) -> calibration_exec
    5.  mcu到fm_a_service
        1.  新增连线 com_service_pnc_node_exec(/iflytek/sensor/mcu_to_fm_info) -> soc_a_communication_exec/soc_b_communication_exec -> fm_a_service_exec
    6.  后视相机在A面添加一份
        1.  camera_node_around_view_exec(/iflytek/sensor/camera/rear_MDCA) -> soc_a_communication_exec/soc_b_communication_exec -> camera_node_panorama_view_exec
        2.  camera_node_around_view_exec(/iflytek/sensor/camera/rear_MDCA) -> calibration_exec
        3.  camera_node_around_view_exec(/iflytek/sensor/camera/rear_full_resolution_h265_MDCA) -> factory_calibration_exec
    7.  超声感知模块移除
        1.  删除模块 uss_perception_exec
        2.  删除连线 uss_perception_exec(/iflytek/uss/uss_model_info) -> obstacle_fusion_exec
    8.  数据回流模块
        1.  新增模块 function_data_loop_exec
        2.  新增连线 calibration_exec(/iflytek/calibration/debug_info) -> function_data_loop_exec
        3.  新增连线 control_exec(/iflytek/control/debug_info) -> soc_b_communication_exec/soc_a_communication_exec -> function_data_loop_exec
        4.  新增连线 planning_exec(/iflytek/planning/debug_info) -> soc_b_communication_exec/soc_a_communication_exec -> function_data_loop_exec
        5.  新增连线 ehr_exec(/iflytek/ehr/debug_info) -> soc_b_communication_exec/soc_a_communication_exec -> function_data_loop_exec
        6.  新增连线 com_service_pnc_node_exec(/iflytek/adas_function/debug_info) -> soc_b_communication_exec/soc_a_communication_exec -> function_data_loop_exec
        7.  新增诊断连线 fm_a_service_exec(IflytekAlarmInfoFunctionDataLoop) -> radar_4d_service_exec
        8.  新增连线 fm_a_service_exec(/iflytek/alarm_info/fm_a_service) -> function_data_loop_exec
        9.  新增连线 fm_a_service_exec(/iflytek/dtc_info/fm_a_service) -> function_data_loop_exec
        10. 新增连线 function_data_loop_exec(/iflytek/fdl/upload_state) -> fm_a_service_exec
        11. 各模块输出全部联到fdl模块
    9.  IflytekConfigManager连线优化
        1.  IflytekConfigManager和OTA共用，省去OTA的连线
    10. 设置EM以root用户启动(generate_config.sh)
28. [代码不统计] (topic名称变更汇总) 
    1. 新增 /iflytek/dtc_info/fm_a_service      IFLYAllDtcState(dtc_code_c.h)
    2. 新增 /iflytek/fdl/upload_state           FdlUploadState(fdl_upload_state_c.h)
    3. 新增 /iflytek/radar_fm_perception/4d_target      RadarPerceptionObjectsInfo(radar_perception_objects_c.h)
    4. 新增 /iflytek/radar_fm_perception/4d_obstacle    RadarPerceptionObjectsInfo(radar_perception_objects_c.h)
    5. 新增 /iflytek/radar_fm_perception/4d_point_cloud RadarPointCloud(sensor_radar_c.h)
    6. 变更 /iflytek/ace/debug_info -> /iflytek/calibration/debug_info
    7. 新增 /iflytek/post_sale_calibration/activation PostSaleCalibActiveInfo/PostSaleCalibActiveInfoResult(post_sale_calibration_c.h) 不做数采
    8. 新增 /iflytek/post_sale_calibration/result PostSaleCalibGetCalibResReqInfo/PostSaleCalibResults(post_sale_calibration_c.h) 不做数采
    9. 新增 /iflytek/post_sale_calibration/stop PostSaleCalibStopReqInfo/PostSaleCalibStopResult(post_sale_calibration_c.h) 不做数采

#### 已知问题
1. 

#### 兼容性
不向前兼容


# interface2.9
1. [新增代码] (sensor_lidar_c.h) 为ace接入lidar rsp128信号
2. [新增代码] (can_raw_eth_data_c.h) 新增can_raw_eth_data_c.h， 记录原始CAN数据。
3. [修改代码] (camera_perception_occupancy_objects_c.h) 调整occ障碍物可见点数量和点数
4. [修改代码] (common_c.h) 增加后视镜请求信息
5. [修改代码] (control_command_c.h) 增加后视镜请求信息，rads
6. [修改代码] (ehr.proto) hpp输出增加所有楼层的路径长度信息。
7. [修改代码] (ehr_sdmap.proto) 兼容高德地图
8. [新增代码] (factory_calib_c.h) 增加工厂标定接口
9. [修改代码] (func_state_machine_c.h) 
   1. 新增泊车故障状态（原FunctionalState_ERROR不再用于泊车故障状态）
   2. 新增hpp建图匹配id
   3. 增加rads状态
10. [修改代码] (fusion_occupancy_objects_c.h) 增加有序点数量
11. [修改代码] (fusion_parking_slot_c.h) 透传isp时间戳，增加用户选中车位信息。
12. [修改代码] (ifly_localization_c.h) 
    1. 添加异常状态位
    2. 为HPP功能添加定位所在楼层
13. [修改代码] (ifly_mapping_status_c.h) 增加建图状态枚举。
14. [修改代码] (ifly_parking_map_c.h) 透传isp时间戳。
15. [修改代码] (ifly_parking_map_c.h) 
    1. 添加异常状态位
    2. 为HPP功能添加定位所在楼层
16. [修改代码] (hmi_inner_c.h) 
    1.  修复命名错误。
    2.  增加rads开关
17. [修改代码] (hmi_soc_outer_c.h) 
    1.  修复命名错误。
    2.  增加rads可视化输出
18. [修改代码] (planning_plam_c.h) 增加rads状态 
19. [修改代码] (uss_wave_info_c.h) usswave换新结构体
20. [修改代码] (vehicle_service_c.h) 增加后视镜状态
21. [代码不统计] (mdc) 
    1. ace接入lidar
        1. 新增模块 lidar_node_exec(A面)
        2. 新增连线 lidar_node_exec(/iflytek/sensor/lidar/rsp128) -> ace_exec
    2. 感知统一模型
        1. 新增连线 panorama_view_camera_perception_exec(/iflytek/camera_perception/feature) -> soc_b_communication_exec/soc_a_communication_exec ->  panorama_view_camera_perception_laneline
        2. 删除连线 soc_a_communication_exec(/iflytek/sensor/camera/front_120)  -> panorama_view_camera_perception_laneline
        3. 删除连线 soc_a_communication_exec(/iflytek/sensor/camera/front_120_eagle)  -> panorama_view_camera_perception_laneline
        4. 删除连线 soc_a_communication_exec(/iflytek/sensor/camera/side_fl)  -> panorama_view_camera_perception_laneline
        5. 删除连线 soc_a_communication_exec(/iflytek/sensor/camera/side_fr)  -> panorama_view_camera_perception_laneline
        6. 删除连线 soc_a_communication_exec(/iflytek/sensor/camera/side_rl)  -> panorama_view_camera_perception_laneline
        7. 删除连线 soc_a_communication_exec(/iflytek/sensor/camera/side_rr)  -> panorama_view_camera_perception_laneline
    3. apa优化
        1. 新增连线 com_service_pnc_node_exec(/iflytek/uss/usswave_info) -> soc_b_communication_exec/soc_a_communication_exec -> around_view_camera_perception
        2. 新增连线 localization_exec(/iflytek/localization/egomotion) -> soc_b_communication_exec/soc_a_communication_exec -> around_view_camera_perception
        3. 新增连线 localization_exec(/iflytek/localization/egomotion) -> com_service_pnc_node_exec
    4. 工厂标定
       1. 新增模块 factory_calibration_exec(A面)
       2. 新增连线 soc_a_communication_exec(/iflytek/sensor/camera/front_120_full_resolution_h265) -> factory_calibration_exec
       3. 新增连线 soc_a_communication_exec(/iflytek/sensor/camera/side_fl) -> factory_calibration_exec
       4. 新增连线 soc_a_communication_exec(/iflytek/sensor/camera/side_fr) -> factory_calibration_exec
       5. 新增连线 soc_a_communication_exec(/iflytek/sensor/camera/side_rl) -> factory_calibration_exec
       6. 新增连线 soc_a_communication_exec(/iflytek/sensor/camera/side_rr) -> factory_calibration_exec
       7. 新增连线 com_service_pnc_node_exec(/iflytek/sensor/imu) -> soc_b_communication_exec/soc_a_communication_exec -> factory_calibration_exec
       8. 新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/resize_image/right) -> factory_calibration_exec
       9. 新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/resize_image/front) -> factory_calibration_exec
       10. 新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/resize_image/left) -> factory_calibration_exec
       11. 新增连线 camera_node_around_view_exec(/iflytek/sensor/camera/surround/resize_image/rear) -> factory_calibration_exec
       12. 新增连线 factory_calibration_exec(/iflytek/factory_calibration/activation) -> fm_a_service_exec
       13. 新增连线 factory_calibration_exec(/iflytek/factory_calibration/result) -> fm_a_service_exec
       14. 新增连线 camera_node_around_view_exec(IflytekSensorGetSurroundCameraIntriParam) -> fm_a_service_exec
       15. 新增连线 camera_node_panorama_view_exec(IflytekSensorGetPanoramaCameraIntriParam) -> soc_b_communication_exec/soc_a_communication_exec -> fm_a_service_exec
       16. 新增连线 hmi_service_exec(/iflytek/hmi/factory_mode) -> soc_b_communication_exec/soc_a_communication_exec -> fm_a_service_exec
22. [代码不统计] (topic名称变更汇总) 
    1. 新增 /iflytek/dc/can/raw   CanRawEthData(can_raw_eth_data_c.h)
    2. 新增 /iflytek/sensor/lidar/rsp128 LidarMsgInfo(sensor_lidar_c.h)
    3. 新增 /iflytek/camera_perception/feature PanoramaViewCameraPerceptionFeature(panorama_view_camera_perception_c.h) 不做数采
    4. 新增 /iflytek/factory_calibration/activation FactoryCalibActiveInfo/FactoryCalibActiveInfoResult(factory_calibration_c.h) 不做数采
    5. 新增 /iflytek/factory_calibration/result FactoryCalibGetCalibResReqInfo/FactoryCalibrationResult(factory_calibration_c.h) 不做数采
    6. 新增 /iflytek/hmi/factory_mode FactoryCalibCarModeInfo(factory_calibration_c.h) 不做数采
    7. 新增 IflytekSensorGetSurroundCameraIntriParam FactoryCalibSvcIntriParamReqInfo/FactoryCalibSvcIntriParamArray(factory_calibration_c.h) 不做数采
    8. 新增 IflytekSensorGetPanoramaCameraIntriParam FactoryCalibPanoramaIntriParamReqInfo/FactoryCalibPanoramaIntriParamArray(factory_calibration_c.h) 不做数采
23. [代码不统计] (interface_description.md) 同topic名称变更汇总

#### 已知问题
1. 

#### 兼容性
不向前兼容

# interface2.8_patch1
1. [代码不统计] (mdc)  统一日志宏定义修复
2. [代码不统计] (manifest.yaml)  ehp修改到1hz

#### 已知问题
1. 

#### 兼容性
兼容interface2.8


# interface2.8
1. [修改代码] (src/c/*.h) 宏名称规范化。前缀统一加上算法模块名（或者算法模块大类）。结尾统一使用_MAX_NUM(最大数量)，_NUM(固定数量)，_STR_LEN(字符长度)。
2. [修改代码] (src/c/*.h) 修复文件名拼写错误，涉及文件camera_perception_deceler_c.h、camera_perception_groundline_c.h、camera_perception_objects_c.h、camera_perception_parking_slot_c.h、camera_perception_tsr_c.h。
3. [删除代码] (avm_output_image_info_debug_c.h) 废弃拼接图调试头文件
4. [修改代码] (camera_preception_groundline_c.h) 添加接地线可见点数量
5. [修改代码] (camera_perception_occupancy_objects_c.h) 添加接地线可见点数量
6. [修改代码] (common_c.h) 增加楼层id信息（建图相关模块公用）。公共障碍物类型枚举增加柱子和几种occ障碍物类型（50-59）
7. [修改代码] (func_state_machine_c.h) 增加记忆泊车内容。增加系统状态信息。
8. [新增代码] (fusion_deceler_c.h) 新增融合减速带头文件及相关结构体。
9. [新增代码] (fusion_groundline_c.h) 新增融合接地线头文件及相关结构体。
10. [修改代码] (fusion_parking_slot_c.h) 增加融合车位数据来源，和限位器数据来源。
11. [新增代码] (fusion_spatial_parking_slot_c.h) 增加融合空间车位头文件及相关结构体。
12. [修改代码] (hmi_inner_c.h) 增加记忆泊车内容。
13. [修改代码] (hmi_soc_outer_c.h) 增加记忆泊车内容。增加泊入泊出方向的内容。
14. [修改代码] (ifly_mapping_status_c.h) 增加记忆泊车内容。
15. [修改代码] (ifly_parking_map_c.h) 增加记忆泊车内容。
16. [修改代码] (planning_hmi_c.h) 增加记忆泊车内容。增加泊入泊出方向的内容。
17. [修改代码] (prediction_c.h) 新增障碍物轨迹点个数
18. [新增代码] (uss_model_info_c.h) 新增超声模型结果头文件及相关结构体。
19. [修改代码] (uss_perception_info_c.h) 超声感知结果改由障碍物融合输出，删除车位相关信息（车位信息在fusion_spatial_parking_slot_c.h），
20. [代码不统计] (mdc) 
    1.  超声架构变更
        1. 删除连线 soc_b_communication_exec(/iflytek/fusion/parking_slot)  -> uss_perception_exec 
        2. 删除连线 uss_perception_exec(/iflytek/uss/uss_perception_info) -> planning_exec
        3. 删除连线 uss_perception_exec(/iflytek/uss/uss_perception_info) -> obstacle_fusion_exec
        4. 删除连线 uss_perception_exec(/iflytek/uss/uss_perception_info) -> soc_b_communication_exec/soc_a_communication_exec -> mega_exec
        5. 新增连线 uss_perception_exec(/iflytek/uss/uss_model_info) -> obstacle_fusion_exec
        6. 新增连线 com_service_pnc_node_exec(/iflytek/uss/usswave_info) -> obstacle_fusion_exec
        7. 新增连线 soc_b_communication_exec(/iflytek/fusion/parking_slot) -> obstacle_fusion_exec
        8. 新增连线 obstacle_fusion_exec(/iflytek/fusion/uss_perception_info) -> planning_exec
        9. 新增连线 obstacle_fusion_exec(/iflytek/fusion/spatial_parking_slot) -> soc_b_communication_exec/soc_a_communication_exec -> mega_exec
    2.  记忆泊车功能
        1. 新增模块 ehp_exec(A面)
        2. 新增连线 gnss_service_node_exec(/iflytek/sensor/gnss) -> finite_state_manager_exec
        3. 新增连线 gnss_service_node_exec(/iflytek/sensor/gnss) -> soc_b_communication_exec/soc_a_communication_exec -> ehp_exec
        4. 新增连线 localization_exec(/iflytek/localization/semantic_info) -> soc_b_communication_exec/soc_a_communication_exec -> mega_exec
        5. 新增连线 localization_exec(/iflytek/localization/semantic_info) -> finite_state_manager_exec
        6. 新增连线 mega_exec(/iflytek/mega/mapping_status) -> soc_a_communication_exec/soc_b_communication_exec -> finite_state_manager_exec
        7. 新增连线 mega_exec(/iflytek/mega/local_map) -> soc_a_communication_exec/soc_b_communication_exec -> localization_exec
        8. 新增连线 mega_exec(/iflytek/mega/mapping_status) -> ehp_exec
        9. 新增连线 ehp_exec(/iflytek/ehp/parking_map) -> soc_a_communication_exec/soc_b_communication_exec -> localization_exec
        10. 新增连线 ehp_exec(/iflytek/ehp/parking_map) -> soc_a_communication_exec/soc_b_communication_exec -> ehr_exec
        11. 新增连线 ehp_exec(/iflytek/ehp/parking_map) -> soc_a_communication_exec/soc_b_communication_exec -> finite_state_manager_exec
        12. 新增连线 ehp_exec(/iflytek/ehp/parking_map_file_info) -> mega_exec
        13. 新增连线 ehr_exec(/iflytek/ehr/static_map) -> planning_exec
        14. 新增连线 ehr_exec(/iflytek/ehr/static_map) -> soc_b_communication_exec/soc_a_communication_exec -> static_fusion_exec
        15. 新增连线 around_view_camera_perception_exec(/iflytek/camera_perception/parking_lane_line) -> mega_exec
        16. 新增连线 around_view_camera_perception_exec(/iflytek/camera_perception/3d_general_objects) -> mega_exec
        17. 新增连线 static_fusion_exec(/iflytek/fusion/speed_bump) -> soc_a_communication_exec/soc_b_communication_exec -> planning_exec
        18. 新增连线 ehp_exec(/iflytek/ehp/map_manager) -> soc_a_communication_exec/soc_b_communication_exec -> finite_state_manager_exec
        19. 新增连线 soc_b_communication_exec(/iflytek/fsm/soc_status) -> ehp_exec
        20. 新增连线 fm_a_service_exec(IflytekAlarmInfoEhp) -> ehp_exec
    3.  ace输入变更
        1.  新增连线 camera_node_panorama_view_exec(/iflytek/sensor/camera/front_120_full_resolution_h265) -> soc_b_communication_exec/soc_a_communication_exec -> ace_exec
        2.  删除连线 soc_a_communication_exec(/iflytek/sensor/camera/front_120)  -> ace_exec
        3.  删除连线 soc_a_communication_exec(/iflytek/sensor/camera/front_120_eagle)  -> ace_exec
21. [代码不统计] (topic名称变更汇总) 
    1.  新增 /iflytek/fusion/spatial_parking_slot   FusionSpatialParkingSlot(fusion_spatial_parking_slot_c.h)
    2.  新增 /iflytek/fusion/uss_perception_info    UssPerceptInfo(uss_percept_info_c.h)
    3.  变更 /iflytek/fusion/ground_line            GroundLinePerceptionInfo(camera_preception_groundline_c.h) -> FusionGroundLineInfo(fusion_groundline_c.h)
    4.  新增 /iflytek/fusion/speed_bump             FusionDecelerInfo(fusion_deceler_c.h)
    5.  新增 /iflytek/localization/semantic_info    SemanticInfo(ifly_parking_map_c.h)
    6.  变更 /iflytek/ehp/parking_map               ParkingInfo(ifly_parking_map_c.h) -> ParkingInfo(ifly_parking_map.proto)
    7.  新增 /iflytek/ehp/map_manager               EhpOutput(ifly_parking_map_c.h)
    8.  删除 /iflytek/camera_perception/vision_slot_all_info
    9.  删除 /iflytek/camera_perception/around_view_camera_perception_debug_info
    10. 删除 /iflytek/camera_perception/avm_reload_result
    11. 删除 /iflytek/uss/uss_perception_info
    12. 新增 /iflytek/uss/uss_model_info            UssModelInfo(uss_model_info_c.h)
    13. 新增 /iflytek/sensor/camera/front_120_full_resolution_h265  Camera_Image_Info(sensor_image_c.h)     不做数采
    14. 新增 /iflytek/ace/debug_info                StructContainer(struct_container.proto)
22. [代码不统计] (interface_description.md) 同topic名称变更汇总

#### 已知问题
1. 

#### 兼容性
不向前兼容


# interface2.7_patch3
1.  [代码不统计] (mdc) /iflytek/sdmap/sdmap_info设置成large_buffer_qos.xml，通信buffer增大

#### 已知问题
1. 

#### 兼容性
不向前兼容


# interface2.7_patch2
1.  [代码不统计] (mdc) 
    1. mega和static fusion移到A面，诊断模块和AB面通信接口作相关调整（详细改动请自行查看拓扑）
    2. 定位增加输入/iflytek/ehr/sdmap_info

#### 已知问题
1. 

#### 兼容性
不向前兼容


# interface2.7
1. [修改代码] (camera_perception_lane_lines_c.h) 增加车道线点的置信度、颜色、线型
2. [修改代码] (camera_perception_occupancy_objects_c.h) 通用障碍物点的最大数量改为（320+256）*2
3. [修改代码] (ehr_sdmap.proto) 
   1. 修改方向枚举的取值范围为1-12，使之符合四维协议
   2. 增加一个字段表示EHR地图输出的状态，标识地图正常导航中还是导航退出还是EHR接收的数据有异常。
4. [修改代码] (func_state_machine_c.h) 增加泊出相关字段，增加泊车状态
5. [修改代码] (fusion_road_c.h) 增加车道线参考点置信度，增加道路边沿轨迹点的置信度
6. [修改代码] (hmi_inner_c.h) 增加泊出相关字段，枚举增加invalid字段
7. [修改代码] (hmi_soc_outer_c.h) 增加泊出相关字段，增加转向开关状态
8. [修改代码] (ifly_localization_c.h) 为llh坐标添加是否可用标记
9. [新增代码] (prediction_debug.proto) 新增prediction调试信息接口，仅当字符串录制
10. [修改代码] (uss_perception_info_c.h) 修复uss接口中单词拼写错误问题
11. [代码不统计] (mdc) 
    1. mmc和mds目录统一规整为mdc目录，弃用mds，使用mdc下的generate_config.sh生成代码
    2. 删除 片间通信到fsm的topic：/iflytek/adas_function/adas
    3. 新增 状态机输出/iflytek/fsm/soc_state 到预测
    4. 新增 /iflytek/camera_perception/lane_lines 到预测
    5. 新增 /iflytek/vehicle service 到预测
    6. 新增 ace_exec模块，A面，连线如下
       1. 新增 /iflytek/sensor/imu -> 经soc_b_communication_exec -> 到ace_exec
       2. 新增 /iflytek/fusion/objects -> 经soc_b_communication_exec -> 到ace_exec
       3. 新增 ace_exec输出/iflytek/ace/calib_status -> 经soc_b_communication_exec -> 到状态机
       4. 新增 /iflytek/sensor/camera/surround/resize_image/right 到ace_exec
       5. 新增 /iflytek/sensor/camera/surround/resize_image/front 到ace_exec
       6. 新增 /iflytek/sensor/camera/surround/resize_image/left 到ace_exec
       7. 新增 /iflytek/sensor/camera/surround/resize_image/rear 到ace_exec
       8. 新增 /iflytek/sensor/camera/front_120 到ace_exec
       9. 新增 /iflytek/sensor/camera/front_120_eagle 到ace_exec
       10. 新增 /iflytek/sensor/camera/side_fl 到ace_exec
       11. 新增 /iflytek/sensor/camera/side_fr 到ace_exec
       12. 新增 /iflytek/sensor/camera/side_rl 到ace_exec
       13. 新增 /iflytek/sensor/camera/side_rr 到ace_exec
       14. 新增 /iflytek/fsm/soc_state 到ace_exec
       15. 新增 /iflytek/vehicle_service 到ace_exec
       16. 新增 诊断接口IflytekAlarmInfoAce 到ace_exec
12. [代码不统计] (interface_description.md) 
    1. 调整prediction调试信息接口的topic名称为/iflytek/prediction/prediction_debug_info
    2. 片间通信无需单独发送topic：/iflytek/adas_function/adas
    3. 增加tbox的topic：/iflytek/sensor/gnss_tbox 仅数采录制
13. [代码不统计] (topic名称变更汇总) 
    1.  新增 /iflytek/sensor/gnss_tbox
    2.  变更 /iflytek/prediction/debug_info -> /iflytek/prediction/prediction_debug_info
    3.  删除 /iflytek/adas_function/adas
14. [新增代码] (ci) 新增接口ci工具初版，使用方法在README.MD中

#### 已知问题
1. 

#### 兼容性
不向前兼容


# interface2.6
1. [修改代码] (common_c.h) 障碍物类型增加车位、减速带、地锁、限位器
2. [新增代码] (hmi_service_c.h) 
   1. 增加CHERY_T26车型的hmi内部接口
   2. 增加CHERY_E0Y车型的ADAS输出信息和PDC报警信息
3. [修改代码] (camera_preception_tsr_c.h) 修改通行状态信息的使用方法
4. [新增代码] (fsm_mcu_out_c.h) 新增系统状态机接口
5. [新增代码] (hmi_mcu_out_c.h) 新增mcu的算法hmi接口
6. [修改代码] (func_state_machine_c.h) 增加功能退出原因枚举：距离收费站过近
7. [修改代码] (hmi_inner_c.h) 增加数据录制按键
8. [修改代码] (hmi_soc_outer_c.h) 车道线、障碍物等，直接使用算法数据结构，删除无用项
9.  [删除代码] (syst_state_machine.h) 删除废弃的代码
10. [修改代码] (uss_perception_info_c.h)
    1.  增加全局车位角点坐标和局部坐标点，可视化适配此字段local_corner_point
    2.  删除UssObjectsInfo
11. [代码不统计] (mds mmc) 
    1. 模块改名 A面 around_view_camera_perception_objects -> around_view_camera_perception
    2. 接口改名 IflytekAlarmInfoAroundViewCameraPerceptionObjects -> IflytekAlarmInfoAroundViewCameraPerception
    3. 状态机增加输入：相机感知车道线，相机感知接地线，相机感知减速带
12. [代码不统计] (interface_description.md) 
    1. 周视相机和环视相机帧率确定为:研发版本MDC10hz；大算力版本20hz；所有数采版本MDC20hz；
    2. 修改系统状态机输出结构体说明
    3. 新增MCU上的hmi的输出说明
13. [代码不统计] (topic名称变更汇总) 无

#### 已知问题
1. FDC版本暂未设计fsm-hmi结构，实现前不可升到此版本

#### 兼容性
不向前兼容


# interface2.5
1. [修改代码] (src/c所有结构体文件) 
   1. 算法输出定义标准化msg_header和msg_meta
   2. 传感器输出定义标准化msg_header和sensor_meta
   3. 统一动态数组长度字段名
   4. 新增SystemVersion定义
2. [修改代码] (camera_perception_lane_lines_c.h) 增加车道线最大数量，减少停止线最大数
3. [修改代码] (camera_preception_tsr_c.h) 根据规划需求，更新TSR接口
4. [修改代码] (ehr_sdmap.proto) 增加收费站信息、坡度和曲率信息
5. [修改代码] (fm_info_c.h) 新增告警次数字段和header字段，此结构体仅数据录制相关
6. [修改代码] (fusion_parking_slot_c.h) 更改限位器结体， 单个库位可包含多个限位器
7. [修改代码] (fusion_road_c.h) 新增融合停止线和道路标识
8. [修改代码] (ifly_parking_map_c.h) 
    1. 调整ParkingInfo数组size，大小降到60多kb
    2.  更改限位器结体， 单个库位可包含多个限位器
9.  [删除代码] (localization_c.h) 接口废弃，切换到新接口/iflytek/localization/egomotion
10. [删除代码] (c_to_json.h) 不再维护
11. [删除代码] (c_to_ros_macro.h) 不再维护
12. [代码不统计] (mds mmc) 
    1. 模块改名 around_view_camera_node_exec -> camera_node_around_view_exec
    2. 接口改名 IflytekAlarmInfoCameraNodeAroundView -> IflytekAlarmInfoCameraNodeAroundView
    3. 模块改名 panorama_view_camera_node_exec -> camera_node_panorama_view_exec
    4. 接口改名 IflytekAlarmInfoPanoramaViewCameraNode -> IflytekAlarmInfoCameraNodePanoramaView
    5. 新增/iflytek/fusion/ground_line接口
    6. 新增/iflytek/camera_perception/traffic_sign_recognition接口到规划和状态机
    7. 修改名称/iflytek/sdmap/sdmap_info
    8. 删除所有/iflytek/localization/ego_pose
    9. 增加定位输出到sdmap
13. [代码不统计] (interface_description.md) 
    1. 增加融合接地线
    2. 定位频率由50hz上升为100hz
    3. 新增sdmapinfo消息定义
14. [代码不统计] (topic名称变更汇总) 请在interface_description.md中自行查找
    1. 新增 /iflytek/fusion/ground_line
    2. 新增 /iflytek/camera_perception/traffic_sign_recognition     首次启用
    3. 新增 /iflytek/sdmap/sdmap_info
    4. 新增 /iflytek/ehr/debuginfo                                  调试信息仅录制
    5. 删除 /iflytek/localization/ego_pose

#### 已知问题
1. FDC版本暂未设计fsm-hmi结构，实现前不可升到此版本

#### 兼容性
不向前兼容


# interface2.4.6
1. [修改代码] (fusion_road_c.h) 增加每条车道中线长度，100个点增加到125个
2. [新增代码] (hmi_service_c.h) fsm-hmi架构变更后，新增底软自用hmi_service_c接口，所有车型均放在一个文件中，新增2个topic
3. [修改代码] (func_state_machine_c.h) fsm-hmi架构变更后，状态机状态变更，增加一些开关项，topic名称变更为/iflytek/fsm/soc_state
4. [新增代码] (hmi_inner_c.h) fsm-hmi架构变更后，新增hmi_service到状态机的数据，topic名称/iflytek/hmi/inner
5. [修改代码] (hmi_soc_outer_c.h) fsm-hmi架构变更后，整合所有fsm需要发送到hmi_service的消息，新增topic名称/iflytek/fsm/hmi_soc_outer，原topic删除
6. [删除代码] (hmi_hpp_outer_c.h) fsm-hmi架构变更后，废弃此接口，原topic删除
7. [删除代码] (hmi_mcu_inner_c.h) fsm-hmi架构变更后，废弃此接口，原topic删除
8. [删除代码] (hmi_soc_inner_c.h) fsm-hmi架构变更后，废弃此接口，原topic删除
9. [删除代码] (hmi_hpp_inner_c.h) fsm-hmi架构变更后，废弃此接口，原topic删除
10. [修改代码] (planning_hmi_c.h) 增加noa相关hmi信息
11. [修改代码] (vehicle_service_c.h) 增加部分车辆信息和枚举
12. [代码不统计] (interface_description.md) 更新文档
13. [代码不统计] (topic名称变更汇总) 请在interface_description.md中自行查找
    1.  新增 /iflytek/hmi/mcu_to_soc_bsw/chery_e0y_mdc510               底软
    2.  新增 /iflytek/hmi/soc_to_mcu_bsw/chery_e0y_mdc510               底软
    3.  新增 /iflytek/hmi/inner                                         底软
    4.  新增 /iflytek/fsm/hmi_soc_outer                                 fsm
    5.  变更 /iflytek/system_state/soc_state ->/iflytek/fsm/soc_state   fsm
    6.  删除 /iflytek/hmi/mcu_inner    hmi
    7.  删除 /iflytek/hmi/soc_inner    hmi
    8.  删除 /iflytek/hmi/hpp_inner    hmi
    9.  删除 /iflytek/hmi/hpp_outer    hmi
    10. 删除 /iflytek/hmi/soc_outer    hmi

#### 已知问题
1. FDC版本暂未设计fsm-hmi结构，实现前不可升到此版本

#### 兼容性
不向前兼容

# interface2.4.5.4
1. [代码不统计] (mds) mega模块增加hmi/soc_inner连线
2. [代码不统计] (interface_description.md) 
   1. 修复片间通信汇总信息书写错误

#### 已知问题
暂无

#### 兼容性
不向前兼容

# interface2.4.5.3
1. [代码不统计] (mds) 规划模块增加超声输入
2. [代码不统计] (interface_description.md) 
   1. 修复MDC偏见通信imu接口信息书写错误

#### 已知问题
暂无

#### 兼容性
不向前兼容

# interface2.4.5.2
1. [修改代码] (type_convert) 修复转包的一些问题
2. [代码不统计] (mds) 修复b面障碍物感知没接状态机的问题
3. [代码不统计] (interface_description.md) 
   1. 补充系统版本号的接口信息
   2. 修复MDC偏见通信imu接口信息书写错误

#### 已知问题
暂无

#### 兼容性
兼容至interface2.4.5

# interface2.4.5.1
1. [修改代码] (fm_info_c.h) 修复文件开头宏定义问题

#### 已知问题
暂无

#### 兼容性
兼容interface2.4.5

# interface2.4.5
1. [修改代码] (fusion_objects_c) 调整障碍物融合输出障碍物数量为128个，宏名称变更
2. [修改代码] (fusion_objects_compress_c.h) 调整输出给AEB的障碍物数量的宏名称，数值不变
3. [新增代码] (fusion_occupancy_objects_c.h)  新增融合通用类型障碍物输出接口
4. [新增代码] (common_c.h) 
    1.  增加车道线颜色增加红色、彩色
    2.  增加车道线虚实类型增加减速线、虚拟线、待转线
    3.  增加车道转向类型枚举
    4.  增加车道分合流方向类型枚举
    5.  增加车道分合流信息结构体
    6.  增加车道线类型增加护栏等
    7.  增加车道类型增加右转弯待转区、岔道
    8.  增加车道通行方向信息结构体
    9.  增加车道类型信息结构体
    10. 新增接地线类型GroundLineType，从camera_preception_groundline_c.h移入
5. [修改代码] (common_c.h) 
   1. 修改Header下seq字段的用途，在消息发生出计数，mcu产生的消息在mcu处计数，转发者不计数只透传
   2. 障碍物位置、速度、加速度信息拓展到3维
   3. 调整header中InputHistoryTimestamp的使用方法
6. [删除代码] (camera_preception_groundline_c.h) 删除接地线类型GroundLineTypeEnum，复用common中的GroundLineType
7. [修改代码] (ifly_parking_map_c.h) 
   1. 道路障碍物类型改为用GroundLineType枚举，移除原有类型定义RoadObstacleType
   2. 降低暂时不用的数组大小，以降低结构体size，以后再重构此结构体
8. [修改代码] (ifly_localization_c.h) 新增body系下加速度、速度以及角速度，enu坐标系下的位置，以及global变换、位置是否可用
9. [修改代码] (uss_perception_info_c.h) 增加超声波障碍物点云全局左边和局部坐标，移除障碍物位置信息；增加超声波障碍物距离跟踪信息
10. [修改代码] (sensor_image_c.h) 调整Camera_Image_Info的id、格式、视角、分辨率等字段的字符串最大长度
11. [新增代码] (sensor_image_c.h) 增加相机发布数据时间戳
12. [修改代码] (camera_perception_lane_lines_c.h) 增加车道信息、地面标识列表、停止线、禁止线
13. [新增代码] (camera_perception_diagnostic_info.proto) 新增感知引擎自用的诊断调试信息
14. [删除代码] (fusion_road_c.h) 车道的转向箭头枚举、分合流结构体、分段信息结构体移到common
15. [修改代码] (fusion_road_c.h) 车道线分段数量调整到10
16. [新增代码] (camera_preception_parking_slot_c) 新增地锁信息
17. [新增代码] (camera_perception_occupancy_objects_c.h) 新增文件，新增通用结构体
18. [新增代码] (ehr_sdmap.proto) 新增ehr高精地图proto
19. [新增代码] (fm_info_c.h)  新增故障管理模块输出接口
20. [代码不统计] (cmakelists) 增加-fPIC选项，修复MDC编译问题
21. [代码不统计] (mmc) 增加MDC的MMC工程（绘图相关）
22. [代码不统计] (mds) 增加MDC的MDS信息（生产代码相关）
23. [代码不统计] (interface_description.md) 
    1.  超声波感知结果topic命名变更
    2.  新增视觉感知车道线topic
    3.  视觉感知调试车道线topic命名变更，帧率变更
    4.  新增可变车道线调试信息topic
    5.  通用障碍物结构体变更
    6.  新增3D通用障碍物topic
    7.  新增3Dbox信息topic
    8.  调整hmi/soc_outer、hmi/hpp_outer的帧率为50帧
24. [代码不统计] (type_convert) 版本工具适配

#### 已知问题

暂无

#### 兼容性
不向前兼容


# interface2.4.4
1. [修改代码] (sensor_gnss_c.h) 统一gnss接口
2. [删除代码] (sensor_pbox_gnss_c.h) 去除pbox_gnss接口，gnss统一到sensor_gnss_c.h
3. [修改代码] (sensor_imu_c.h) 统一gnss接口
4. [删除代码] (sensor_pbox_imu_c.h) 去除pbox_imu接口，gnss统一到sensor_imu_c.h
5. [新增代码] (common_c.h) 增加相机对时记录结构体定义
6. [新增代码] (camera_perception_lane_lines_c.h) 增加相机对时记录
7. [新增代码] (camera_preception_objects_c.h) 增加相机对时记录
8. [代码不统计] (type_convert) 版本工具适配

#### 已知问题

暂无

#### 兼容性
不向前兼容


# interface2.4.3
1. [修改代码] (type_convert) 修改数据转换部分代码

#### 已知问题

暂无

#### 兼容性

兼容至interface2.4.2


# interface2.4.2
1. [修改代码] (interface_description.md) 增加mega的输出；静态融合替换道路融合模块，接口不变；增加通用障碍物输出；增加E0Y片间通信传输说明
2. [修改代码] (ifly_localization_c.h) 删除一个无用的宏定义

#### 已知问题

暂无

#### 兼容性

暂无


# interface2.4.1
1. [修改代码] (vehicle_service_c.h) 方向灯拨杆状态增加左右轻拨状态
2. [新增代码] (camera_preception_tsr_c.h) 增加相机感知TSR输出初版接口
3. [修改代码] (hmi_soc_outer_c.h) 优化hmi输出数据结构，适配E0Y车型
4. [修改代码] (ifly_parking_map_c.h) 降低记忆泊车建图数据结构大小；增加车位来源信息；
5. [修改代码] (common_c.h) 障碍物类型增加，障碍物增加灯光状态;融合车位来源枚举移到此目录
6. [修改代码] (fusion_road_c.h) 新增车道线宽度
7. [修改代码] (fusion_parking_slot_c.h) 增加车位来源信息，优化枚举使用
8. [新增代码] (time_synchronization_c.h) 增加时间戳接口，用于在无法授时的情况下，向工控机提供域控时间
9. [删除代码] (private) 删除proto格式的调试信息

#### 已知问题

暂无

#### 兼容性

暂无

# interface2.4.0.5
1. [修改代码] (common_c.h) 删除common里通用点集PointsSet2f和PointsSet3f
2. [修改代码] (avm_output_image_info_debug_c.h) 环视感知debug_info中新增4路相机曝光时间戳、拼接图输出info中新增图像格式信息、修改最大车位数量
2. [修改代码] (fusion_road_c.h) 修改车道线点数，增加中心线点数
3. [修改代码] (hmi_soc_outer_c.h) 缩减中控显示障碍物数量和车道线数量，降低通信负载
4. [修改代码] (radar_perception_objects_c.h) 障碍物增加到32个
5. [修改代码] (fusion_objects_compress_c.h) 新增障碍物融合发送到MCU结构体
6. [修改代码] (private/planning) 增加调试字段

#### 已知问题

暂无

#### 兼容性

暂无


# interface2.4.0
1. [修改代码] (all)切换为c结构体

### 接口变更确认项
1. 是否可以使用struct内嵌enum，有重名风险
- 本次都改成前缀区分，不嵌套
2. 所有int定义的枚举成员都要额外增加enum显式定义枚举值
- 本次增加enum定义
3. 所有数字/UNDEFINED定义的数组大小都要用宏定义替换
- 本地确定所有数组大小，非固定长度数组要确保增加_size字段
4. 是否可以使用二维数组，转json不支持
- 本次改成结构体一维数组
5. c/iflyauto_icc.h是否需要保留
- done，已删
6. mcu_base_interface/下哪些需要保留
- done，mcu不需要工具转头文件了，直接使用c接口
7. 所有.h指定owner，确认内容是否与2.3.2接口一致、正确
- 会后输出owner列表，周五完成确认
8. 统一注释格式
- 各owner自行把握补充或者清理注释，删除改成enum的值罗列。
9. owner确认interface_description.md下内容正确
10. topic名字统一：

| old | new |
| --- | --- |
| /prediction_debug_info | /iflytek/prediction/debug_info |
| /iflyauto/sensor/camera/front/image | /iflytek/sensor/camera/front/image |
| /iflyauto/sensor/camera/surround/image | /iflytek/sensor/camera/ |surround/| image |
| /iflyauto/sensor/camera/panorama/image | /iflytek/sensor/camera/ |panorama/| image |
| /iflyauto/dc/camera/front_30 | /iflytek/dc/camera/front_30 |
| /iflyauto/dc/camera/front_120 | /iflytek/dc/camera/front_120 |
| /iflyauto/dc/camera/rear | /iflytek/dc/camera/rear |
| /iflyauto/dc/camera/side_fl | /iflytek/dc/camera/side_fl |
| /iflyauto/dc/camera/side_fr | /iflytek/dc/camera/side_fr |
| /iflyauto/dc/camera/side_rl | /iflytek/dc/camera/side_rl |
| /iflyauto/dc/camera/side_rr | /iflytek/dc/camera/side_rr |
| /iflyauto/dc/camera/surround_right | /iflytek/dc/camera/surround_right |
| /iflyauto/dc/camera/surround_front | /iflytek/dc/camera/surround_front |
| /iflyauto/dc/camera/surround_left | /iflytek/dc/camera/surround_left |
| /iflyauto/dc/camera/surround_rear | /iflytek/dc/camera/surround_rear |

10. 片间通讯下行topic
- 没有localization，有planning，topic名字不带uncompress，参照interface_description.md
11. 新定位接口跟随hpp合主线
- 在c结构体切换之后，届时定位上下游模块统一切换

# interface2.3.7

1. [修改代码] (private/planning) 增加refline调试信息

#### 已知问题

暂无

#### 兼容性

暂无

# interface2.3.6

1. [修改代码] (private/planning) 增加lateral path optimization调试信息
2. [修改代码] (test) 增加测试编译的工程
3. [修改代码] (cmakelists) 完善编译脚本

#### 已知问题

暂无

#### 兼容性

暂无

# interface2.3.5

1. [修改代码] (private/planning)gap selector合入时新增proto文件，主要用于pybind与问题复现

#### 已知问题

暂无

#### 兼容性

暂无

# interface2.3.4

1. [修改代码] (private/planning)增加规划ilqr lateral path optimizer的调试信息

#### 已知问题

暂无

#### 兼容性

暂无

# interface2.3.3

1. [修改代码] (private/perception)增加环视感知的调试信息
2. [修改代码] (private/planning)增加规划的调试信息

#### 已知问题

暂无

#### 兼容性

暂无

# interface2.3.2
1. [修改代码] (private)增加规划的调试信息

####  已知问题  
暂无

####  兼容性  
暂无

# interface2.3.1
2. [修改代码] (ifly_localization)修改proto文件，修改topic名称，增加欧拉角输出
3. [修改代码] (ifly_parking_map)修改proto文件，定义泊车地图输出
4. [修改代码] (hmi_hpp_inner)修改proto文件，补充hpp所需字段
5. [修改代码] (hmi_hpp_outer)修改proto文件，补充hpp所需字段
6. [新增代码] (ifly_mapping_status)新增proto文件，输出地图状态
7. [修改代码] (planning_hmi)修改proto文件，增加记忆泊车行驶距离和起点提示
8. [修改代码] (private)修改规划的调试信息
9. [删除代码] (apa_simulation_config)废弃调试信息
10. [删除代码] (parking_fusion_debug)废弃调试信息
11. [代码不统计] (interface_description)更新文档

####  已知问题  
暂无

####  兼容性  
暂无


# interface2.3.0

####  修改说明  
1. [代码不统计] (ehr)修改proto文件，增加注释
2. [新增代码] (ifly_localization)新增proto文件，重新定义定位输出
3. [新增代码] (ifly_parking_map)新增proto文件，定义泊车地图输出
4. [新增代码] (deceler_perception)新增proto文件，新增感知减速带结果
5. [新增代码] (groundline_perception)新增proto文件，新增感知接地线结果
6. [修改代码] (func_state_machine)修改proto文件和头文件，增加hpp相关状态
7. [修改代码] (hmi_soc_outer)修改proto文件，修正一个参数命名错误
8. [新增代码] (hmi_hpp_inner)新增proto文件，增加hpp上位机输入消息
9. [新增代码] (hmi_hpp_outer)新增proto文件，增加发送给hpp上位机的消息
10. [修改代码] (fusion_road)修改proto文件和头文件，增加hpp相关状态
11. [修改代码] (parking_fusion)修改proto文件，增加是否可泊状态
12. [修改代码] (parking_slot_list)修改proto文件，修改置信度数据类型，增加传感器时间戳
13. [修改代码] (parking_fusion)修改proto文件，增加hpp相关状态
14. [修改代码] (planning_hmi)修改proto文件，增加hpp相关的hmi提示信息
15. [修改代码] (planning_plan)修改proto文件，增加hpp规划状态
16. [修改代码] (adas_function_debug)修改proto文件和头文件，增加调试信息
17. [修改代码] (radar_perception_objects)修改proto文件和头文件，增加计数id
18. [修改代码] (vehicle_service)修改proto文件和头文件，增加转角和角速度信息
19. [修改代码] (sensor_gnss)修改proto文件和头文件，修正单位和频率的描述
20. [修改代码] (sensor_imu)修改proto文件和头文件，修正单位和频率的描述
21. [修改代码] (sensor_pbox_gnss)修改proto文件和头文件，修正单位和频率的描述
22. [修改代码] (sensor_pbox_imu)修改proto文件和头文件，修正单位和频率的描述
23. [修改代码] (common)修改proto文件和头文件，增加mcu数据计数标志、四元数、地图瓦片等公用结构体
24. [修改代码] (interface_converter)适配接口修改
25. [修改代码] (private)修改控制和规划的调试信息

####  已知问题  
暂无

####  兼容性  
暂无


# interface2.2.3

####  修改说明  
1. [新增代码] (ehr)增加地图适配接口，文档待整理
2. [修改代码] (fusion_road)修改proto文件和头文件，增加noa部分内容，文档待整理
3. [删除代码] (map)废弃此proto文件，文档待整理
4. [修改代码] (planning_hmi)修改proto文件，增加noa部分内容，文档待整理
5. [修改代码] (sensor_imu)修改proto文件，统一相机节点的数采接口和研测接口
6. [新增代码] (interface_converter)增加raw message结构体，文档待整理

####  已知问题  
1.文档待整理，接口需要重新评审

####  兼容性  
暂无


# interface2.2.1

####  修改说明  
1. [删除代码] (private planning)增加调试字段

####  已知问题  
暂无

####  兼容性  
暂无

# interface2.2.0

####  修改说明  
1. [删除代码] (hmi_soc_outer)修改proto文件和头文件，增加车道线起始点纵向距离，拆分acc和scc的状态结构体
2. [新增代码] (sensor_imu)修改proto文件和头文件，增加信号发出时间戳和头信息
3. [修改代码] (hmi_mcu_inner)修改proto文件和头文件，增加acc实际巡航车速和表显车速
4. [修改代码] (common)修改proto文件和头文件，增加数据源时间戳记录
5. [修改代码] (planning_debug_info)修改proto文件，增加统计数据流延时字段
6. [修改代码] (sensor_pbox_gnss)修改proto文件和头文件，修改PBOX_DATA_GNSS_QUALITY中枚举值，使其与设备原值保持一致
7. [修改代码] (fusion_objects)修改proto文件和头文件，新增融合障碍物速度角信息
8. [代码不统计] (fusion_road)增加一个topic输出简易车道线融合信息，降低负载，结构体不变
9. [修改代码] (vehicle_and_sensor_param)修改头文件，新增时间戳来源配置项
10. [修改代码] (sensor_image)修改proto文件，更新相机输出的时间名称
11. [修改代码] (planning_hmi)修改proto文件，增加cipv信息
12. [代码不统计] (interface_description)修改说明文档，mcu中算法的接口名称

####  已知问题  
暂无

####  兼容性  
暂无

# interface2.1.0

####  修改说明  
1. [删除代码] (planning_hmi)删除头文件，并从icc.h中去除
2. [新增代码] (lidar_objects)新增激光雷达数据结构，暂用于真值采集
3. [修改代码] (hmi_mcu_inner)修改proto文件和头文件，增加acc实际巡航车速和表显车速
4. [修改代码] (adas_function_debug)修改proto文件和头文件，增加调试字段
5. [修改代码] (vehicle_service)修改proto文件和头文件，增加温度字段
6. [新增代码] (pbox)新增pbox的gnss和imu的头文件和proto文件
7. [修改代码] (sensor_image)修改proto文件，增加周视定义字段，重构使其兼容实车算法以及数采需求
8. [修改代码] (private)增加算法模块debug信息proto文件供工具链使用
9. [新增代码] (common_vehicle_param)增加公共配制文件数据结构proto文件
10. [代码不统计] (interface_description)修改说明文档，增加pbox、common_vehicle_param、private的说明

####  已知问题  
暂无

####  兼容性  
暂无

# interface2.0.0.1

####  修改说明  
1. [新增代码] (planning_hmi)修改proto文件，为工控机的可视化提供变道提示消息
2. [代码不统计] (interface_description)修改说明文档和hmi_soc_outer的proto及头文件中的注释，纠正topic名称错误问题
3. [新增代码] (parking_slot_list)修改proto文件，增加限位器部分字段
4. [修改代码] (parking_slot_list)修改proto文件，限位器和车位角点位置单位统一为米；限位器和车位角点置信度范围备注修复为0-100，数据类型uint；删除限位器和车位角点数量字段，通过repeated判断
5. [修改代码] (lane_lines)修改proto文件，车道线方程的系数由float类型调整为double类型
6. [代码不统计] (interface_description)修改说明文档，road structure fusion输出所有topic的频率改为50Hz

####  已知问题  
暂无

####  兼容性  
暂无


# interface0.0.0.4

####  修改说明  
1. [修改代码] (所有模块)接口文档和proto合并统一，增加interface_description.md
2. [新增代码] (localization)增加片间通信头文件localization.h，透传定位信息到MCU
3. [修改代码] (uss_wave_info)修改proto及其片间通信头文件，修改apa障碍物数据大小12改为2，删除无用的数据项uss_dis
4. [修改代码] (uss_wave_debug_info)修改proto及其片间通信头文件，增加head头信息
5. [修改代码] (uss_percept_info)修改proto，修改超声波障碍物结构体名称
6. [修改代码] (uss_percept_debug_info)修改proto，修改多处参数名称
7. [修改代码] (fusion_objects)修改proto及其片间通信头文件，新增绝对坐标系有效性字段，跟踪状态使用枚举类型
8. [修改代码] (fusion_road)修改proto及其片间通信头文件，删除车道线列表，复用车道列表中的车道线信息
9. [修改代码] (parking_fusion)修改proto，新增限位器接口，增加车位对应的超声车位id信息
10. [新增代码] (vehicle_service)修改proto及其片间通信头文件，增加车速方向和光照传感器接口
11. [修改代码] (vehicle_service)修改片间通信头文件，修改驻车状态、轮速等数据结构，uint32改成uint8
12. [新增代码] (hmi_soc_inner)新增proto文件hmi_soc_inner.proto，用于临时接收工具组hmi消息，用于泊车

####  已知问题  
1. 此版本仅用于新架构第一阶段联调

####  兼容性  
暂无


# interface0.0.0.3

####  修改说明  
1. [修改代码] (proto)修改fusion_objects.proto，增加未融合的超声波雷达障碍物
2. [修改代码] (proto)修改fusion_road.proto及其片间通信头文件，增加EMU坐标下的输出
3. [修改代码] (proto)修改planning_plan.proto，增加规划成功的车位信息
4. [修改代码] (proto)修改sensor_gnss.proto，将gnss报文信息和移远设备诊断信息分成两个结构体

####  已知问题  
1. 此版本仅用于新架构第一阶段联调

####  兼容性  
暂无

# interface0.0.0.2

####  修改说明  
1. [修改代码] (proto)修改radar_perception_objects.proto，减少雷达数据结构内容，降低内存  
2. [修改代码] (mcu_base_interface)修改片间通信相关文件，减少融合车道线、融合障碍物和规划的片间通信数据量，降低内存  
3. [修改代码] (proto)修改sensor_gnss.proto，增加和header字段  
4. [修改代码] (proto)修改uss_wave_info.proto，调整部分数据结构和命名  

####  已知问题  
1. 此版本仅用于新架构第一阶段联调

####  兼容性  
暂无

# interface0.0.0.1

####  修改说明  
1. [新增代码] (interface_version)新增interface_version模块，用于各模块标记自己使用的接口
2. [新增代码] (mcu_base_interface)新增mcu_base_interface模块，包含所有MCU用到的头文件，片间通信放在mcu_base_interface/icc目录下，非片间通信放在mcu_base_interface目录下
3. [新增代码] (proto)新proto模块，包含所有SOC上需要用到的proto

####  已知问题  
1. 此版本仅用于新架构第一阶段联调

####  兼容性  
暂无

