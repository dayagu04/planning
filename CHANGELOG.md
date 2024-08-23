# planning.2.5.0
####  修改说明
1. 使用新定位，弃用老定位接口:/iflytek/localization/ego_pose
2. 增加hmi信息: 收费站距离、道路类型、信息; 实线抑制、手动取消信息
3. Track ego lane consider of Large curvature scene
4. APA适配新状态机 & 新定位
5. 信号灯通行的基础功能: 红灯停绿灯行（默认关闭）
6. 移除中间件adapter层
7. 加入计算merge区域
### [fix]
1. fix latent coredump
2. fix heading_diff not normalize in SelectEgoLaneWithoutPlan
3. fix bug for calculate dis to last merge point

# planning.2.4.9
####  修改说明
1. Add new HMI/FSM
2. 适配感知220m特性
3. 更新系统状态机topic，不再接收hmi topic
4. 修复acc状态先ego lane选取
5. 加入锥桶换道功能
6. 启用高架NOA
7. 修复lead_one_change引起二次起步
8. 放开曲率限速加速度限制，cut in判断不使用lat offset

# planning.2.4.8
####  修改说明
1. Add planning InputHistoryInfo of Header
2. 限制acc bound下限；修改远距离慢车场景中的JLT触发频率
3. Regard virtual line type as dashed line type when lc
4. 增加planning IO topic 故障码
5. 优化横向path，提升横向舒适度，以及匝道内的收敛性
6. 调整大车的避让buffer
7. 增加agent_longitudinal_decider，优化cut in
8. Add lane line dashed and solid judgment for ilc
9. 使用前后帧车道重合率代替轨迹与车道计算cost
10. 添加舒适性减速的感知距离需求

# planning.2.4.7
####  修改说明
1. 变道结束前1s使用avoid offset
2. 打开起步模式下低速重规划（仅起步模式生效）
3. APA decide static flag from local pose and vel
4. 新增静态障碍物避让
5. 新增静态障碍物预减速
6. 新增道路边缘bound
7. 新增障碍物bound(关闭状态)
8. 场景抑制主动变道
9. 新增连续匝道的判断，防止自车在连续匝道场景，先加速后减速的问题
10. lane change wait状态下，gap不作为bound引入

# planning.2.4.6
####  修改说明
1. 完善导航地图变道
2. 更新replan方案
3. Track ego lane，优化分流口选取
4. 纵向s bound调整，启用lead two
5. 调整NOA匝道限速
6. 泊车通用障碍物适配
7. 垂直泊车实时碰撞检测

# planning.2.4.4
####  修改说明
1. 长时方案上线，已完成0630演示目标
2. 行泊车共分支,c结构体2.4.3接口

# planning.2.3.3.2
####  修改说明
1. 更新planning框架
2. 默认发版为实时方案，长时继续自测迭代
3. 暂不启用交通障碍物

# planning.2.3.3.1
####  修改说明
1. 变更SCC默认配置为SCC_PLANNER轨迹方案
2. 调整jerk weight
3. interface接口同步为 interface2.3.5
4. 添加退自动请求
5. 修复长时s bound, lead属性清除等

# planning.2.3.2.3
####  修改说明
1. 更新interface接口至2.3.5版本
2. 对应系统版本为2.3.4

# planning.2.3.2.2
####  修改说明
1. 修复在hpp场景的U型弯，规划起点在ref上的投影出错导致决策规划的ref错误的问题
2. 修复轨迹拼接时planning_loop_dt默认取0.1的问题
3. add lat theta replan
4. 更新可视化工具,加入自车包络框的planning轨迹
5. 优化代码耗时在virtual_lane_manager计算超视距信息部分

# planning.2.3.2.1
####  修改说明
1. 修改长时重规划的参数设置
2. 修复因交互式变道获取车道线type引起的的崩溃问题
3. 区分hpp和highway场景
4. 适配新定位接口
5. local_view接入parking_map与groundline

# planning.2.3.1.1
####  修改说明
1. 合入0115奇瑞演示变更项
2. 合入长时planning，默认实时

# planning.2.3.1.0
####  修改说明
1. 适配2.3接口

# planning.2.3.0.1
####  修改说明
1. LDP功能接入实车控制

# planning.2.2.1.7
####  修改说明
1. APA/SCC共分支
2. SCC针对交付需求优化以及APA新架构与交付优化
3. 0115奇瑞验收

# planning.2.2.1.6
####  修改说明
1. 修改fusion source，纵向使用BEV全方位障碍物
2. 解决max_lat_offset不生效
3. 优化taskpipeline，不析构
4. scc planner合入，默认不开启

# planning.2.2.1.5
####  修改说明
1. 调整换道速度
2. SCC/ACC无车道线情况下构建虚拟车道

# planning.2.2.1.4
####  修改说明
1. realtime planner输出轨迹化适配noa, 默认不开启

# planning.2.2.1.3
####  修改说明
1. NOA合入develop分支
2. interface2.2.3

# planning.2.2.1.2
####  修改说明
1. 关闭奇瑞主动换道功能

# planning.2.2.1.1
####  修改说明
1. 修复reference_path & target_tracks nullptr引起的coredump

# planning.2.2.1.0
####  修改说明
1. APA新架构合入develop分支

# planning.2.2.0.3_noa
####  修改说明
1. NOA修改
2. 增加uncompress topic结构体输出，降低片间通负载

# planning.2.2.0.4
####  修改说明
1. 增加realtime planner，横纵向均接入轨迹化处理
2. 优化纵向跟车与跟停效果

# planning.2.2.0.3
####  修改说明
1. 修复快速调整巡航车速导致的急加速、急减速现象
2. 适配acc模式

# planning.2.2.0.2
####  修改说明
1. 修复变道过程车道线消失的崩溃问题
2. 适配融合障碍物unknown的使用，避免障碍物误筛

# planning.2.2.0.1
####  修改说明
1. 适配interface2.2.0
2. 增加APA专用可视化工具
3. 增加数据延时统计信息到header
4. hmi添加cipv信息

# planning.2.1.0.4

####  修改说明
1. Release版本默认打日志到/dev/null
2. 优化leadone&cutin障碍物筛选逻辑，减少对cutin障碍物制动过晚、制动过重问题

# planning.2.1.0.3

####  修改说明
1. 优化判定障碍物起步逻辑，减少二次起步问题
2. 修复apa选无效车位状态机跳转异常问题

# planning.2.1.0.2

1.适配HMI输出CIPV信息
2.适配系统状态机功能接口
3.初步修复障碍物横向速度计算的bug
4.优化障碍物车道分配逻辑
5.修复变道贴线抖动问题
6.修复实线变道问题
7.apa planning使用统一车辆参数
8.表显车速修改为表显对应的实际车速
9.bugfix of apa state machine in apa searching
10.优化障碍物筛选逻辑，减少主动变道误触发

# planning.2.1.0.1

####  修改说明
1. 平滑变道返回fix_lane设置
2. 修复不合理清空车道分配，导致参考线跳变问题
3. 修复接管后主动变道筛选障碍物不清空，导致转向灯请求不取消问题
4. 优化可变道返回判定逻辑，初步考虑执行器响应特性
5. 结合S811响应特性，优化状态跳转逻辑&fix_lane更新逻辑，减少变道执行过程中参考线的频繁变换
6. 屏蔽对环视范围内检测的横向近距障碍物的大幅避让功能
7. 修复障碍物类型定义使用错误导致的纵向cutin判断失效的问题
8. 车辆静止时，更新standstill以及启停状态供系统状态机使用
9. 添加启停状态判断UpdateStartStopState，在纵向跟停误差1m内，可以防止二次起步
10. 适配interface2.1.0
11. 修复apa碰撞检测buffer方向给反问题


# planning.2.0.0.8

####  修改说明
1. 修复不合理清空local_view数据，导致自车晃动问题
2. 修复实时规划不合理使用预测信息的bug
3. 输出header携带版本号，输出topic时间戳保持一致
4. planning loop rate写入gflags
5. 修复接管状态不合理产生主动变道请求bug
6. 修复避让车计时单位使用错误导致避让车信息维持不住，造成fix_lane抖动问题
7. 暂时关闭避让回复过程中横向期望位置受车道线曲率影响，减少fix_lane抖动
8. 可视化工具链支持html生成
9. 修复candidate_states未正确赋值，导致崩溃的问题
10. 上游障碍物管理模块统一实时长时规划障碍物筛选逻辑，仅筛除不可信部分

# planning.2.0.0.6/planning.2.0.0.7

####  修改说明
1. 长时纵向规划采用s_lead进行CIPV跟车
2. 更新apa可视化脚本
3. 更新车辆参数配置文件

# planning.2.0.0.5

####  修改说明
1. 适配interface2.0.0.1
2. map lane bound protect: clip for short lane distance
3. 修复state_machine_output未加&，导致下游task获取当前状态信息错误的bug
4. fixed avd radar obj bug & avd_car_past_ keep bug
5. 修复当前车道index的bug

# planning.2.0.0.3

####  修改说明
1. 增加产物中版本号auto_version
2. longtime,realtime初步可跑
3. 代码格式化，集成通用编译脚本

####  已知问题
无

####  兼容性
无

# planning.2.0.0.2

####  修改说明
1. 适配interface0.0.0.4 ed1d77e6fd6bf7f73a5b12898627b782b47b641d
2. 增加adas功能实现
3. 添加换道信息hmi输出
4. 修复长时lateral_obstacle参考线拿不到问题，修复target_select取障碍物错误的问题

####  已知问题
无

####  兼容性
无

# planning.2.0.0.1
####  修改说明
1. 初始版本，支持行车规划

####  已知问题
无

####  兼容性
无

