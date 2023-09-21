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

