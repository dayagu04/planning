# planning.2.0.0.6

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

