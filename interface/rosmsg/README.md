# rosmsg
数据采集统一接口定义
sensor_interface: 
包含毫米波/Lidar/GNSS/IMU等传感器输出定义

vehicle_msgs: 
车辆输出接口定义

struct_msgs: 
算法相关消息定义(C结构体)

proto_msgs:
算法相关消息定义(protobuf)

# 使用方法：
在ros环境中创建文件夹dev_ws/src
cd dev_ws/src
git clone ssh://git@code.iflytek.com:30004/ZNQC_AutonomousDriving/interface.git -b xxx（指定分支）
cd dev_ws/
catkin_make
source ./devel/setup.zsh或者source ./devel/setup.sh

后可以在ros环境中使用ros相关命令
