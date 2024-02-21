#!/bin/bash

if [ $# -ne 2 ]; then
    echo "usage: record.sh [output path] [bag prefix]"
    echo "example: record.sh /user_data long_time"
    exit -1 
fi

output_dir=`realpath $1`
bag_prefix=$2

bag_index=0
index_file=$output_dir/.record_index
if [ -f $index_file ]; then
    bag_index=`cat $index_file`
    bag_index=`expr $bag_index + 1`
fi
echo $bag_index > $index_file

bag_file=$output_dir/${bag_prefix}_${bag_index}
echo "recording to $bag_file ..."

cyber_recorder record -c \
    /iflytek/fusion/road_fusion \
    /iflytek/localization/ego_pose \
    /iflytek/vehicle_service \
    /iflytek/planning/debug_info \
    /iflytek/planning/plan \
    /iflytek/planning/hmi \
    /iflytek/adas_function_debug \
    /iflytek/prediction/prediction_result \
    /iflytek/fusion/objects \
    /iflytek/system_state/soc_state \
    /iflytek/control/control_command \
    /iflytek/control/debug_info \
    /iflytek/hmi/mcu_inner \
    /iflytek/hmi/soc_outer \
    /iflytek/sensor/gnss \
    /iflytek/sensor/imu \
    /iflytek/camera_perception/lane_lines \
    /iflytek/fusion/parking_slot \
    /mobileye/camera_perception/lane_lines \
    /mobileye/camera_perception/objects \
    -i 3600 \
    -m 4096 \
    -o $bag_file
