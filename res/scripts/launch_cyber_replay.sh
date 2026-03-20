#!/bin/bash
#

source ./res/scripts/set_env.sh



# run replay file
cyber_recorder play -f /docker_share/astar_0627_2/test_4.00000 \
# cyber_recorder play -f /docker_share/sim/test_3.00000 \
    -c /iflytek/fusion/road_fusion \
    -c /iflytek/localization/ego_pose \
    -c /iflytek/camera_perception/ground_line \
    -c /iflytek/fusion/ground_line \
    -c /iflytek/vehicle_service \
    -c /iflytek/planning/debug_info \
    -c /iflytek/planning/plan \
    -c /iflytek/planning/hmi \
    -c /iflytek/adas_function_debug \
    -c /iflytek/prediction/prediction_result \
    -c /iflytek/fusion/objects \
    -c /iflytek/system_state/soc_state \
    -c /iflytek/control/control_command \
    -c /iflytek/control/debug_info \
    -c /iflytek/hmi/mcu_inner \
    -c /iflytek/hmi/soc_outer \
    -c /iflytek/fusion/parking_slot \
    -c /iflytek/uss/wave_info \
    -c /iflytek/UssPerceptInfo
    # -s 61
    # -b 2023-12-05-15:01:50


# Hit Ctrl+C to stop, Space to pause, or 's' to step.