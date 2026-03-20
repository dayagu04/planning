#!/bin/bash
#

source ./res/scripts/set_env.sh


mainboard -d /asw/planning/planning.dag &
mainboard -d /asw/control/control.dag &

# process: planning, control, hmi, virtual chassis, cyber_recorder
start_procs="viz2d_component_main virtual_chassis_component_main"


# SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

echo $SCRIPTS_PATH

export PROCESS_PATH=${SCRIPTS_PATH}/../../build
export PLANNING_PATH=${SCRIPTS_PATH}/../modules/planning

for proc in $start_procs; do
    #save $start_procs output to log, make terminal clean
    if [ "$proc"x = "planning_component_main"x ]; then
        # GLOG_v=4 ./$proc --flagfile=${PLANNING_PATH}/conf/planning.conf &
        # ./$proc --flagfile=${PLANNING_PATH}/conf/planning.conf &
        ${PROCESS_PATH}/$proc
        echo $proc
    elif [ "$proc"x = "viz2d_component_main"x ]; then
            ${PROCESS_PATH}/$proc &
        echo $proc
    elif [ "$proc"x = "virtual_chassis_component_main"x ]; then
            ${PROCESS_PATH}/$proc &
        echo $proc

    else
        ./$proc &
    fi
done




# Hit Ctrl+C to stop, Space to pause, or 's' to step.