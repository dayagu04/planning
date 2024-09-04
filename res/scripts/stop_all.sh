#!/bin/bash
#

ended_procs="viz2d_component_main
            virtual_chassis_component_main
            cyber_recorder
            cyber_monitor
            planning.dag
            control.dag
            planning_and_control.dag
            "

curr_dir=$(pwd)

for proc in $ended_procs; do
    echo "kill $proc"
    proc_trunc=${proc:0:14}
    ps_proc=`ps -ef | grep ${proc} | grep -v grep | awk '{print $2}'`

    if [ -n "$ps_proc" ]
    then
        kill -9 $ps_proc
    fi
done


