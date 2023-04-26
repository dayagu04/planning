#!/bin/bash

# stop planning and apa simulation
apa_simulation_and_planning_pid=`ps -ef | grep apa_simulation_and_planning.dag | grep -v grep | awk '{print $2}'`
if [ -n "$apa_simulation_and_planning_pid" ]
then
  kill -9 $apa_simulation_and_planning_pid
fi