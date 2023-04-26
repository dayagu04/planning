#!/bin/bash

# stop apa simulation
apa_simulation_pid=`ps -ef | grep apa_simulation.dag | grep -v grep | awk '{print $2}'`
if [ -n "$apa_simulation_pid" ]
then
  kill -9 $apa_simulation_pid
fi