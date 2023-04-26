#!/bin/bash

# stop planning and control
planning_and_control_pid=`ps -ef | grep planning_and_control.dag | grep -v grep | awk '{print $2}'`
if [ -n "$planning_and_control_pid" ]
then
  kill -9 $planning_and_control_pid
fi