#!/bin/bash

# stop planning
planning_pid=`ps -ef | grep planning.dag | grep -v grep | awk '{print $2}'`
if [ -n "$planning_pid" ]
then
  kill -9 $planning_pid
fi