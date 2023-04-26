#!/bin/bash

# restart planning and control
bash /asw/Planning/scripts/stop_planning_and_control.sh&
sleep 2.0s
bash /asw/Planning/scripts/start_planning_and_control.sh&