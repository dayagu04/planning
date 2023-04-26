#!/bin/bash

# restart planing
bash /asw/Planning/scripts/stop_planning.sh&
sleep 2.0s
bash /asw/Planning/scripts/start_planning.sh&