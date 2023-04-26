#!/bin/bash

# restart apa simulation and planning
bash /asw/Planning/scripts/stop_apa_simulation_and_planning.sh&
sleep 2.0s
bash /asw/Planning/scripts/start_apa_simulation_and_planning.sh&