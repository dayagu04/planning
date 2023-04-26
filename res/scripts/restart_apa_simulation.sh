#!/bin/bash

# restart apa simulation
bash /asw/Planning/scripts/stop_apa_simulation.sh&
sleep 2.0s
bash /asw/Planning/scripts/start_apa_simulation.sh&