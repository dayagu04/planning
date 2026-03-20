#!/bin/bash
#

source ./res/scripts/set_env.sh


# process: planning, control, hmi, virtual chassis, cyber_recorder
start_procs="viz2d_component_main"


# SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)


export PROCESS_PATH=${SCRIPTS_PATH}/../../build

${PROCESS_PATH}/$start_procs





# Hit Ctrl+C to stop, Space to pause, or 's' to step.