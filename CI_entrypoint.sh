#!/bin/bash

# param from environment variables 
# VERSION_1="scc/develop_v2.3.4"
# VERSION_2="scc/develop_v2.3.3"
# CURRENT_TIME=$(date "+%Y%m%d_%H%M%S")
# IN_DIR="/data_cold/abu_zone/simulation_scenario/"

if [ "$VERSION_1" = "" ] || [ "$VERSION_2" = "" ] || [ "$CURRENT_TIME" = "" ] || [ "$IN_DIR" = "" ]; then
    echo "Environment variables VERSION_1, VERSION_2 and CURRENT_TIME and IN_DIR are required."
    exit
fi

# conda initialize -------------------------------
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/root/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/root/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/root/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/root/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup

# common param -------------------------------
PROJECT_PATH="/root"
# PROJECT_PATH="/docker_share"
# COMMON_TOOL_BRANCH="develop"
COMMON_TOOL_BRANCH="planning_tools_develop"
OUT_ROOT="/data_cold/abu_zone"
OUT_SUFFIX=".PP"

CONFIG_FILE="$PROJECT_PATH/planning/res/conf/module_configs/general_planner_module_highway.json"
OLD_TEXT="\"planner_type\": 0"
NEW_TEXT="\"planner_type\": 1"

VERSION_DIR_1=$(echo $VERSION_1 | tr '/' '_')
OUT_DIR_1="$OUT_ROOT/simulation_test_result/CI/$CURRENT_TIME/${VERSION_DIR_1}_${COMMIT_ID_1}"
mkdir -p $OUT_DIR_1

# run PP for VERSION_1 -------------------------------
cd $PROJECT_PATH/planning

git fetch
git checkout -b $VERSION_1/$CURRENT_TIME origin/$VERSION_1
if [ -n "$COMMIT_ID_1" ]; then
    git checkout $COMMIT_ID_1
fi
git submodule update
sed -i "s/$OLD_TEXT/$NEW_TEXT/g" $CONFIG_FILE
make clean
make pp_build BUILD_TYPE=Release

# translate string to arry
IFS=','
read -ra IN_DIR <<< "$IN_DIR"

index=0
for element in "${IN_DIR[@]}"
do
    ((index++))
    last_folder=$(basename "$element")
    OUT_DIR="${OUT_DIR_1}/${last_folder}_${index}"
    cd $PROJECT_PATH/planning
    python tools/planning_player/pp.py --dir $element --out_suffix $OUT_SUFFIX --out_dir $OUT_DIR --close_loop 1 --ignore_suffix 1
    cd $PROJECT_PATH/planning/jupyter_pybind/notebooks_scc/scripts/
    python html_generator.py plot_lat_plan_html $OUT_DIR
    python html_generator.py plot_lon_plan_html $OUT_DIR
done

# run checker for VERSION_1 -------------------------------
cd $PROJECT_PATH/common_tools/

git fetch
git checkout -b $CURRENT_TIME origin/$COMMON_TOOL_BRANCH

cd $PROJECT_PATH/common_tools/checker/task

index=0
for element in "${IN_DIR[@]}"
do
    ((index++))
    last_folder=$(basename "$element")
    OUT_DIR="${OUT_DIR_1}/${last_folder}_${index}"
    JSON_NAME=$(date "+%Y%m%d_%H%M%S")
    python prepare_CI_json.py --input_dir $OUT_DIR --out_dir $OUT_DIR --output_json $JSON_NAME.json
    python scc_checker_task.py $JSON_NAME.json
done

if [ -n "$VERSION_2" ]; then
    VERSION_DIR_2=$(echo $VERSION_2 | tr '/' '_')
    OUT_DIR_2="$OUT_ROOT/simulation_test_result/CI/$CURRENT_TIME/${VERSION_DIR_2}_${COMMIT_ID_2}"
    mkdir -p $OUT_DIR_2

    # run PP for VERSION_2 -------------------------------
    cd $PROJECT_PATH/planning

    git fetch
    git checkout -b $VERSION_2/$CURRENT_TIME origin/$VERSION_2
    if [ -n "$COMMIT_ID_2" ]; then
        git checkout $COMMIT_ID_2
    fi
    git submodule update
    sed -i "s/$OLD_TEXT/$NEW_TEXT/g" $CONFIG_FILE
    make clean
    make pp_build BUILD_TYPE=Release

    index=0
    for element in "${IN_DIR[@]}"
    do
        ((index++))
        last_folder=$(basename "$element")
        cd $PROJECT_PATH/planning
        OUT_DIR="${OUT_DIR_2}/${last_folder}_${index}"
        python tools/planning_player/pp.py --dir $element --out_suffix $OUT_SUFFIX --out_dir $OUT_DIR --close_loop 1 --ignore_suffix 1
        cd $PROJECT_PATH/planning/jupyter_pybind/notebooks_scc/scripts/
        python html_generator.py plot_lat_plan_html $OUT_DIR
        python html_generator.py plot_lon_plan_html $OUT_DIR
    done

    # run checker for VERSION_2 -------------------------------
    cd $PROJECT_PATH/common_tools/checker/task

    index=0
    for element in "${IN_DIR[@]}"
    do
        ((index++))
        last_folder=$(basename "$element")
        OUT_DIR="${OUT_DIR_2}/${last_folder}_${index}"
        JSON_NAME=$(date "+%Y%m%d_%H%M%S")
        python prepare_CI_json.py --input_dir $OUT_DIR --out_dir $OUT_DIR --output_json $JSON_NAME.json
        python scc_checker_task.py $JSON_NAME.json
    done
fi



