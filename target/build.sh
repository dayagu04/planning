#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
PRODUCT_NAME=JAC_S811
TYPE=SYSTEM
if [[ $# -ge 1 ]]; then
    PRODUCT_NAME=$1
fi

if [[ $# -ge 2 ]]; then
    TYPE=$2
fi

if [[ $TYPE == "SYSTEM" ]]; then
    echo "SYSTEM BUILD!"
    rm -rf install/planning
    cd $SCRIPT_DIR/..

    make build BUILD_TYPE=Release PLATFORM=BZT PRODUCT=$PRODUCT_NAME NUM_JOB=8

    cp target/README.md install/planning/
    cp CHANGELOG.md install/planning/

    strings install/planning/Lib/libplanning_component.so | grep auto_version > install/planning/VERSION

    make package
else
    echo "LOCAL BUILD!"
    rm -rf install/planning
    make build BUILD_TYPE=Release PLATFORM=BZT PRODUCT=$PRODUCT_NAME NUM_JOB=16

    rm ./planning.7z
    7z a ./planning.7z ./install/planning

    commit_hash=$(git log --pretty=format:"%h" -n 1)
    echo $commit_hash
    mv ./planning.7z ./planning-$commit_hash-$PRODUCT_NAME.7z
fi
