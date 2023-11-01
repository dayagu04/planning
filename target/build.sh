#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
PRODUCT_NAME=JAC_S811
if [[ $# -ge 1 ]]; then
    PRODUCT_NAME=$1
fi

cd $SCRIPT_DIR/..

make build BUILD_TYPE=Release PLATFORM=BZT PRODUCT=$PRODUCT_NAME NUM_JOB=4

cp target/README.md install/planning/
cp CHANGELOG.md install/planning/

strings install/planning/Lib/libplanning_component.so | grep auto_version > install/planning/VERSION

make package
