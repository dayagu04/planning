#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
cd $SCRIPT_DIR/..

make build BUILD_TYPE=Release PLATFORM=BZT NUM_JOB=4

cp target/README.md install/planning/
cp CHANGELOG.md install/planning/
