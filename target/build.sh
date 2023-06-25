#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
cd $SCRIPT_DIR/..

make build BUILD_TYPE=Release PLATFORM=BZT

cp target/README.md install/planning/
cp CHANGELOG.md install/planning/
