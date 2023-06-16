#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
cd $SCRIPT_DIR/..

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
make install
cd ..

cp target/README.md install/planning/
cp CHANGELOG.md install/planning/
