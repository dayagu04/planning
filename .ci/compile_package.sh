#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
SRC_DIR=$SCRIPT_DIR/..

cd $SRC_DIR

mkdir build
cd build
cmake ..
make
make install
cd ..

rm target/Planning.tar
cd install
tar -cvf ../target/Planning.tar planning
ls -lh ../target
