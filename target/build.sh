#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
cd $SCRIPT_DIR/..

mkdir build
cd build
cmake ..
make -j && make install