#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
SRC_DIR=$SCRIPT_DIR/..

cd $SRC_DIR

sh target/build.sh
