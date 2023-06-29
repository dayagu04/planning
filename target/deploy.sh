#!/bin/bash

MODULE_NAME=planning

SCRIPT_DIR=$(dirname "$0")
PACKAGE_TAR=$SCRIPT_DIR/$MODULE_NAME.tar
DEPLOY_DIR=/asw
DEPLOY_MODULE_DIR=$DEPLOY_DIR/$MODULE_NAME
ts=`date '+%s'`

if [ ! -f $PACKAGE_TAR ]; then
    echo "$PACKAGE_TAR does not exist"
    exit -1
fi

if [ -d $DEPLOY_MODULE_DIR ]; then
    echo "moving $DEPLOY_MODULE_DIR to $DEPLOY_MODULE_DIR.$ts"
    mv $DEPLOY_MODULE_DIR $DEPLOY_MODULE_DIR.$ts
fi

tar xvf $PACKAGE_TAR -C $DEPLOY_DIR

echo "deploy $DEPLOY_MODULE_DIR finished"
