#!/bin/bash

all_modules=(planning control prediction localization)

DEPLOY_DIR=/asw
SCRIPT_DIR=$(dirname "$0")

for module in ${all_modules[@]}; do
    PACKAGE_TAR=$SCRIPT_DIR/$module.tar
    DEPLOY_MODULE_DIR=$DEPLOY_DIR/$module
    ts=`date '+%s'`

    if [ ! -f $PACKAGE_TAR ]; then
        echo "-- $PACKAGE_TAR does not exist"
        continue
    fi

    if [ -d $DEPLOY_MODULE_DIR ]; then
        echo "-- backing up $DEPLOY_MODULE_DIR to $DEPLOY_MODULE_DIR.$ts ..."
        mv $DEPLOY_MODULE_DIR $DEPLOY_MODULE_DIR.$ts
    fi

    echo "-- deploying $DEPLOY_MODULE_DIR ..."

    tar xvf $PACKAGE_TAR -C $DEPLOY_DIR

    echo "-- deploy $DEPLOY_MODULE_DIR finished"
done
