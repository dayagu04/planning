#!/bin/bash

PWD=$(pwd)
echo $PWD

# source env related
LD_LIBRARY_PATH+=":$PWD"

LIB_PATH="$PWD/src/thirdparty/osqp.out/out"
if [ -d "$LIB_PATH" ]; then
  LD_LIBRARY_PATH+=":$PWD/src/thirdparty/osqp.out/out"
fi

LIB_PATH="/root/miniconda3/lib"
if [ -d "$LIB_PATH" ]; then
  LD_LIBRARY_PATH+=":/root/miniconda3/lib"
fi

echo $LD_LIBRARY_PATH
export LD_LIBRARY_PATH

SCRIPTS_PATH=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# export CYBER_DOMAIN_ID=80
# export CYBER_IP=127.0.0.1

# log path related
# export CYBER_TIME=`date +"%Y%m%d_%H%M%S"`
export CYBER_TIME=`date +"%Y%m%d_%H"`
echo $CYBER_TIME

export IFLY_GLOG_PATH="/asw/planning/glog"

cd ${SCRIPTS_PATH}

echo $SCRIPTS_PATH


if [ -d "$IFLY_GLOG_PATH" ]; then
  ### 如果 $DIR 存在，请执行操作 ###
  echo "log directory in ${IFLY_GLOG_PATH}..."
else
  mkdir ${IFLY_GLOG_PATH}
  exit 1
fi

cd ${IFLY_GLOG_PATH}

mkdir $CYBER_TIME

cd ${SCRIPTS_PATH}

# export GLOG_log_dir=${IFLY_GLOG_PATH}/${CYBER_TIME}
export GLOG_log_dir=${IFLY_GLOG_PATH}

echo $GLOG_log_dir

# log content related
export GLOG_alsologtstderr=1
export GLOG_logtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

# for DEBUG log
#export GLOG_minloglevel=-1
# export GLOG_v=5
