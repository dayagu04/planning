#!/bin/bash
num_args=$#
if [ $num_args -ne 4 ]; then
    echo "使用方法: $0 GIT_BRANCH COMMIT_ID START_TIME"
    exit 1
fi

branch_name=$1
commit=$2
# 将毫秒时间戳转换为秒
timestamp=$(($3 / 1000 ))
builder=$4
# 将秒时间戳转换为指定格式
sim_time=$(date -d @$timestamp +"%Y%m%d_%H%M%S")

echo "1:" $branch_name "2:" $commit "3:" $sim_time "4:" $builder 

name=$(sed -n '1p' /disk1/build_conf/sim_ci_key.txt)
key=$(sed -n '2p' /disk1/build_conf/sim_ci_key.txt)

# 检查命令是否成功执行
if curl -H "X-Api-Key:$key" https://artifacts.iflytek.com:443/artifactory/auto-docker-product-public/autofpilotdevtools/simulation/planning_simulation/ | grep -q "$commit"; then
    echo "镜像 $commit 已经存在"
    rm -rf output.html
    exit 0
fi

#登录docker 容器
docker login artifacts.iflytek.com/ -u $name -p $key

#拉取最新的镜像
docker pull artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/simulation_for_ci:latest

#运行打镜像命令
docker build --build-arg VERSION=$branch_name --build-arg COMMIT=$commit -t planning_simulation:$commit -f simulation/Dockerfile .

#如果上述命令运行成功，则用以下命令将镜像推送到镜像仓库；如果不成功，展示报错：
docker tag planning_simulation:$commit artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/planning_simulation:$commit

#推送镜像
docker push artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/planning_simulation:$commit

docker logout artifacts.iflytek.com

#删除本地临时镜像
docker rmi planning_simulation:$commit
docker rmi artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/planning_simulation:$commit