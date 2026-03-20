#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd $SCRIPT_DIR

MDS_NAME=MDC_Development_Studio-3.0.004-0000000-Ubuntu

# download MDS tool
if [ ! -d "$HOME/$MDS_NAME" ]; then
    curl -H "Authorization:Basic aGFvaGUxNDpBS0NwQnVuc0VQc1JZeTZHV1hBSE1SNjdtUWlScG5yRzFSUDMzQnFvNjl4NzJhY2JrQnZFRldtQ1d1VXliclZEb2RZY29yYzF5" -X GET -o "$MDS_NAME.tar" https://artifacts.iflytek.com/artifactory/AUTO-private-repo/ZNQC_ZDJS/Huawei_mdc510_repo/$MDS_NAME.tar
    tar xf $MDS_NAME.tar -C $HOME
    rm $MDS_NAME.tar
fi

# change system time for mds license
# echo "123456" | su -c "date --set=\"2024-10-01 14:30:00\""
# date -R

# generate code and configs using mds
echo calling $HOME/$MDS_NAME/generate.sh
rm -rf generated
rm -rf outputcfg
find $SCRIPT_DIR/arxml -type f -name "*.arxml" -exec sed -i 's|<TRANSPORT-PLUGIN>SHM|<TRANSPORT-PLUGIN>FSHM|g' {} +

bash $HOME/$MDS_NAME/generate.sh gen -i $SCRIPT_DIR/arxml -o $SCRIPT_DIR -r -s -q
if [ $? -ne 0 ]; then
    echo "generate failed"
    exit -1
fi

# patch .h
echo patching generated/includes/impl_type_uint8.h
cat <<EOF > generated/includes/impl_type_uint8.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_UINT8_H
#define IMPL_TYPE_UINT8_H

#include <cstdint>

using UInt8 = std::uint8_t;


#endif // IMPL_TYPE_UINT8_H
EOF

echo patching generated/includes/impl_type_uint16.h
cat <<EOF > generated/includes/impl_type_uint16.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_UINT16_H
#define IMPL_TYPE_UINT16_H

#include <cstdint>

using UInt16 = std::uint16_t;


#endif // IMPL_TYPE_UINT16_H
EOF

echo patching generated/includes/impl_type_uint32.h
cat <<EOF > generated/includes/impl_type_uint32.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_UINT32_H
#define IMPL_TYPE_UINT32_H

#include <cstdint>

using UInt32 = std::uint32_t;


#endif // IMPL_TYPE_UINT32_H
EOF

echo patching generated/includes/impl_type_uint64.h
cat <<EOF > generated/includes/impl_type_uint64.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_UINT64_H
#define IMPL_TYPE_UINT64_H

#include <cstdint>

using UInt64 = std::uint64_t;


#endif // IMPL_TYPE_UINT64_H
EOF

echo patching generated/includes/impl_type_int8.h
cat <<EOF > generated/includes/impl_type_int8.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_INT8_H
#define IMPL_TYPE_INT8_H

#include <cstdint>

using Int8 = std::int8_t;


#endif // IMPL_TYPE_INT8_H
EOF

echo patching generated/includes/impl_type_int16.h
cat <<EOF > generated/includes/impl_type_int16.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_INT16_H
#define IMPL_TYPE_INT16_H

#include <cstdint>

using Int16 = std::int16_t;


#endif // IMPL_TYPE_INT16_H
EOF

echo patching generated/includes/impl_type_int32.h
cat <<EOF > generated/includes/impl_type_int32.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_INT32_H
#define IMPL_TYPE_INT32_H

#include <cstdint>

using Int32 = std::int32_t;


#endif // IMPL_TYPE_INT32_H
EOF

echo patching generated/includes/impl_type_int64.h
cat <<EOF > generated/includes/impl_type_int64.h
/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2024. All rights reserved.
 */

#ifndef IMPL_TYPE_INT64_H
#define IMPL_TYPE_INT64_H

#include <cstdint>

using Int64 = std::int64_t;


#endif // IMPL_TYPE_INT64_H
EOF

#patch outputcfg scfi_mapping.yaml
echo patching scfi_mapping.yaml
find outputcfg -type f -name "scfi_mapping.yaml" -exec sed -i '4s/.*/common_swc_lib:/' {} +

#patch outputcfg large_buffer_qos.xml
echo patching large_buffer_qos.xml
find outputcfg -type f -name "large_buffer_qos.xml" -exec sed -i 's/409600/819200/g' {} \;
find outputcfg -type f -name "large_buffer_qos.xml" -exec sed -i 's/<depth>50<\/depth>/<depth>10<\/depth>/g' {} \;

#patch MANIFEST.json
echo patching MANIFEST.json
find outputcfg -type f -name "MANIFEST.json" -exec bash -c '
    dirname=$(dirname "$0")
    module_name=$(basename "$dirname")
    cluster=$(echo $(basename $(dirname $dirname)) | cut -c7 | tr "[:upper:]" "[:lower:]")
    process_name=$(awk '\''NR==14 {print $2}'\'' "$0" | tr -d \",)
    echo $process_name
    sed -i "5s/.*/      \"string\": \"$module_name\"/" "$0"
  
    sed -i "31,44d" "$0"

    # sed -i "31d" "$0"

    if [[ $module_name == around_view_camera_perception_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"enter_exit_timeout\",\n\
                \"value\": {\n\
                  \"object[]\": [\n\
                    {\n\
                      \"key\": \"enter_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 16.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    },\n\
                    {\n\
                      \"key\": \"exit_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 4.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    }\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                    \"Cpu.Processor0_Core0\",\n\
                    \"Cpu.Processor0_Core1\",\n\
                    \"Cpu.Processor0_Core2\",\n\
                    \"Cpu.Processor0_Core3\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_not_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              }" "$0"
    elif [[ $module_name == panorama_view_camera_perception_laneline_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"enter_exit_timeout\",\n\
                \"value\": {\n\
                  \"object[]\": [\n\
                    {\n\
                      \"key\": \"enter_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 60.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    },\n\
                    {\n\
                      \"key\": \"exit_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 4.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    }\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_not_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              }" "$0"
    elif [[ $module_name == panorama_view_camera_perception_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"enter_exit_timeout\",\n\
                \"value\": {\n\
                  \"object[]\": [\n\
                    {\n\
                      \"key\": \"enter_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 60.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    },\n\
                    {\n\
                      \"key\": \"exit_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 8.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    }\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_not_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              }" "$0"
    elif [[ $module_name == function_data_loop_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"enter_exit_timeout\",\n\
                \"value\": {\n\
                  \"object[]\": [\n\
                    {\n\
                      \"key\": \"enter_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 60.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    },\n\
                    {\n\
                      \"key\": \"exit_timeout_s\",\n\
                      \"value\": {\n\
                        \"double\": 8.0\n\
                      },\n\
                      \"checksum\": 444\n\
                    }\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_not_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                    \"Cpu.Processor0_Core3\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }" "$0"
    else
      sed -i "31i\\
              {\n\
                \"key\": \"shall_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"shall_not_run_ons\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                    \"Cpu.Processor0_Core3\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }" "$0"
    fi

    is_in_list() {
      local target="$1"
      shift  # 移除第一个参数，剩余参数为列表元素
      for item in "$@"; do
          if [[ "$item" == "$target" ]]; then
              return 0  # 存在，返回 true (0)
          fi
      done
      return 1  # 不存在，返回 false (1)
    }

    algo_lists=("around_view_camera_perception_exec" "panorama_view_camera_perception_laneline_exec" "panorama_view_camera_perception_exec" 
    "finite_state_manager_exec" "localization_exec" "obstacle_fusion_exec" "static_fusion_exec" "mega_exec" "ehr_exec" "ehp_exec" 
    "planning_exec" "control_exec" "calibration_exec" )

    if [[ $module_name == factory_calibration_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"function_group_states\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                    \"IflyadState.Factory\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    elif [[ $module_name == panorama_view_camera_perception_laneline_exec || $module_name == around_view_camera_perception_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"function_group_states\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                            \"IflyadState.Postsale\",\n\
                            \"IflyadState.WorkingNoCam\",\n\
                            \"IflyadState.Working\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    elif [[ $module_name == calibration_exec || $module_name == localization_exec || $module_name == obstacle_fusion_exec || $module_name == panorama_view_camera_perception_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"function_group_states\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                            \"IflyadState.Factory\",\n\
                            \"IflyadState.Postsale\",\n\
                            \"IflyadState.WorkingNoCam\",\n\
                            \"IflyadState.Working\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    elif [[ $module_name == function_data_loop_exec ]]; then
      sed -i "31i\\
              {\n\
                \"key\": \"function_group_states\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                            \"IflyadState.Startup\",\n\
                            \"IflyadState.Factory\",\n\
                            \"IflyadState.Postsale\",\n\
                            \"IflyadState.WorkingNoCam\",\n\
                            \"IflyadState.Working\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    elif is_in_list "$module_name" "${algo_lists[@]}"; then
      sed -i "31i\\
              {\n\
                \"key\": \"function_group_states\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                            \"IflyadState.Factory\",\n\
                            \"IflyadState.WorkingNoCam\",\n\
                            \"IflyadState.Working\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    else
      sed -i "31i\\
              {\n\
                \"key\": \"function_group_states\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                            \"IflyadState.Startup\",\n\
                            \"IflyadState.Factory\",\n\
                            \"IflyadState.WorkingNoCam\",\n\
                            \"IflyadState.Working\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    fi

    if [[ $module_name == planning_exec ]]; then
        sed -i "31i\\
              {\n\
                \"key\": \"machine_states\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"environments\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                    \"VIZ_ADDRESS=192.168.1.102 7000\",\n\
                    \"CM_LOG_TYPE=SYS_LOG\",\n\
                    \"CM_LOG_LEVEL=LOG_INFO\",\n\
                    \"PATH=/usr/bin/:/usr/sbin/:/usr/bin/busybox/:\${PATH}\",\n\
                    \"LD_LIBRARY_PATH=/opt/usr/app/1/gea/lib/:/opt/usr/app/1/gea/runtime_service/$module_name/lib/:\${LD_LIBRARY_PATH}\",\n\
                    \"CM_CONFIG_FILE_PATH=/opt/usr/app/1/gea/runtime_service/$module_name/etc/$process_name\",\n\
                    \"IFLYAD_LOG_CONFIG_PATH=/opt/usr/app/1/gea/conf/iflyadlog.conf\",\n\
                    \"SET_IFLY_LOG_LEVEL=5\",\n\
                    \"ENABLE_MDC_LOG_CONSOLE=ON\",\n\
                    \"SET_MDC_AP_LOG_LEVEL=FATAL\",\n\
                    \"EM_INNER_APP_USER=root\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    elif [[ $module_name == function_data_loop_exec ]]; then
        sed -i "31i\\
              {\n\
                \"key\": \"machine_states\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"environments\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                    \"VIZ_ADDRESS=192.168.1.102 7000\",\n\
                    \"CM_LOG_TYPE=SYS_LOG\",\n\
                    \"CM_LOG_LEVEL=LOG_INFO\",\n\
                    \"PATH=/usr/bin/:/usr/sbin/:/usr/bin/busybox/:\${PATH}\",\n\
                    \"LD_LIBRARY_PATH=/opt/usr/app/1/gea/lib/:/opt/usr/app/1/gea/runtime_service/$module_name/lib/:\${LD_LIBRARY_PATH}\",\n\
                    \"CM_CONFIG_FILE_PATH=/opt/usr/app/1/gea/runtime_service/$module_name/etc/$process_name\",\n\
                    \"IFLYAD_LOG_CONFIG_PATH=/opt/usr/app/1/gea/conf/iflyadlog.conf\",\n\
                    \"SET_IFLY_LOG_LEVEL=5\",\n\
                    \"ENABLE_MDC_LOG_CONSOLE=ON\",\n\
                    \"SET_MDC_AP_LOG_LEVEL=FATAL\",\n\
                    \"SOMEIP_CONFIG_FILE=/opt/usr/app/1/entity_gea/runtime_service/$module_name/conf/rds_service_someip.json\",\n\
                    \"SOMEIP_APP_NAME=rds_service_someip\",\n\
                    \"EM_INNER_APP_USER=root\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    else
        sed -i "31i\\
              {\n\
                \"key\": \"machine_states\",\n\
                \"value\": {\n\
                  \"string[]\": []\n\
                },\n\
                \"checksum\": 333\n\
              },\n\
              {\n\
                \"key\": \"environments\",\n\
                \"value\": {\n\
                  \"string[]\": [\n\
                    \"VIZ_ADDRESS=192.168.1.102 7000\",\n\
                    \"CM_LOG_TYPE=SYS_LOG\",\n\
                    \"CM_LOG_LEVEL=LOG_INFO\",\n\
                    \"PATH=/usr/bin/:/usr/sbin/:/usr/bin/busybox/:\${PATH}\",\n\
                    \"LD_LIBRARY_PATH=/opt/usr/app/1/gea/lib/:/opt/usr/app/1/gea/runtime_service/$module_name/lib/:\${LD_LIBRARY_PATH}\",\n\
                    \"CM_CONFIG_FILE_PATH=/opt/usr/app/1/gea/runtime_service/$module_name/etc/$process_name\",\n\
                    \"IFLYAD_LOG_CONFIG_PATH=/opt/usr/app/1/gea/conf/iflyadlog.conf\",\n\
                    \"SET_IFLY_LOG_LEVEL=5\",\n\
                    \"ENABLE_MDC_LOG_CONSOLE=ON\",\n\
                    \"SET_MDC_AP_LOG_LEVEL=FATAL\",\n\
                    \"EM_INNER_APP_USER=root\"\n\
                  ]\n\
                },\n\
                \"checksum\": 333\n\
              }," "$0"
    fi

' {} \;

# restart_attempt_num 设置重启次数为3次
# find outputcfg -type f -name "MANIFEST.json" -exec bash -c 'sed -i "/\"key\": \"restart_attempt_num\"/{N;N;s/\"uint32\": 0/\"uint32\": 3/}" "$0"' {} \;

sed -i "s|panorama_view_camera_perception_laneline_exec/lib|panorama_view_camera_perception_laneline_exec/res/perception_engine/Release/SDK/lib|" outputcfg/MdcSocAMachine/panorama_view_camera_perception_laneline_exec/MANIFEST.json
sed -i "s|panorama_view_camera_perception_exec/lib|panorama_view_camera_perception_exec/res/perception_engine/Release/SDK/lib|" outputcfg/MdcSocBMachine/panorama_view_camera_perception_exec/MANIFEST.json

find outputcfg -type f -name "default_qos.xml" -exec sed -i 's|<FragSize>409600</FragSize>|<FragSize>102400</FragSize>|g' {} +
find outputcfg -type f -name "small_buffer_qos.xml" -exec sed -i 's|<FragSize>409600</FragSize>|<FragSize>20480</FragSize>|g' {} +
find outputcfg -type f -name "small_buffer_qos.xml" -exec sed -i 's|<depth>3</depth>|<depth>2</depth>|g' {} +
find outputcfg -type f -name "*.xml" -exec sed -i 's|<depth>10</depth>|<depth>3</depth>|g' {} +
find outputcfg -type f -name "*.xml" -exec sed -i 's|<ListSize>20|<ListSize>10|g' {} +

# reset system time
# echo "123456" | su -c "ntpdate cn.pool.ntp.org"
# date -R
