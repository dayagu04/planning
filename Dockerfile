FROM artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/simulation_for_ci:V11

ARG CONFIG_FILE="/root/planning/res/conf/module_configs/general_planner_module_highway.json"
ARG OLD_TEXT='\"planner_type\": 0'
ARG NEW_TEXT='\"planner_type\": 1'

WORKDIR /root/planning

COPY . /root/planning/
COPY ./common_tools /root/common_tools

ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib:$LD_LIBRARY_PATH \
    CMAKE_PREFIX_PATH=/opt/ros/melodic:$CMAKE_PREFIX_PATH

RUN sed -i "s/$OLD_TEXT/$NEW_TEXT/g" $CONFIG_FILE && \
    /root/miniconda3/bin/pip install empy==3.3.4 boto3 && \
    echo 'source /opt/ros/melodic/setup.sh' >> ~/.bashrc && \
    mkdir -p /tmp && \
    make clean

RUN bash -c "source /opt/ros/melodic/setup.sh && make pp_build BUILD_TYPE=Release && rm -rf /root/planning/build/"

ENTRYPOINT ["/bin/bash", "/root/planning/CI_entrypoint_simulation.sh"]