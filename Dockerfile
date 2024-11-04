FROM artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/simulation_for_ci:V11

WORKDIR /root/planning

ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib:$LD_LIBRARY_PATH \
    CMAKE_PREFIX_PATH=/opt/ros/melodic:$CMAKE_PREFIX_PATH

RUN /root/miniconda3/bin/pip install empy==3.3.4 shapely boto3 && \
    echo 'source /opt/ros/melodic/setup.sh' >> ~/.bashrc && \
    mkdir -p /tmp

COPY . /root/planning/
COPY ./common_tools /root/common_tools

RUN bash -c "source /opt/ros/melodic/setup.sh && make clean && make pp_build BUILD_TYPE=Release && rm -rf /root/planning/build/"

ENTRYPOINT ["/bin/bash", "/root/planning/CI_entrypoint_simulation.sh"]