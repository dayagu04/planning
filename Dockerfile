FROM artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/simulation_for_ci:V11

WORKDIR /root/planning

ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib:$LD_LIBRARY_PATH \
    CMAKE_PREFIX_PATH=/opt/ros/melodic:$CMAKE_PREFIX_PATH

RUN /root/miniconda3/bin/pip install empy==3.3.4 shapely boto3 && \
    echo 'source /opt/ros/melodic/setup.sh' >> ~/.bashrc && \
    mkdir -p /tmp

COPY ./CMakeLists.txt ./Makefile /root/planning/
COPY ./.ci /root/planning/.ci
COPY ./thirdparty /root/planning/thirdparty/
RUN bash -c "make clean && make thirdparty_build BUILD_TYPE=Release"
COPY ./interface /root/planning/interface/
RUN bash -c "source /opt/ros/melodic/setup.sh && make interface_build BUILD_TYPE=Release"
COPY ./ad_common /root/planning/ad_common/
RUN bash -c "source /opt/ros/melodic/setup.sh && make ad_build BUILD_TYPE=Release"

COPY . /root/planning/
COPY ./common_tools /root/common_tools
RUN bash -c "source /opt/ros/melodic/setup.sh && make pp_build NUM_JOB=32 BUILD_TYPE=Release && rm -rf /root/planning/build/"

ENTRYPOINT ["/bin/bash", "/root/planning/CI_entrypoint_simulation.sh"]