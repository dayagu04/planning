FROM artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/simulation/simulation_for_ci:latest
ARG VERSION
ARG COMMIT
ARG CONFIG_FILE="/root/planning/res/conf/module_configs/general_planner_module_highway.json"
ARG OLD_TEXT='\"planner_type\": 0'
ARG NEW_TEXT='\"planner_type\": 1'
WORKDIR /root/planning
RUN git fetch 

RUN if [ -n "$VERSION" ]; then \
        git checkout -b simulation/$VERSION origin/$VERSION || (echo "Error, VERSION is not exist, exiting the build process..." && exit 1) \
    else \
        echo "Error, Parameter VERSION is empty, exiting the build process..." && exit 1;\
    fi

RUN if [ -n "$COMMIT" ]; then \
        git checkout $COMMIT || (echo "Error, COMMIT is not exist, exiting the build process..." && exit 1) \
    else \
        echo "Error, Parameter COMMIT is empty, exiting the build process..." && exit 1; \
    fi

RUN git submodule update
RUN sed -i "s/$OLD_TEXT/$NEW_TEXT/g" $CONFIG_FILE
RUN make clean
RUN make pp_build BUILD_TYPE=Release || (echo "Error, compilation  failed, exiting the build process..." ; exit 1)
