PACKAGE_NAME := planning
PACKAGE_VERSION := planning.2.0.0.8

include .ci/utils.mk

pybind_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) -DPYBIND_TOOL_ENABLE=True .. && \
	make -j $(NUM_JOB)"

pp_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) -DPP_ENABLE=True .. && \
	make -j $(NUM_JOB) && \
	make install"
