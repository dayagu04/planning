PACKAGE_NAME := planning

include .ci/utils.mk

pybind_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) -DPYBIND_TOOL_ENABLE=True .. && \
	make -j"

pp_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) -DPP_ENABLE=True .. && \
	make -j && \
	make install"
