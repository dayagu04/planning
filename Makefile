PACKAGE_NAME := planning

include .ci/utils.mk

pybind_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) -DPYBIND_TOOL_ENABLE=True .. && \
	make -j jupyter_pybind && \
	make install"

pp_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) .. && \
	make -j pp && \
	make install"
