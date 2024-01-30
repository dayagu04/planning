PACKAGE_NAME := planning
PACKAGE_VERSION := planning.2.3.2.3

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

ut_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) -DUNIT_TEST_ENABLE=True .. && \
	make -j $(NUM_JOB)"

submodules_update:
	/bin/bash -c " \
	git submodule deinit src/thirdparty/interface; \
	git rm --cached src/thirdparty/interface; \
	git submodule add ../interface.git interface; \
	git submodule add ../ad_third_party_libraries.git thirdparty; \
	git submodule update --init --recursive"
