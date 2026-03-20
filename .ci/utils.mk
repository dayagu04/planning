# usage: include .ci/utils.mk

PACKAGE_NAME ?=
PACKAGE_VERSION ?=

#Release #Debug
BUILD_TYPE ?= Debug
INSTALL_DIR ?= install
#cyber ap
ADAPTER ?= cyber
#BZT X86
PLATFORM ?= X86
PRODUCT ?= JAC_S811
NUM_JOB ?= 16

CMAKE_ARGS := \
	-DPACKAGE_VERSION=$(PACKAGE_VERSION) \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DPLATFORM=$(PLATFORM) \
	-DPRODUCT=$(PRODUCT) \
	-DADAPTER=$(ADAPTER)

.PHONY: clang-format submodules clean build package unittest_build unittest_run unittest_gdb unittest_coverage jupyter_start

clang-format:
	cp .ci/_clang-format .clang-format
	.ci/format.py
submodules:
	/bin/bash -c "git submodule update --init --recursive"
clean:
	rm -rf build
	rm -rf install
build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) .. && \
	make -j $(NUM_JOB) && \
	make install"
package:
	/bin/bash -c "rm target/$(PACKAGE_NAME).tar 2>/dev/null; \
	cd $(INSTALL_DIR) && \
	tar -cvf ../target/$(PACKAGE_NAME).tar $(PACKAGE_NAME)"

unittest_build:
	mkdir -p build && cd build && \
	/bin/bash -c "cmake $(CMAKE_ARGS) .. && \
	make -j $(NUM_JOB) unit_test && \
	make install"
unittest_run: build/test/unit_test
	$<
unittest_gdb: build/test/unit_test
	gdb -ex="set history save on; set pagination off; set breakpoint pending on; run" --args $<
unittest_coverage:
	mkdir -p build && cd build && \
    /bin/bash -c "sudo apt install -y lcov && \
	cmake $(CMAKE_ARGS) -DCODE_COVERAGE=ON .. && \
    make -j $(NUM_JOB) gen_unittest_coverage"

jupyter_start:
	/bin/bash -c "jupyter notebook --ip 0.0.0.0 --port 8080 --allow-root"

# my_build_all:
# 	python3 $(RUN_IN_BUILD_ENV_PY) --build-env $(BUILD_ENVS_JSON) --includes u16 devcar orin -- make build
# my_package_all:
# 	python3 $(RUN_IN_BUILD_ENV_PY) --build-env $(BUILD_ENVS_JSON) --includes u16 devcar orin -- make package
# my_upload_all:
# 	python3 $(RUN_IN_BUILD_ENV_PY) --build-env $(BUILD_ENVS_JSON) --includes u16 devcar orin -- make upload
