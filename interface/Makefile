all:
	@cmake -B build -DINTERFACE_COMPILE_MODE=check_type_convert
	@cmake --build build -j $(nproc)
	@cmake --build build --target rosbag_converter -j $(nproc)
	@cmake --build build --target convert_test -j $(nproc)

rc:
	@rm -f install
	@cmake -B build -DINTERFACE_COMPILE_MODE=check_type_convert
	@cmake --build build --target rosbag_converter -j $(nproc)

check:
	@cmake -B build -DCHECK_MODULE="${CHECK_MODULE}" -DINTERFACE_COMPILE_MODE=check_type_convert
	@cmake --build build -j $(nproc)
	@cmake --build build --target rosbag_converter -j $(nproc)

checkall:
	@cmake -B build -DCHECK_MODULE="*.h" -DINTERFACE_COMPILE_MODE=check_type_convert
	@cmake --build build -j $(nproc)
	@cmake --build build --target rosbag_converter -j $(nproc)

clean:
	@rm -rf build install