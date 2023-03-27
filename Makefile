# 一些make外部调用接口

# 搭建初期暂时方便个人操作，后期增量编译
clean:
	rm -rf build/ && \
	rm -rf src/proto/generated_files/*.cc && \
	rm -rf src/proto/generated_files/*.h

build:
	mkdir -p build && cd build && \
	cmake .. && \
	make -j 4
