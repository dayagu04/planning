# 一些make外部调用接口

# 搭建初期暂时方便个人操作，后期增量编译
clean:
	rm -rf build/ && \
	rm -rf src/proto/generated_files/*.cc && \
	rm -rf src/proto/generated_files/*.h

build:
	mkdir -p build && cd build && \
	cmake .. && \
	make -j 32

deploy:
	mkdir -p build/Planning/Lib && \
	mkdir -p build/Planning/Res/config && \
	mkdir -p build/Planning/log && \
	cp	build/src/libplanning_component.so build/Planning/Lib && \
	cp	build/src/libapa_simulation_component.so build/Planning/Lib && \
	cp -rf res/conf/* build/Planning/Res/config && \
	cp -rf res/dag/* build/Planning && \
	cp -rf res/scripts build/Planning && \
	cp -rf CHANGELOG.md build/Planning && \
	cp -rf README.md build/Planning
