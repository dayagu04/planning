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
	mkdir -p build/planning/lib && \
	mkdir -p build/planning/res/conf && \
	mkdir -p build/planning/log && \
	cp	build/src/libplanning_component.so build/planning/lib && \
	cp	build/src/libapa_simulation_component.so build/planning/lib && \
	cp -rf res/conf/* build/planning/res/conf && \
	cp -rf res/dag/* build/planning && \
	cp -rf res/scripts build/planning && \
	cp -rf CHANGELOG.md build/planning && \
	cp -rf README.md build/planning
