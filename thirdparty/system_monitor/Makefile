# 编译器设置
CXX := g++
CXXFLAGS := -Wall -Wextra -O2

# 目标可执行文件名称
TARGET := system_monitor

# 自动查找所有.cpp文件
SOURCES := $(wildcard *.cpp)

INCLUDES = -I../../common/threadsafe_container

LDFLAGS := -Wl,--no-as-needed -lpthread -lz
# 默认目标
all: $(TARGET)

# 直接编译所有源文件生成可执行文件
$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(TARGET) $(LDFLAGS) $(INCLUDES)

# 清理
clean:
	rm -f $(TARGET)

# 重新编译
rebuild: clean all

# 显示信息
info:
	@echo "Target: $(TARGET)"
	@echo "Sources: $(SOURCES)"
	@echo "Compiler: $(CXX)"
	@echo "Flags: $(CXXFLAGS)"

.PHONY: all clean rebuild info
