#!/bin/bash

set -e  # 遇到错误立即退出

BUILD_DIR="build"
CMAKE_TOOLCHAIN_FILE="../cmake/gcc-arm-none-eabi.cmake"

echo "=== 构建项目 ==="

# 检查是否需要重新运行 CMake
if [ ! -d "$BUILD_DIR" ]; then
    echo "创建构建目录..."
    mkdir -p "$BUILD_DIR"
fi

cd "$BUILD_DIR"

if [ ! -f "CMakeCache.txt" ] || [ "$CMAKE_TOOLCHAIN_FILE" -nt "CMakeCache.txt" ]; then
    echo "运行 CMake 配置..."
    cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE="$CMAKE_TOOLCHAIN_FILE" ..
else
    echo "CMake 配置未变化，使用现有配置"
fi

echo "开始编译..."
ninja -j24

echo "=== 构建完成 ==="
cd ..