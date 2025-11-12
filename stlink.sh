#!/bin/bash

# OpenOCD 可执行文件路径
OPENOCD_PATH="D:/toolchains/openocd/openocd/bin/openocd.exe"

# 程序文件路径
BIN_FILE="build/aurora-farmework.elf"

# OpenOCD 配置文件路径
CONFIG_DIR="D:/toolchains/openocd/openocd/share/openocd/scripts"

# ST-Link 配置文件
STLINK_CONFIG="$CONFIG_DIR/interface/stlink.cfg"

# STM32 配置文件（根据你的芯片型号选择合适的文件）
STM32_CONFIG="$CONFIG_DIR/target/stm32f4x.cfg"

# 检查文件是否存在
if [ ! -f "$BIN_FILE" ]; then
  echo "错误：程序文件 $BIN_FILE 不存在！"
  exit 1
fi

if [ ! -f "$STLINK_CONFIG" ]; then
  echo "错误：ST-Link 配置文件 $STLINK_CONFIG 不存在！"
  exit 1
fi

if [ ! -f "$STM32_CONFIG" ]; then
  echo "错误：STM32 配置文件 $STM32_CONFIG 不存在！"
  exit 1
fi

# 启动 OpenOCD 并烧录程序
echo "开始烧录程序到 STM32..."
$OPENOCD_PATH -f "$STLINK_CONFIG" -f "$STM32_CONFIG" -c "init; reset halt; wait_halt; flash write_image erase $BIN_FILE $FLASH_ADDRESS; reset; shutdown"

# 检查烧录是否成功
if [ $? -eq 0 ]; then
  echo "烧录完成！"
else
  echo "烧录失败，请检查错误信息！"
fi