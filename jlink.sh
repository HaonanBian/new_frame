#!/bin/bash

# 配置参数
JLINK_EXE=""D:/toolchains/SEGGER/JLink_V844a/JLink.exe""  # JLink路径"C:/Program Files/SEGGER/JLink_V812f/JLink.exe"
DEVICE="stm32f4x"      # 芯片型号
INTERFACE="SWD"           # 接口类型
SPEED="40000"              # 通信速度 (kHz)
BIN_FILE="build/aurora-farmework.elf"  # 要烧录的BIN文件
FLASH_ADDR="0x08000000"   # Flash起始地址

# 自动烧录命令
"$JLINK_EXE" << EOF
si 1
device $DEVICE
if $INTERFACE
speed $SPEED
connect
halt
loadfile $BIN_FILE $FLASH_ADDR
r
q
EOF

# 检查执行结果
if [ $? -eq 0 ]; then
    echo "✔ 烧录成功！"
else
    echo "✘ 烧录失败！"
    exit 1
fi