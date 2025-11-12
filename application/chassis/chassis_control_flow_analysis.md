# 底盘控制模块关系分析

## 1. 初始化流程

**ChassisInit() 函数流程：**
1. **电机初始化**：
   - 配置M3508电机参数（PID参数、CAN通信等）
   - 通过PowerControlInit()初始化四个轮毂电机（LF、RF、LB、RB）
   - 设置电机方向标志

2. **PID与控制初始化**：
   - 初始化缓冲能量PID（buffer_PID）
   - 配置超级电容（SuperCapInit）

3. **通信初始化**：
   - 单板模式：初始化消息发布订阅系统
   - 双板模式：初始化CAN通信和IMU

## 2. 运行时流程

**ChassisTask() 主循环：**
1. **数据获取**：
   - 获取控制命令（单板通过消息订阅，双板通过CAN）
   - 设置功率限制

2. **电机状态控制**：
   - 根据模式决定电机启停
   - 启用或停止四个轮毂电机

3. **控制模式处理**：
   - CHASSIS_NO_FOLLOW：设置wz=0
   - CHASSIS_FOLLOW_GIMBAL_YAW：根据偏移角度计算wz
   - CHASSIS_ROTATE：设置固定wz值

4. **坐标变换**：
   - 根据云台角度偏移进行坐标系映射
   - 计算底盘vx和vy

5. **运动学解算**：
   - 调用MecanumCalculate()计算各轮速度

6. **输出限制**：
   - 调用LimitChassisOutput()进行功率限制和电机参考值设置

7. **速度估算**：
   - 调用EstimateSpeed()计算实际速度

8. **反馈推送**：
   - 推送底盘反馈数据

## 3. 功能模块关系

### 3.1 底盘与电机驱动
- **初始化关系**：ChassisInit() → PowerControlInit() → 电机实例创建
- **运行时关系**：ChassisTask() → LimitChassisOutput() → DJIMotorSetRef() → PowerControl()
- **通信关系**：通过DJIMotorInstance结构体进行数据交换

### 3.2 底盘与功率控制
- PowerControl()函数负责：
  - 计算串级PID
  - 实现功率限制算法
  - CAN报文分组发送
  - 处理电机反转
  - 支持速度/电流前馈控制

### 3.3 底盘与通信系统
- 单板模式：使用发布订阅（Publisher/Subscriber）
- 双板模式：使用CAN通信（CANComm）

### 3.4 底盘与裁判系统
- 获取功率限制信息
- 获取机器人状态数据

### 3.5 底盘与IMU（双板模式）
- 使用IMU数据进行速度估算和姿态控制

## 4. 状态转换

底盘控制模式转换：
- CHASSIS_ZERO_FORCE：急停模式，电机停止
- CHASSIS_NO_FOLLOW：无旋转模式，wz=0
- CHASSIS_FOLLOW_GIMBAL_YAW：跟随云台模式，根据角度偏移计算wz
- CHASSIS_ROTATE：自旋模式，固定wz值

## 5. 数据流向

1. **命令输入**：
   - 控制命令 → 坐标系转换 → 运动学解算 → 各轮速度

2. **反馈输出**：
   - 电机速度反馈 → 速度估算 → 底盘状态数据

3. **控制流**：
   - 控制命令 → 串级PID计算 → 功率限制 → 电机输出