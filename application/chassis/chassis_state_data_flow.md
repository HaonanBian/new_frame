# 底盘控制状态转换与数据流向

## 1. 状态转换流程

```mermaid
stateDiagram-v2
    [*] --> 初始化
    初始化 --> 运行时
    运行时 --> 运行时 : 周期执行ChassisTask
    
    state 控制模式 {
        CHASSIS_ZERO_FORCE --> CHASSIS_NO_FOLLOW : 模式切换
        CHASSIS_ZERO_FORCE --> CHASSIS_FOLLOW_GIMBAL_YAW : 模式切换
        CHASSIS_ZERO_FORCE --> CHASSIS_ROTATE : 模式切换
        CHASSIS_NO_FOLLOW --> CHASSIS_FOLLOW_GIMBAL_YAW : 模式切换
        CHASSIS_NO_FOLLOW --> CHASSIS_ROTATE : 模式切换
        CHASSIS_FOLLOW_GIMBAL_YAW --> CHASSIS_NO_FOLLOW : 模式切换
        CHASSIS_FOLLOW_GIMBAL_YAW --> CHASSIS_ROTATE : 模式切换
        CHASSIS_ROTATE --> CHASSIS_NO_FOLLOW : 模式切换
        CHASSIS_ROTATE --> CHASSIS_FOLLOW_GIMBAL_YAW : 模式切换
        [*] --> CHASSIS_ZERO_FORCE : 默认启动
    }
    
    state 电机状态 {
        MOTOR_STOP --> MOTOR_ENABLE : 模式非CHASSIS_ZERO_FORCE
        MOTOR_ENABLE --> MOTOR_STOP : 模式为CHASSIS_ZERO_FORCE
    }
```

## 2. 详细数据流向

### 2.1 控制命令流

```mermaid
flowchart TD
    A[控制命令输入
Chassis_Ctrl_Cmd_s] --> B{检查控制模式}
    B -->|CHASSIS_ZERO_FORCE| C[电机停止
DJIMotorStop]
    B -->|其他模式| D[启用电机
DJIMotorEnable]
    D --> E{设置旋转速度}
    E -->|CHASSIS_NO_FOLLOW| F[wz = 0]
    E -->|CHASSIS_FOLLOW_GIMBAL_YAW| G[wz = f(offset_angle)]
    E -->|CHASSIS_ROTATE| H[wz = 定值]
    F --> I[坐标系变换]
    G --> I
    H --> I
    I --> J[运动学解算
MecanumCalculate]
    J --> K[功率限制和输出
LimitChassisOutput]
    K --> L[电机参考值设置
DJIMotorSetRef]
```

### 2.2 功率控制流

```mermaid
flowchart TD
    A[PowerControl
调用] --> B[遍历所有电机]
    B --> C[计算速度环PID]
    C --> D[计算初始功率]
    D --> E{总功率是否超限}
    E -->|是| F[计算比例系数]
    F --> G[按比例缩小各电机输出]
    E -->|否| H[保持原有输出]
    G --> I[设置CAN发送缓冲区]
    H --> I
    I --> J[CAN报文发送]
```

### 2.3 反馈数据流

```mermaid
flowchart TD
    A[电机反馈数据] --> B[DecodeDJIMotor
解析CAN数据]
    B --> C[更新电机测量值
measure结构]
    C --> D[EstimateSpeed
速度估算]
    D --> E[更新底盘反馈数据
Chassis_Upload_Data_s]
    E --> F{通信模式}
    F -->|单板模式| G[PubPushMessage
发布反馈]
    F -->|双板模式| H[CANCommSend
CAN发送]
```

## 3. 关键数据结构流向

### 3.1 Chassis_Ctrl_Cmd_s 流向
- **来源**：消息中心订阅（单板）或CAN接收（双板）
- **处理**：在ChassisTask中处理，更新vx, vy, wz
- **去向**：用于MecanumCalculate()的运动学解算

### 3.2 电机控制数据流向
- **从底盘到电机**：
  - ChassisTask() → LimitChassisOutput() → DJIMotorSetRef() → PowerControl() → CAN发送

- **从电机到底盘**：
  - CAN接收 → DecodeDJIMotor() → measure结构更新 → EstimateSpeed()

### 3.3 PID控制数据流
- **输入**：电机参考值、电机反馈值
- **处理**：PIDCalculate()计算控制量
- **输出**：电流控制值，通过CAN发送给电机

## 4. 初始化数据流

```mermaid
flowchart TD
    A[ChassisInit
调用] --> B[初始化电机配置
Motor_Init_Config_s]
    B --> C[初始化四个轮毂电机
PowerControlInit]
    C --> D[初始化缓冲PID
PIDInit]
    D --> E[初始化超级电容
SuperCapInit]
    E --> F{硬件模式}
    F -->|单板模式| G[初始化消息中心
Pub/Sub]
    F -->|双板模式| H[初始化IMU和CAN通信]
```

## 5. 模式转换详细逻辑

| 控制模式 | wz计算方式 | 电机状态 | 适用场景 |
|---------|-----------|---------|----------|
| CHASSIS_ZERO_FORCE | 不计算，电机停止 | 停止 | 急停、重要模块离线 |
| CHASSIS_NO_FOLLOW | wz = 0 | 启用 | 调整云台姿态，底盘不旋转 |
| CHASSIS_FOLLOW_GIMBAL_YAW | wz = -1.5f * offset_angle * abs(offset_angle) | 启用 | 底盘跟随云台旋转 |
| CHASSIS_ROTATE | wz = 4000 | 启用 | 底盘自旋，保持全向机动 |

## 6. 功率限制算法流程

```mermaid
flowchart TD
    A[计算各电机初始功率] --> B[累加总功率]
    B --> C{总功率 > 最大功率限制?}
    C -->|否| D[保持原输出]
    C -->|是| E[计算比例系数: ratio = max_power / total_power]
    E --> F[按比例缩小各电机功率]
    F --> G[解二次方程重新计算控制量]
    G --> H[限制输出范围]
    H --> I[更新CAN发送缓冲区]
    D --> I
```