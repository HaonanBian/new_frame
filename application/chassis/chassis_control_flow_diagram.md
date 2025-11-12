# 底盘控制程序流程图

## 1. 总览流程图

```mermaid
flowchart TD
    subgraph 初始化阶段
        A[系统启动] --> B[ChassisInit
底盘初始化]
        B --> C[初始化M3508电机
PowerControlInit]
        C --> D[初始化PID参数和控制器]
        D --> E[初始化通信系统
Pub/Sub或CAN]
        E --> F[初始化超级电容和裁判系统]
    end
    
    subgraph 运行时阶段
        G[系统启动完成] --> H[ChassisTask
周期性执行]
        H --> I[获取控制命令
chassis_cmd_recv]
        I --> J[设置功率限制
SetPowerLimit]
        J --> K{检查控制模式}
        
        K -->|CHASSIS_ZERO_FORCE| L[停止所有电机
DJIMotorStop]
        K -->|其他模式| M[启用所有电机
DJIMotorEnable]
        
        M --> N{设置旋转速度wz}
        N -->|CHASSIS_NO_FOLLOW| O[wz = 0]
        N -->|CHASSIS_FOLLOW_GIMBAL_YAW| P[wz = f(offset_angle)]
        N -->|CHASSIS_ROTATE| Q[wz = 4000]
        
        O --> R[坐标系变换
计算chassis_vx/vy]
        P --> R
        Q --> R
        
        R --> S[运动学解算
MecanumCalculate]
        S --> T[功率限制和输出
LimitChassisOutput]
        T --> U[电机参考值设置
DJIMotorSetRef]
        U --> V[PowerControl
功率控制和CAN发送]
        
        V --> W[EstimateSpeed
速度估算]
        W --> X[推送反馈数据
PubPushMessage/CANCommSend]
        X --> H
    end
    
    F --> G
    L --> X
```

## 2. 底盘初始化详细流程图

```mermaid
flowchart TD
    A[ChassisInit开始] --> B[配置电机参数结构
Motor_Init_Config_s]
    B --> C[配置M3508速度环PID
Kp=4.5, Ki=0, Kd=0]
    C --> D[配置电机控制设置
速度环闭环]
    
    D --> E1[初始化左前电机
PowerControlInit]
    D --> E2[初始化右前电机
PowerControlInit]
    D --> E3[初始化左后电机
PowerControlInit]
    D --> E4[初始化右后电机
PowerControlInit]
    
    E1 --> F[初始化缓冲能量PID
Buffer_pid_conf]
    E2 --> F
    E3 --> F
    E4 --> F
    
    F --> G[初始化超级电容
SuperCapInit]
    G --> H{硬件模式判断}
    
    H -->|CHASSIS_BOARD
双板模式| I[初始化底盘IMU
INS_Init]
    I --> J[初始化CAN通信
CANCommInit]
    
    H -->|ONE_BOARD
单板模式| K[初始化消息发布
PubRegister]
    K --> L[初始化消息订阅
SubRegister]
    
    J --> M[ChassisInit完成]
    L --> M
```

## 3. ChassisTask运行时详细流程图

```mermaid
flowchart TD
    A[ChassisTask开始] --> B{获取控制命令}
    B -->|ONE_BOARD| C[SubGetMessage
获取chassis_cmd_recv]
    B -->|CHASSIS_BOARD| D[CANCommGet
获取chassis_cmd_recv]
    
    C --> E[SetPowerLimit
设置功率限制]
    D --> E
    
    E --> F{chassis_mode判断}
    F -->|CHASSIS_ZERO_FORCE| G[停止四个轮毂电机]
    F -->|其他模式| H[启用四个轮毂电机]
    
    G --> P[推送反馈数据]
    
    H --> I{chassis_mode判断}
    I -->|CHASSIS_NO_FOLLOW| J[wz = 0]
    I -->|CHASSIS_FOLLOW_GIMBAL_YAW| K[wz = -1.5f * offset_angle * abs(offset_angle)]
    I -->|CHASSIS_ROTATE| L[wz = 4000]
    
    J --> M[坐标系变换
根据offset_angle计算vx/vy]
    K --> M
    L --> M
    
    M --> N[MecanumCalculate
计算各轮速度vt_lf/vt_rf/vt_lb/vt_rb]
    N --> O[LimitChassisOutput
设置电机参考值]
    
    O --> Q[EstimateSpeed
速度估算]
    Q --> P
    P --> A
```

## 4. PowerControl功率控制流程图

```mermaid
flowchart TD
    A[PowerControl开始] --> B[遍历所有电机实例]
    B --> C{检查速度环配置}
    C -->|启用速度环| D[计算速度环PID
PIDCalculate]
    D --> E[计算电机实际转矩]
    E --> F[计算初始功率需求]
    
    F --> G[累加总功率需求]
    G --> H{总功率 > 最大功率限制?}
    
    H -->|否| I[保持原有输出]
    H -->|是| J[计算比例系数ratio]
    J --> K[按比例调整各电机功率]
    K --> L[解二次方程重新计算控制量]
    
    L --> M[限制输出范围
-15000~15000]
    I --> N[设置CAN发送缓冲区
sender_assignment]
    M --> N
    
    N --> O{检查电机停止标志}
    O -->|停止| P[清零对应CAN缓冲区]
    O -->|启用| Q[保留缓冲区值]
    
    P --> R[遍历sender_enable_flag]
    Q --> R
    R --> S{是否需要发送?}
    S -->|是| T[CANTransmit
发送CAN报文]
    S -->|否| U[跳过发送]
    
    T --> V[PowerControl结束]
    U --> V
```

## 5. 电机控制与反馈流程图

```mermaid
flowchart TD
    subgraph 控制命令发送
        A[DJIMotorSetRef
设置电机参考值] --> B[更新motor_controller.pid_ref]
        B --> C[PowerControl
功率控制计算]
        C --> D[CANTransmit
发送CAN控制报文]
    end
    
    subgraph 反馈数据处理
        E[CAN接收中断] --> F[DecodeDJIMotor
解析电机反馈]
        F --> G[更新measure结构
角度/速度/电流]
        G --> H[计算多圈角度
measure.total_angle]
        H --> I[更新PID反馈值]
    end
    
    D --> J[电机驱动执行]
    J --> E
```

## 6. 底盘坐标系变换流程图

```mermaid
flowchart TD
    A[输入控制命令
chassis_cmd_recv.vx/vy] --> B[计算cos_theta
cos(offset_angle*DEGREE_2_RAD)]
    A --> C[计算sin_theta
sin(offset_angle*DEGREE_2_RAD)]
    
    B --> D[chassis_vx = vx*cos_theta - vy*sin_theta]
    C --> E[chassis_vy = vx*sin_theta + vy*cos_theta]
    
    D --> F[运动学解算输入]
    E --> F
```

## 7. 麦轮运动学解算流程图

```mermaid
flowchart TD
    A[输入底盘速度
chassis_vx/chassis_vy/chassis_cmd_recv.wz] --> B[计算各轮速度]
    B --> C1[vt_lf = -vx - vy - wz*LF_CENTER]
    B --> C2[vt_rf = -vx + vy - wz*RF_CENTER]
    B --> C3[vt_lb = vx - vy - wz*LB_CENTER]
    B --> C4[vt_rb = vx + vy - wz*RB_CENTER]
    
    C1 --> D[LimitChassisOutput
设置电机参考值]
    C2 --> D
    C3 --> D
    C4 --> D
```

## 8. 控制模式状态转换图

```mermaid
stateDiagram-v2
    [*] --> CHASSIS_ZERO_FORCE
    
    CHASSIS_ZERO_FORCE --> CHASSIS_NO_FOLLOW : 模式切换
    CHASSIS_ZERO_FORCE --> CHASSIS_FOLLOW_GIMBAL_YAW : 模式切换
    CHASSIS_ZERO_FORCE --> CHASSIS_ROTATE : 模式切换
    
    CHASSIS_NO_FOLLOW --> CHASSIS_ZERO_FORCE : 急停触发
    CHASSIS_NO_FOLLOW --> CHASSIS_FOLLOW_GIMBAL_YAW : 模式切换
    CHASSIS_NO_FOLLOW --> CHASSIS_ROTATE : 模式切换
    
    CHASSIS_FOLLOW_GIMBAL_YAW --> CHASSIS_ZERO_FORCE : 急停触发
    CHASSIS_FOLLOW_GIMBAL_YAW --> CHASSIS_NO_FOLLOW : 模式切换
    CHASSIS_FOLLOW_GIMBAL_YAW --> CHASSIS_ROTATE : 模式切换
    
    CHASSIS_ROTATE --> CHASSIS_ZERO_FORCE : 急停触发
    CHASSIS_ROTATE --> CHASSIS_NO_FOLLOW : 模式切换
    CHASSIS_ROTATE --> CHASSIS_FOLLOW_GIMBAL_YAW : 模式切换
```

## 9. 完整数据流程图

```mermaid
flowchart TD
    subgraph 输入层
        A[遥控器输入] --> B[机器人控制命令
RobotControlCmd]
        C[云台角度反馈] --> B
        D[裁判系统数据] --> E[功率限制设置]
    end
    
    subgraph 控制层
        B --> F[ChassisTask
底盘任务]
        E --> F
        F --> G[坐标系变换]
        G --> H[运动学解算
MecanumCalculate]
        H --> I[功率控制
PowerControl]
    end
    
    subgraph 驱动层
        I --> J[CAN通信模块]
        J --> K[M3508电机驱动]
        K --> L[轮毂电机执行]
    end
    
    subgraph 反馈层
        L --> M[电机编码器反馈]
        M --> N[电机状态解析
DecodeDJIMotor]
        N --> O[速度估算
EstimateSpeed]
        O --> P[底盘状态反馈]
        P --> Q[上级控制系统]
    end
    
    Q --> B
```

## 10. 关键函数调用关系图

```mermaid
flowchart TD
    A[RobotInit
系统初始化] --> B[ChassisInit
底盘初始化]
    
    C[RobotTask
系统任务] --> D[ChassisTask
底盘任务]
    
    B --> E[PowerControlInit
电机初始化]
    B --> F[PIDInit
PID控制器初始化]
    B --> G[SuperCapInit
超级电容初始化]
    
    D --> H[SetPowerLimit
设置功率限制]
    D --> I[DJIMotorStop/Enable
电机启停控制]
    D --> J[MecanumCalculate
运动学解算]
    D --> K[LimitChassisOutput
输出限制]
    D --> L[EstimateSpeed
速度估算]
    
    K --> M[DJIMotorSetRef
设置电机参考值]
    M --> N[PowerControl
功率控制]
    
    N --> O[PIDCalculate
PID计算]
    N --> P[CANTransmit
CAN发送]
    
    Q[CAN中断] --> R[DecodeDJIMotor
解析电机反馈]
```