# 机器人各模块交互拓扑图

本文档使用Mermaid语法详细描述机器人系统中各模块之间的交互关系、数据流向和通信机制。拓扑图基于实际代码分析，覆盖了系统的主要组件和它们之间的依赖关系。

## 1. 系统整体架构拓扑图

```mermaid
graph TB
    subgraph 应用层[应用层]
        RobotTask[机器人主任务] --> ChassisTask[底盘任务]
        RobotTask --> GimbalTask[云台任务]
        RobotTask -.-> ShootTask[发射任务]:::disabled
        RobotTask --> RobotCMDTask[命令处理任务]
    end

    subgraph 核心任务层[核心任务层]
        INSTask[惯性导航任务] 
        MotorControlTask[电机控制任务]
        DaemonTask[守护任务]
        UITask[UI任务]
    end

    subgraph 硬件驱动层[硬件驱动层]
        BMI088[BMI088 IMU传感器]
        DJIMotor[大疆电机驱动]
        DMmotor[DM电机驱动]
        HTmotor[HT电机驱动]
        BSP[板级支持包]
        Buzzer[蜂鸣器]
    end

    subgraph 通信层[通信层]
        CAN[CAN通信]
        USART[串口通信]
        VisionSend[视觉数据发送]
        PowerControl[功率控制模块]
    end

    subgraph 模式配置[模式配置]
        OneBoard[单板模式] -.OR.-> DualBoard[双板模式]
        DualBoard --> ChassisBoard[底盘板]
        DualBoard --> GimbalBoard[云台板]
    end

    %% 初始化流程
    RobotInit[RobotInit初始化] --> BSPInit[BSPInit初始化]
    RobotInit --> ChassisInit[ChassisInit初始化]
    RobotInit --> GimbalInit[GimbalInit初始化]
    RobotInit -.-> RobotCMDInit[RobotCMDInit初始化]
    RobotInit --> OSTaskInit[OSTaskInit初始化]
    
    %% 任务创建
    OSTaskInit --> INSTask
    OSTaskInit --> MotorControlTask
    OSTaskInit --> DaemonTask
    OSTaskInit --> RobotTask
    OSTaskInit --> UITask
    
    %% 数据采集
    INSTask --> BMI088
    BMI088 --> INSTask
    
    %% 电机控制链
    ChassisTask --> PowerControl
    GimbalTask --> PowerControl
    ShootTask -.-> PowerControl
    
    PowerControl --> MotorControlTask
    MotorControlTask --> MotorSenderGrouping[电机分组发送]
    MotorSenderGrouping --> CAN
    
    CAN --> DJIMotor
    CAN --> DMmotor
    DJIMotor --> DecodeDJIMotor[解码电机数据]
    DMmotor --> DecodeDJIMotor
    DecodeDJIMotor --> MotorControlTask
    
    %% 守护与监控
    DaemonTask --> Buzzer
    DaemonTask --> MotorControlTask
    
    %% 通信流
    INSTask --> VisionSend
    VisionSend --> USART
    UITask --> RefereeSend[裁判系统通信]
    
    %% 控制命令流
    RobotCMDTask --> Chassis_Ctrl_Cmd[底盘控制命令]
    RobotCMDTask --> Gimbal_Ctrl_Cmd[云台控制命令]
    RobotCMDTask -.-> Shoot_Ctrl_Cmd[发射控制命令]
    
    Chassis_Ctrl_Cmd --> ChassisTask
    Gimbal_Ctrl_Cmd --> GimbalTask
    Shoot_Ctrl_Cmd -.-> ShootTask
    
    %% 条件编译控制
    OneBoard -.编译控制.-> RobotInit
    ChassisBoard -.编译控制.-> ChassisInit
    GimbalBoard -.编译控制.-> GimbalInit
    
    classDef application fill:#f9f,stroke:#333,stroke-width:2px;
    classDef core fill:#bbf,stroke:#333,stroke-width:2px;
    classDef hardware fill:#bfb,stroke:#333,stroke-width:2px;
    classDef communication fill:#fbb,stroke:#333,stroke-width:2px;
    classDef config fill:#ff9,stroke:#333,stroke-width:2px;
    classDef disabled fill:#ddd,stroke:#999,stroke-width:2px,stroke-dasharray: 5 5;
    
    class RobotTask,ChassisTask,GimbalTask,RobotCMDTask application;
    class ShootTask disabled;
    class INSTask,MotorControlTask,DaemonTask,UITask core;
    class BMI088,DJIMotor,DMmotor,HTmotor,BSP,Buzzer hardware;
    class CAN,USART,VisionSend,PowerControl communication;
    class OneBoard,DualBoard,ChassisBoard,GimbalBoard config;
```

## 2. 任务调度与优先级关系

```mermaid
graph TD
    OS[FreeRTOS] --> INSTask[INS任务\n优先级:AboveNormal\n频率:1kHz]
    OS --> MotorControlTask[电机控制任务\n优先级:Normal\n频率:1kHz]
    OS --> DaemonTask[守护任务\n优先级:Normal\n频率:100Hz]
    OS --> RobotTask[机器人主任务\n优先级:Normal\n频率:200-500Hz]
    OS --> UITask[UI任务\n优先级:Normal\n频率:~1kHz]
    
    classDef task fill:#f9f,stroke:#333,stroke-width:1px;
    class INSTask,MotorControlTask,DaemonTask,RobotTask,UITask task;
```

## 3. 数据流向图

```mermaid
graph LR
    subgraph 控制命令流[控制命令流]
        Remote[遥控器输入] --> RobotCMDTask
        RobotCMDTask --> Chassis_Ctrl_Cmd[Chassis_Ctrl_Cmd_s]
        RobotCMDTask --> Gimbal_Ctrl_Cmd[Gimbal_Ctrl_Cmd_s]
        RobotCMDTask --> Shoot_Ctrl_Cmd[Shoot_Ctrl_Cmd_s]
        
        Chassis_Ctrl_Cmd --> ChassisTask
        Gimbal_Ctrl_Cmd --> GimbalTask
        Shoot_Ctrl_Cmd --> ShootTask
    end
    
    subgraph 反馈数据流[反馈数据流]
        DJIMotor --> DecodeDJIMotor[解码电机数据]
        DecodeDJIMotor --> MotorControlTask
        
        BMI088 --> INS_Task
        INS_Task --> attitude[attitude_t姿态数据]
        attitude --> GimbalTask
        attitude --> ChassisTask
    end
    
    subgraph 控制执行流[控制执行流]
        ChassisTask --> PowerControl[功率控制]
        GimbalTask --> PowerControl
        ShootTask --> PowerControl
        
        PowerControl --> MotorSenderGrouping[电机分组发送]
        MotorSenderGrouping --> CAN
        CAN --> DJIMotor
    end
    
    classDef flow fill:#f9f,stroke:#333,stroke-width:1px;
    class Remote,RobotCMDTask,Chassis_Ctrl_Cmd,Gimbal_Ctrl_Cmd,Shoot_Ctrl_Cmd,ChassisTask,GimbalTask,ShootTask flow;
    class DJIMotor,DecodeDJIMotor,MotorControlTask,BMI088,INS_Task,attitude flow;
    class PowerControl,MotorSenderGrouping,CAN flow;
```

## 4. 模块职责与依赖关系

```mermaid
graph LR
    subgraph 应用层[应用层]
        Chassis[底盘模块]
        Gimbal[云台模块]
        Shoot[发射模块]
        RobotCMD[命令处理模块]
    end
    
    subgraph 中间层[中间层]
        PowerControl[功率控制模块]
        MotionControl[运动控制模块]
    end
    
    subgraph 底层[底层]
        MotorControl[电机控制模块]
        INS[惯性导航模块]
        BSP[板级支持包]
    end
    
    Chassis --> PowerControl
    Gimbal --> PowerControl
    Shoot --> PowerControl
    
    Chassis --> INS
    Gimbal --> INS
    
    PowerControl --> MotorControl
    MotionControl --> MotorControl
    
    MotorControl --> BSP
    INS --> BSP
    
    RobotCMD --> Chassis
    RobotCMD --> Gimbal
    RobotCMD --> Shoot
    
    classDef layer1 fill:#f9f,stroke:#333,stroke-width:1px;
    classDef layer2 fill:#bbf,stroke:#333,stroke-width:1px;
    classDef layer3 fill:#bfb,stroke:#333,stroke-width:1px;
    
    class Chassis,Gimbal,Shoot,RobotCMD layer1;
    class PowerControl,MotionControl layer2;
    class MotorControl,INS,BSP layer3;
```

## 5. 配置与参数依赖关系

```mermaid
graph TD
    robot_def[robot_def.h] --> RobotTask[机器人任务]
    robot_def --> ChassisTask[底盘任务]
    robot_def --> GimbalTask[云台任务]
    robot_def --> ShootTask[发射任务]
    
    robot_def --> Chassis_Ctrl_Cmd[底盘控制命令]
    robot_def --> Gimbal_Ctrl_Cmd[云台控制命令]
    robot_def --> Shoot_Ctrl_Cmd[发射控制命令]
    
    robot_def --> Chassis_Upload_Data[底盘上传数据]
    robot_def --> Gimbal_Upload_Data[云台上传数据]
    robot_def --> Shoot_Upload_Data[发射上传数据]
    
    ins_task[ins_task.h] --> INSTask[惯性导航任务]
    motor_task[motor_task.h] --> MotorControlTask[电机控制任务]
    
    classDef config fill:#f9f,stroke:#333,stroke-width:1px;
    classDef task fill:#bbf,stroke:#333,stroke-width:1px;
    classDef data fill:#bfb,stroke:#333,stroke-width:1px;
    
    class robot_def,ins_task,motor_task config;
    class RobotTask,ChassisTask,GimbalTask,ShootTask,INSTask,MotorControlTask task;
    class Chassis_Ctrl_Cmd,Gimbal_Ctrl_Cmd,Shoot_Ctrl_Cmd,Chassis_Upload_Data,Gimbal_Upload_Data,Shoot_Upload_Data data;
```

## 6. 守护线程与系统监控拓扑图

```mermaid
graph TD
    DaemonTask[守护任务] --> MotorHealthCheck[电机健康检查]
    DaemonTask --> SystemStatusMonitor[系统状态监控]
    DaemonTask --> BuzzerControl[蜂鸣器控制]
    
    MotorHealthCheck --> DJIMotorStatus[大疆电机状态]
    MotorHealthCheck --> DMmotorStatus[DM电机状态]
    MotorHealthCheck --> HTmotorStatus[HT电机状态]
    
    SystemStatusMonitor --> INSStatus[惯导系统状态]
    SystemStatusMonitor --> PowerSupplyStatus[电源状态]
    SystemStatusMonitor --> TaskDelayMonitor[任务延时监控]
    
    TaskDelayMonitor --> INSTaskDelay[INS任务延时监控]
    TaskDelayMonitor --> MotorTaskDelay[电机任务延时监控]
    TaskDelayMonitor --> RobotTaskDelay[机器人任务延时监控]
    
    BuzzerControl --> BuzzerTask[蜂鸣器任务]
    BuzzerTask --> Buzzer[蜂鸣器硬件]
    
    classDef daemon fill:#f9f,stroke:#333,stroke-width:1px;
    classDef health fill:#bbf,stroke:#333,stroke-width:1px;
    classDef status fill:#bfb,stroke:#333,stroke-width:1px;
    
    class DaemonTask,BuzzerControl daemon;
    class MotorHealthCheck,DJIMotorStatus,DMmotorStatus,HTmotorStatus health;
    class SystemStatusMonitor,INSStatus,PowerSupplyStatus,TaskDelayMonitor,INSTaskDelay,MotorTaskDelay,RobotTaskDelay,BuzzerTask,Buzzer status;
```

## 主要模块说明

1. **应用层模块**：
   - 底盘模块(Chassis)：负责底盘运动控制，支持零力模式、旋转模式、不跟随模式和跟随云台模式
   - 云台模块(Gimbal)：负责云台姿态控制，支持零力模式、自由模式和陀螺仪反馈模式
   - 发射模块(Shoot)：负责发射系统控制（当前代码中被注释禁用）
   - 命令处理模块(RobotCMD)：处理遥控器输入并生成各模块的控制命令

2. **核心任务层**：
   - 惯性导航任务(INSTask)：以1kHz频率运行，处理IMU数据，进行姿态解算，优先级较高
   - 电机控制任务(MotorControlTask)：以1kHz频率运行，控制各类型电机，处理电机反馈
   - 守护任务(DaemonTask)：以100Hz频率运行，监控系统状态和电机健康
   - 机器人主任务(RobotTask)：以200-500Hz频率运行，协调调用各应用层任务
   - UI任务(UITask)：处理用户界面和裁判系统通信

3. **硬件驱动层**：
   - BMI088 IMU传感器：提供加速度和角速度数据，用于姿态解算
   - 电机驱动：包括大疆电机、DM电机和HT电机驱动，通过CAN总线通信
   - 板级支持包(BSP)：提供底层硬件抽象，包括ADC、CAN、GPIO等驱动
   - 蜂鸣器：用于系统状态提示和警报

4. **通信层**：
   - CAN通信：用于电机控制命令发送和反馈数据接收
   - 串口通信：用于调试信息输出和视觉数据传输
   - 功率控制模块：实现功率限制算法，确保系统安全运行

5. **配置模式**：
   - 单板模式：所有功能在一个控制板上实现
   - 双板模式：分为底盘板和云台板，通过通信协议协同工作

通过这个拓扑图，可以清晰地了解机器人系统中各模块的组织结构、任务优先级、数据流向以及依赖关系，有助于系统的维护、调试和开发。拓扑图中虚线表示当前代码中被注释禁用的功能，实线表示活跃的功能模块。