/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "arm_math.h"

// 调试：裁判系统状态检查计数器
static uint32_t referee_check_counter = 0;
#define REFEREE_CHECK_INTERVAL 100 // 每100次调用检查一次（约1秒）

/* 根据robot_def.h中的macro自动计算的参数 */
//#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
//#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define MOTOR_TO_CENTER 0.2626   // 轮子到中心的距离，单位m
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
#include "ins_task.h"
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
attitude_t *Chassis_IMU_data;                       // ONE_BOARD下复用一个IMU实例给底盘
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static PIDInstance angle_PID;              // 角度环 PID (外环)
static PIDInstance yaw_rate_PID;           // 角速度环 PID (内环,串级控制)
static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static referee_info_t *referee_data;                               // 裁判系统数据指针

// 调试用：裁判系统状态结构体（可在 Ozone 中直接展开监视）
typedef struct {
    uint8_t connected_flag;     // 连接状态：0=未连接, 1=已连接
    uint16_t last_cmd_id;
    uint8_t game_robot_state_rx;
    uint8_t power_heat_data_rx;
    uint8_t shoot_data_rx;
    uint16_t robot_id;          // 机器人ID
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t power_limit;       // 功率限制
    uint16_t heat_limit;
    uint16_t cooling_value;
    uint8_t gimbal_power_output;
    uint8_t chassis_power_output;
    uint8_t shooter_power_output;
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;        // 底盘功率
    uint16_t buffer_energy;
    uint16_t shooter_heat;      // 17mm枪口热量
    uint16_t shooter_17mm_2_heat;
    uint16_t shooter_42mm_heat;
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
    uint8_t recv_counter;       // 接收帧计数
    uint8_t crc8_fail_count;    // CRC8校验失败计数
    uint8_t crc16_fail_count;   // CRC16校验失败计数
} Referee_Debug_s;

extern Referee_Debug_s referee_debug; // 在 rm_referee.c 中定义

// 不接裁判系统时的默认功率限制
#define DEFAULT_POWER_LIMIT 45.0f

// === 复活检测相关 ===
#define CHASSIS_REVIVE_BUFFER_MS 200    // 复活后零输出缓冲时间 (ms)
#define CHASSIS_TASK_PERIOD_MS   5       // ChassisTask 周期 (200Hz)
#define CHASSIS_REVIVE_BUFFER_CYCLES (CHASSIS_REVIVE_BUFFER_MS / CHASSIS_TASK_PERIOD_MS)  // 40个周期 ≈ 200ms

// 底盘输出能力判断：HP > 0 且 裁判系统允许底盘输出
static uint8_t IsChassisOutputEnabled(void)
{
    if (referee_data == NULL) {
        // 无裁判系统时，默认允许输出
        return 1;
    }
    
    // 检查底盘是否可以被控制：HP > 0
    if (referee_data->GameRobotState.current_HP == 0) {
        return 0;
    }
    
    // 检查裁判系统是否允许底盘输出
    // power_management_chassis_output: 1=允许, 0=不允许(被裁判系统惩罚)
    if (referee_data->GameRobotState.power_management_chassis_output == 0) {
        return 0;
    }
    
    return 1;
}

// 复活检测状态机
typedef enum {
    CHASSIS_REVIVE_STATE_NORMAL = 0,  // 正常运行
    CHASSIS_REVIVE_STATE_JUST_ENABLED, // 刚检测到可输出，上升沿
    CHASSIS_REVIVE_STATE_BUFFER,       // 零输出缓冲中
} chassis_revive_state_e;

static chassis_revive_state_e chassis_revive_state = CHASSIS_REVIVE_STATE_NORMAL;
static uint32_t chassis_revive_start_tick = 0;  // 复活起始时间戳

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅

// 调试变量：底盘跟随误差角追踪
float debug_chassis_offset_recv = 0;     // 从 cmd 接收到的 offset_angle
float debug_chassis_follow_err = 0;      // 底盘跟随时实际使用的误差角 (wrap 后)
float debug_chassis_wz_output = 0;       // 底盘跟随 PID 输出的旋转速度 (滤波后)
float debug_chassis_wz_raw = 0;          // PID 输出的旋转速度 (滤波前)
float debug_chassis_yaw_rate_feedback = 0; // 角速度反馈值 (滤波后)
float debug_chassis_yaw_rate_raw = 0;   // 角速度反馈值 (滤波前)
float debug_chassis_yaw_rate_target = 0;   // 角速度目标值 (外环输出)
float debug_chassis_angle_output = 0;      // 外环输出 (内环给定)

// 复活检测调试变量（可在 Ozone 中直接展开监视）
float debug_chassis_revive_state = 0;       // 0=正常, 1=刚检测到复活, 2=缓冲中
float debug_chassis_output_enabled = 0;     // 当前底盘是否可输出
float debug_chassis_buffer_cycles = 0;      // 当前缓冲剩余周期数

// === 底盘跟随抗抖动滤波参数 ===
#define YAW_RATE_LPF_ALPHA     0.15f   // 角速度反馈低通滤波系数 (越小越平滑,建议0.1-0.3)
#define WZ_OUTPUT_LPF_ALPHA    0.20f   // wz输出低通滤波系数 (越小越平滑)
#define WZ_STEADY_DEADZONE     15.0f   // 稳态 wz 死区阈值 (deg/s)，低于此值认为已到位
#define WZ_Settle_AngleThreshold 2.5f   // 角度误差小于此值时才启用稳态死区 (度)

// 滤波状态变量
static float yaw_rate_lpf = 0.0f;       // 角速度反馈滤波值
static float wz_lpf = 0.0f;             // wz输出滤波值

static float WrapAngle180Deg(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 8, // 4.5
                .Ki = 0,   // 0
                .Kd = 0,   // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 设置为开环，电机设定值由下面的功率控制设定，不走普通的pid
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    //使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = PowerControlInit(&chassis_motor_config);//左前

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = PowerControlInit(&chassis_motor_config);//右前

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = PowerControlInit(&chassis_motor_config);//左后

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config);//右后

    // 裁判系统数据初始化（仅初始化数据接收，不包含UI）
    // 不接裁判系统时 referee_data 为 NULL，后续任务中会自动使用默认功率限制
    referee_data = RefereeDataInit(&huart6);

/* Buffer环暂未测试，逻辑是计算期望buffer与实际buffer的差值，转换为冗余的功率，todo：输入给功率控制部分，待完善 */
    PID_Init_Config_s Buffer_pid_conf = {
        .Kp = 0.1,
        .Ki = 0,
        .Kd = 0,
        .IntegralLimit = 1000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 1000,
    };
    PIDInit(&buffer_PID, &Buffer_pid_conf); // 缓冲能量PID初始化

    PID_Init_Config_s Angle_pid_conf = {
        .Kp = 6.0f,         // 适当减小，降低超调
        .Ki = 0.0f,
        .Kd = 4.0f,         // 减小微分，降低振荡
        .IntegralLimit = 500.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05f,  // 增强滤波
        .MaxOut = 2500.0f,  // 限制输出峰值，降低响应激进程度
        .DeadBand = 2.0f,  // 适当增大死区，减少稳态小幅振荡
    };
    PIDInit(&angle_PID, &Angle_pid_conf);

    // 角速度环 PID (内环,串级控制)
    PID_Init_Config_s YawRate_pid_conf = {
        .Kp = 35.0f,        // 略降低，使响应更柔和
        .Ki = 0.0f,
        .Kd = 2.0f,         // 略降低微分
        .IntegralLimit = 3000.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 8000.0f,  // 降低最大输出限制
        .DeadBand = 5.0f,
    };
    PIDInit(&yaw_rate_PID, &YawRate_pid_conf);

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    Chassis_IMU_data = INS_Init(); // 复用云台的IMU,底盘跟随需要用到角速度
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)


/**
 * @brief 计算每个全向轮电机的输出,正运动学解算
 *        X型布局全向轮解算公式，每个轮子与X轴成45度角
 *        公式推导：v = vx*cos(theta) + vy*sin(theta) + wz*R
 *        其中theta为轮子安装角度，R为轮子到中心距离
 */
static void OmniWheelCalculate()
{
    static const float k = 0.7071f; // sin(45°) = cos(45°) = √2/2 ≈ 0.7071

    vt_lf = (-chassis_vx - chassis_vy) * k - chassis_cmd_recv.wz * MOTOR_TO_CENTER;
    vt_rf = (-chassis_vx + chassis_vy) * k - chassis_cmd_recv.wz * MOTOR_TO_CENTER;
    vt_lb = (chassis_vx - chassis_vy) * k - chassis_cmd_recv.wz * MOTOR_TO_CENTER;
    vt_rb = (chassis_vx + chassis_vy) * k - chassis_cmd_recv.wz * MOTOR_TO_CENTER;
}
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 复活缓冲期间强制零输出，防止积分释放导致抖动
    if (chassis_revive_state == CHASSIS_REVIVE_STATE_BUFFER) {
        DJIMotorSetRef(motor_lf, 0.0f);
        DJIMotorSetRef(motor_rf, 0.0f);
        DJIMotorSetRef(motor_lb, 0.0f);
        DJIMotorSetRef(motor_rb, 0.0f);
        return;
    }

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
//static void EstimateSpeed()
//{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...

    //后续添加打滑检测的代码
//}

/* 裁判系统调试检查函数 */
static void CheckRefereeStatus()
{
    referee_check_counter++;
    if (referee_check_counter % REFEREE_CHECK_INTERVAL != 0)
        return;

    if (referee_data == NULL) {
        referee_debug.connected_flag = 0;
        referee_debug.robot_id = 0;
        referee_debug.robot_level = 0;
        referee_debug.current_hp = 0;
        referee_debug.maximum_hp = 0;
        referee_debug.power_limit = 0;
        referee_debug.heat_limit = 0;
        referee_debug.cooling_value = 0;
        referee_debug.gimbal_power_output = 0;
        referee_debug.chassis_power_output = 0;
        referee_debug.shooter_power_output = 0;
        referee_debug.chassis_voltage = 0;
        referee_debug.chassis_current = 0;
        referee_debug.chassis_power = 0.0f;
        referee_debug.buffer_energy = 0;
        referee_debug.shooter_heat = 0;
        referee_debug.shooter_17mm_2_heat = 0;
        referee_debug.shooter_42mm_heat = 0;
        referee_debug.bullet_type = 0;
        referee_debug.shooter_id = 0;
        referee_debug.bullet_freq = 0;
        referee_debug.bullet_speed = 0.0f;
        return;
    }

    // 更新调试变量（可以在 Ozone 中直接展开 referee_debug 监视）
    referee_debug.robot_id = referee_data->GameRobotState.robot_id;
    referee_debug.robot_level = referee_data->GameRobotState.robot_level;
    referee_debug.current_hp = referee_data->GameRobotState.current_HP;
    referee_debug.maximum_hp = referee_data->GameRobotState.maximum_HP;
    referee_debug.power_limit = referee_data->GameRobotState.chassis_power_limit;
    referee_debug.heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    referee_debug.cooling_value = referee_data->GameRobotState.shooter_barrel_cooling_value;
    referee_debug.gimbal_power_output = referee_data->GameRobotState.power_management_gimbal_output;
    referee_debug.chassis_power_output = referee_data->GameRobotState.power_management_chassis_output;
    referee_debug.shooter_power_output = referee_data->GameRobotState.power_management_shooter_output;
    referee_debug.chassis_voltage = referee_data->PowerHeatData.chassis_voltage;
    referee_debug.chassis_current = referee_data->PowerHeatData.chassis_current;
    referee_debug.chassis_power = referee_data->PowerHeatData.chassis_power;
    referee_debug.buffer_energy = referee_data->PowerHeatData.buffer_energy;
    referee_debug.shooter_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    referee_debug.shooter_17mm_2_heat = referee_data->PowerHeatData.shooter_17mm_2_barrel_heat;
    referee_debug.shooter_42mm_heat = referee_data->PowerHeatData.shooter_42mm_barrel_heat;
    referee_debug.bullet_type = referee_data->ShootData.bullet_type;
    referee_debug.shooter_id = referee_data->ShootData.shooter_id;
    referee_debug.bullet_freq = referee_data->ShootData.bullet_freq;
    referee_debug.bullet_speed = referee_data->ShootData.bullet_speed;
    referee_debug.connected_flag = (referee_data->GameRobotState.robot_id != 0) ? 1 : 0;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 裁判系统状态检查
    CheckRefereeStatus();

    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    // 裁判系统功率数据更新（不接裁判系统时使用默认功率限制）
    if (referee_data != NULL && referee_data->GameRobotState.robot_id != 0) {
        float power_limit = (float)referee_data->GameRobotState.chassis_power_limit;
        UpdatePowerControlRefereeData(
            referee_data->PowerHeatData.chassis_power,
            referee_data->PowerHeatData.buffer_energy,
            referee_data->GameRobotState.chassis_power_limit
        );
        SetPowerLimit(power_limit);
    } else {
        UpdatePowerControlRefereeData(0.0f, 0.0f, 0);
        SetPowerLimit(DEFAULT_POWER_LIMIT);
    }

    // === 复活检测：检测底盘输出能力从"不可输出"变为"可输出"的上升沿 ===
    static uint8_t last_chassis_output_enabled = 0;
    uint8_t current_chassis_output_enabled = IsChassisOutputEnabled();

    // 检测上升沿：从不可输出变为可输出
    if (current_chassis_output_enabled && !last_chassis_output_enabled) {
        // 检测到复活上升沿，清除所有PID状态
        ResetPIDRuntimeState(&angle_PID);
        ResetPIDRuntimeState(&yaw_rate_PID);
        ResetAllMotorPID();  // 清除四个底盘电机速度PID

        // 进入缓冲状态
        chassis_revive_state = CHASSIS_REVIVE_STATE_JUST_ENABLED;
        chassis_revive_start_tick = 0;  // 将由下一帧设置

        LOGINFO("[chassis] revive detected, clearing PID states");
    }
    last_chassis_output_enabled = current_chassis_output_enabled;

    // === 复活后零输出缓冲处理 ===
    uint8_t force_zero_output = 0;
    if (chassis_revive_state == CHASSIS_REVIVE_STATE_JUST_ENABLED ||
        chassis_revive_state == CHASSIS_REVIVE_STATE_BUFFER) {

        // 初始化起始时间戳
        if (chassis_revive_start_tick == 0) {
            chassis_revive_start_tick = 1;  // 标记为已启动
        } else if (chassis_revive_start_tick == 1) {
            chassis_revive_start_tick = 2;  // 第二帧开始计时
        } else if (chassis_revive_start_tick >= 2) {
            chassis_revive_start_tick++;

            // 缓冲时间结束，恢复正常运行
            if (chassis_revive_start_tick >= CHASSIS_REVIVE_BUFFER_CYCLES) {
                chassis_revive_state = CHASSIS_REVIVE_STATE_NORMAL;
                chassis_revive_start_tick = 0;
                LOGINFO("[chassis] revive buffer finished, resuming normal control");
            } else {
                // 缓冲期间强制零输出
                force_zero_output = 1;
                if (chassis_revive_state == CHASSIS_REVIVE_STATE_JUST_ENABLED) {
                    chassis_revive_state = CHASSIS_REVIVE_STATE_BUFFER;
                }
            }
        }
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        chassis_cmd_recv.vx = 0.0f;
        chassis_cmd_recv.vy = 0.0f;
        chassis_cmd_recv.vx_target = 0.0f;
        chassis_cmd_recv.vy_target = 0.0f;
        chassis_cmd_recv.wz = 0.0f;
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    static chassis_mode_e last_chassis_mode = (chassis_mode_e)0xff;

    // === 模式切换时重置底盘跟随 PID 状态 ===
    if (last_chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW &&
        chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        ResetPIDRuntimeState(&angle_PID);
        ResetPIDRuntimeState(&yaw_rate_PID);
        // 重置滤波器状态，防止历史值导致初始冲击
        yaw_rate_lpf = 0.0f;
        wz_lpf = 0.0f;
    }

    // 更新复活检测调试变量
    debug_chassis_revive_state = (float)chassis_revive_state;
    debug_chassis_output_enabled = (float)current_chassis_output_enabled;
    if (chassis_revive_state == CHASSIS_REVIVE_STATE_BUFFER) {
        debug_chassis_buffer_cycles = (float)(CHASSIS_REVIVE_BUFFER_CYCLES - chassis_revive_start_tick);
    } else {
        debug_chassis_buffer_cycles = 0.0f;
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台
    {
        debug_chassis_offset_recv = chassis_cmd_recv.offset_angle;
        float follow_err_deg = WrapAngle180Deg(chassis_cmd_recv.offset_angle);
        debug_chassis_follow_err = follow_err_deg;

        // 获取底盘角速度反馈 (deg/s) - Chassis_IMU_data 在 CHASSIS_BOARD 和 ONE_BOARD 下都已初始化
        float yaw_rate_raw = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE; // rad/s -> deg/s
        debug_chassis_yaw_rate_raw = yaw_rate_raw;

        // 低通滤波：平滑角速度反馈，抑制高频噪声尖峰
        yaw_rate_lpf = yaw_rate_lpf * (1.0f - YAW_RATE_LPF_ALPHA) + yaw_rate_raw * YAW_RATE_LPF_ALPHA;
        debug_chassis_yaw_rate_feedback = yaw_rate_lpf;

        // 串级PID：外环(角度环) -> 内环(角速度环)
        // 外环：角度误差 -> 角速度目标
        float yaw_rate_target = PIDCalculate(&angle_PID, 0.0f, follow_err_deg);
        debug_chassis_angle_output = yaw_rate_target;
        debug_chassis_yaw_rate_target = yaw_rate_target;

        // 内环：角速度误差 -> 最终输出 (使用滤波后的角速度反馈)
        // 按 PID 接口顺序使用角速度反馈和角速度目标
        float wz_raw = PIDCalculate(&yaw_rate_PID, yaw_rate_lpf, yaw_rate_target);
        debug_chassis_wz_raw = wz_raw;

        // 稳态死区：当角度误差很小且 wz 输出很小时，清零输出防止持续小幅振荡
        if (fabsf(follow_err_deg) < WZ_Settle_AngleThreshold && fabsf(wz_raw) < WZ_STEADY_DEADZONE) {
            wz_raw = 0.0f;
        }

        // wz 输出低通滤波：平滑 PID 输出尖峰，减少底盘抖动
        wz_lpf = wz_lpf * (1.0f - WZ_OUTPUT_LPF_ALPHA) + wz_raw * WZ_OUTPUT_LPF_ALPHA;
        chassis_cmd_recv.wz = wz_lpf;
        debug_chassis_wz_output = wz_lpf;
    }
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 10000;
        break;

    default:
        break;
    }

    last_chassis_mode = chassis_cmd_recv.chassis_mode;

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    // offset_angle 单位为角度(°)，arm_sin/cos 需要弧度(rad)
    static float sin_theta, cos_theta;
    float offset_rad = WrapAngle180Deg(chassis_cmd_recv.offset_angle) * 0.01745329252f; // deg -> rad
    if (chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        offset_rad = -offset_rad;
    }
    cos_theta = arm_cos_f32(offset_rad);
    sin_theta = arm_sin_f32(offset_rad);
    chassis_vx = chassis_cmd_recv.vx_target * cos_theta + chassis_cmd_recv.vy_target * sin_theta;
    chassis_vy = -chassis_cmd_recv.vx_target * sin_theta + chassis_cmd_recv.vy_target * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    OmniWheelCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    //EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}