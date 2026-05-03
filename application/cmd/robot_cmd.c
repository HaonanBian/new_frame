                                                                                                                    // app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "usart.h"
#include <math.h>

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE ((float)YAW_CHASSIS_ALIGN_ECD) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE ((float)PITCH_HORIZON_ECD) // pitch水平时电机的角度,0-360

// 调试数据结构体：集中管理所有需要观测的变量
typedef struct
{
    // === Yaw 电机原始反馈链路 ===
    float yaw_motor_raw;           // 云台回传的原始 yaw_motor_single_round_angle (uint16)
    float yaw_motor_deg;           // 转换后的角度 0-360°
    float yaw_motor_signed;        // 转换为 ±180° 的有符号角
    float yaw_align_angle;         // 对齐角 (YAW_ALIGN_ANGLE)
    
    // === IMU 原始反馈链路 ===
    float imu_yaw_total;           // IMU 的 YawTotalAngle (多圈累计)
    float imu_yaw_single;          // IMU 的 Yaw (单圈 ±180°)
    float imu_yaw_total_rad;       // IMU YawTotalAngle 转弧度
    float imu_gyro_z;              // IMU Z轴角速度 (rad/s)
    
    // === 云台控制目标链路 ===
    float gimbal_yaw_target;       // 云台 yaw 目标角 (度)
    float gimbal_yaw_target_rad;   // 云台 yaw 目标角 (弧度)
    float gimbal_yaw_target_err;   // 云台目标角误差 (target - actual) 度
    float gimbal_yaw_target_err_rad; // 云台目标角误差 (target - actual) 弧度
    float gimbal_mode;             // 云台模式
    
    // === 角度转换中间过程 ===
    float angle_before_wrap;       // Wrap 前的角度差
    float angle_after_wrap;        // Wrap 后的角度差
    float yaw_motor_to_imu_diff;   // 电机角与 IMU 角的差值
    float imu_yaw_init_offset;     // IMU 初始对齐偏移量
    
    // === 底盘跟随链路 ===
    float offset_relative;         // 计算出的相对角 (应该在 ±180° 内)
    float chassis_mode;            // 当前底盘模式
    float chassis_follow_err;      // 底盘跟随时实际使用的误差角
    float chassis_offset_angle;    // 发送给底盘的 offset_angle
    float chassis_wz;              // 底盘旋转速度输出
    
    // === 潜在漂移源追踪 ===
    float yaw_motor_raw_delta;     // yaw_motor_raw 的变化量
    float imu_yaw_total_delta;     // imu_yaw_total 的变化量
    float gimbal_target_delta;     // gimbal_yaw_target 的变化量
    float offset_angle_delta;      // offset_angle 的变化量
    
    // === 统计计数器 ===
    uint32_t calc_offset_count;    // CalcOffsetAngle 调用次数
    uint32_t mode_switch_count;    // 模式切换次数
    uint32_t rotate_to_follow_sync_count; // 从小陀螺切到底盘跟随的同步次数

    // === 切换瞬间抓拍 ===
    float rotate_to_follow_imu_total;     // 切换瞬间 IMU 累计角
    float rotate_to_follow_motor_deg;     // 切换瞬间电机角(0-360)
    float rotate_to_follow_offset;        // 切换瞬间 offset
    float rotate_to_follow_target_yaw;    // 切换后目标 yaw

    // === 状态有效性标志 ===
    float follow_mode_valid;              // 1:底盘跟随+云台GYRO, 0:无效采样
    
    // === 旧调试变量 (兼容) ===
    float tast_angle;
    float tast_angle_signed;
    float tast_delta;
    float tast_angle_total;
} RobotCmdDebugData_t;

// 全局调试数据实例
RobotCmdDebugData_t cmd_debug_data = {0};

// 为了兼容现有代码和 Ozone 调试，保留原变量名作为宏别名
#define debug_yaw_motor_raw         (cmd_debug_data.yaw_motor_raw)
#define debug_yaw_motor_deg         (cmd_debug_data.yaw_motor_deg)
#define debug_yaw_motor_signed      (cmd_debug_data.yaw_motor_signed)
#define debug_yaw_align_angle       (cmd_debug_data.yaw_align_angle)
#define debug_offset_relative       (cmd_debug_data.offset_relative)
#define debug_imu_yaw_total         (cmd_debug_data.imu_yaw_total)
#define debug_imu_yaw_single        (cmd_debug_data.imu_yaw_single)
#define debug_chassis_mode          (cmd_debug_data.chassis_mode)
#define debug_follow_err            (cmd_debug_data.chassis_follow_err)
#define debug_gimbal_yaw_target     (cmd_debug_data.gimbal_yaw_target)
#define debug_gimbal_yaw_target_err (cmd_debug_data.gimbal_yaw_target_err)
#define debug_gimbal_mode           (cmd_debug_data.gimbal_mode)
#define tast_angle                  (cmd_debug_data.tast_angle)
#define tast_angle_signed           (cmd_debug_data.tast_angle_signed)
#define tast_delta                  (cmd_debug_data.tast_delta)
#define tast_angle_total            (cmd_debug_data.tast_angle_total)

// 新增调试变量的快捷访问
#define debug_imu_yaw_total_rad     (cmd_debug_data.imu_yaw_total_rad)
#define debug_gimbal_yaw_target_rad (cmd_debug_data.gimbal_yaw_target_rad)
#define debug_angle_before_wrap     (cmd_debug_data.angle_before_wrap)
#define debug_angle_after_wrap      (cmd_debug_data.angle_after_wrap)
//
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

static uint8_t last_control_mode = 0; // 0=未知, 1=遥控器, 2=键鼠
#define CONTROL_MODE_RC 1
#define CONTROL_MODE_KB 2
#define CHASSIS_VEL_ACCEL_STEP 600.0f
#define CHASSIS_VEL_DECEL_STEP 45.0f
#define KB_CHASSIS_MOVE_VALUE 5000.0f
#define PITCH_RC_DEADBAND 5
#define RAD2DEG 57.295779513f
// #define MOUSE_DEADBAND 2
// #define MOUSE_MAX_DELTA 240
// #define MOUSE_REJECT_DELTA 2000
#define MOUSE_DEADBAND 4
#define MOUSE_MAX_DELTA 120
#define MOUSE_REJECT_DELTA 2000

static float RampFollow(float input, float current, float accel_step, float decel_step)
{
    float delta = input - current;
    float step = decel_step;
    if (current == 0.0f || current * delta > 0.0f)
        step = accel_step;
    if (delta > step)
        delta = step;
    else if (delta < -step)
        delta = -step;
    return current + delta;
}

static int16_t MouseDeltaFilter(int16_t raw_delta)
{
    int16_t abs_delta = (raw_delta >= 0) ? raw_delta : (int16_t)(-raw_delta);

    if (abs_delta <= MOUSE_DEADBAND)
        return 0;

    if (abs_delta >= MOUSE_REJECT_DELTA)
        return 0;

    if (raw_delta > MOUSE_MAX_DELTA)
        return MOUSE_MAX_DELTA;
    if (raw_delta < -MOUSE_MAX_DELTA)
        return -MOUSE_MAX_DELTA;

    return raw_delta;
}

static void UpdateChassisVelTarget(void)
{
    chassis_cmd_send.vx_target = RampFollow(chassis_cmd_send.vx, chassis_cmd_send.vx_target, CHASSIS_VEL_ACCEL_STEP, CHASSIS_VEL_DECEL_STEP);
    chassis_cmd_send.vy_target = RampFollow(chassis_cmd_send.vy, chassis_cmd_send.vy_target, CHASSIS_VEL_ACCEL_STEP, CHASSIS_VEL_DECEL_STEP);
}

// 底盘角度跟随滤波参数
#define CHASSIS_ANGLE_FILTER_K    0.3f   // 角度滤波系数，越小越平滑

// 全局标志：请求重新校准底盘对齐基准
static uint8_t g_recalibrate_chassis_alignment = 0;
static float chassis_offset_filtered = 0.0f;  // 滤波后的底盘偏移角度

static void ClearChassisMotionCommand(void)
{
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.vx_target = 0.0f;
    chassis_cmd_send.vy_target = 0.0f;
    chassis_cmd_send.wz = 0.0f;
}

static void RequestOffsetAngleReset(void)
{
    chassis_offset_filtered = chassis_cmd_send.offset_angle;
}

static float WrapAngle180Deg(float angle_deg)
{
    while (angle_deg > 180.0f)
        angle_deg -= 360.0f;
    while (angle_deg < -180.0f)
        angle_deg += 360.0f;
    return angle_deg;
}

static void SyncGimbalTargetToCurrent(void)
{
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_motor_position;
}

// 在模式切换时重新校准 IMU-电机对应关系（补偿 IMU 漂移）
static void RecalibrateIMUMotorAlignment(void)
{
    // 设置标志，让 CalcOffsetAngle 在下次调用时重新记录 IMU 和电机的对应关系
    g_recalibrate_chassis_alignment = 1;
}

void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    last_control_mode = 0;
    shoot_cmd_send.shoot_mode = SHOOT_OFF; // 初始化为关闭,等待控制函数开启
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.lid_mode = LID_CLOSE;
    shoot_cmd_send.bullet_speed = (Bullet_Speed_e)22;
    shoot_cmd_send.shoot_rate = 0;
    chassis_cmd_send.chassis_power_limit = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 使用电机单圈角度计算 offset_angle
    // 记录 IMU 和电机的对应关系，在模式切换时重新校准
    static float imu_yaw_at_align = 0.0f;  // 记录对齐时的 IMU 角度
    static float motor_yaw_at_align = 0.0f; // 记录对齐时的电机角度
    static uint8_t alignment_initialized = 0;
    
    // 第一次调用或者请求重新校准时，记录当前 IMU 和电机的对应关系
    if (!alignment_initialized || g_recalibrate_chassis_alignment)
    {
        float motor_angle = ((float)gimbal_fetch_data.yaw_motor_single_round_angle) / 182.044444444f;
        float motor_angle_signed = motor_angle;
        if (motor_angle_signed > 180.0f)
            motor_angle_signed -= 360.0f;
        
        motor_yaw_at_align = motor_angle_signed;
        imu_yaw_at_align = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        alignment_initialized = 1;
        g_recalibrate_chassis_alignment = 0;  // 清除标志
    }
    
    // 使用电机角度计算 offset_angle（原有逻辑）
    static float angle;
    angle = ((float)gimbal_fetch_data.yaw_motor_single_round_angle) / 182.044444444f; // 0-360 deg

    float angle_signed = angle;
    if (angle_signed > 180.0f)
        angle_signed -= 360.0f;
    
   float yaw_align_signed = YAW_ALIGN_ANGLE;
    if (yaw_align_signed > 180.0f)
        yaw_align_signed -= 360.0f;

    float angle_diff_before_wrap = angle_signed - yaw_align_signed;
    float relative_angle = WrapAngle180Deg(angle_diff_before_wrap);

    chassis_cmd_send.offset_angle = relative_angle;
    
    // === 更新调试数据结构体 ===
    
    // 保存上一次的值用于计算 delta
    static float last_yaw_motor_raw = 0;
    static float last_imu_yaw_total = 0;
    static float last_offset_angle = 0;
    
    // Yaw 电机反馈链路
    debug_yaw_motor_raw = (float)gimbal_fetch_data.yaw_motor_single_round_angle;
    debug_yaw_motor_deg = angle;
    debug_yaw_motor_signed = angle_signed;
    debug_yaw_align_angle = yaw_align_signed;
    
    // IMU 反馈链路
    debug_imu_yaw_total = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    debug_imu_yaw_single = gimbal_fetch_data.gimbal_imu_data.Yaw;
    cmd_debug_data.imu_yaw_total_rad = debug_imu_yaw_total * 0.01745329252f;
    cmd_debug_data.imu_gyro_z = gimbal_fetch_data.gimbal_imu_data.Gyro[2];
    
    // 角度转换中间过程
    cmd_debug_data.angle_before_wrap = angle_diff_before_wrap;
    cmd_debug_data.angle_after_wrap = relative_angle;
    
    // 计算 IMU 相对于初始对齐的漂移量
    float imu_drift = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle - imu_yaw_at_align;
    float motor_drift = angle_signed - motor_yaw_at_align;
    cmd_debug_data.yaw_motor_to_imu_diff = imu_drift - motor_drift;  // IMU 和电机的漂移差异
    cmd_debug_data.imu_yaw_init_offset = imu_yaw_at_align;  // 记录对齐基准 
    
    // 底盘跟随链路
    debug_offset_relative = relative_angle;
    debug_chassis_mode = (float)chassis_cmd_send.chassis_mode;
    debug_follow_err = relative_angle;
    cmd_debug_data.chassis_offset_angle = chassis_cmd_send.offset_angle;
    cmd_debug_data.follow_mode_valid = (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW &&
                                        gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) ? 1.0f : 0.0f;
    
    // 计算变化量 (delta)
    cmd_debug_data.yaw_motor_raw_delta = debug_yaw_motor_raw - last_yaw_motor_raw;
    cmd_debug_data.imu_yaw_total_delta = debug_imu_yaw_total - last_imu_yaw_total;
    cmd_debug_data.offset_angle_delta = relative_angle - last_offset_angle;
    
    // 更新上一次的值
    last_yaw_motor_raw = debug_yaw_motor_raw;
    last_imu_yaw_total = debug_imu_yaw_total;
    last_offset_angle = relative_angle;
    
    // 调用计数器
    cmd_debug_data.calc_offset_count++;
    
    // 兼容旧变量
    tast_angle = angle;
    tast_angle_signed = angle_signed;
    tast_delta = relative_angle;
    tast_angle_total = relative_angle;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    static gimbal_mode_e last_gimbal_mode = (gimbal_mode_e)0xff;
    static chassis_mode_e last_chassis_mode = (chassis_mode_e)0xff;
    gimbal_mode_e next_gimbal_mode = gimbal_cmd_send.gimbal_mode;
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        next_gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],小陀螺模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        next_gimbal_mode = GIMBAL_GYRO_MODE;
    }

    if (last_chassis_mode == CHASSIS_ROTATE &&
        chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        RequestOffsetAngleReset();
        RecalibrateIMUMotorAlignment();  // 重新校准 IMU-电机对应关系，补偿小陀螺期间的 IMU 漂移
        cmd_debug_data.rotate_to_follow_sync_count++;
        cmd_debug_data.rotate_to_follow_imu_total = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        cmd_debug_data.rotate_to_follow_motor_deg = ((float)gimbal_fetch_data.yaw_motor_single_round_angle) / 182.044444444f;
        cmd_debug_data.rotate_to_follow_offset = chassis_cmd_send.offset_angle;
        cmd_debug_data.rotate_to_follow_target_yaw = gimbal_cmd_send.yaw;
        cmd_debug_data.mode_switch_count++;  // 记录模式切换
    }
    
    // 记录其他模式切换
    if (last_chassis_mode != chassis_cmd_send.chassis_mode && last_chassis_mode != (chassis_mode_e)0xff)
    {
        cmd_debug_data.mode_switch_count++;
    }
    last_chassis_mode = chassis_cmd_send.chassis_mode;

    if (next_gimbal_mode != last_gimbal_mode)
    {
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_motor_position;
        last_gimbal_mode = next_gimbal_mode;
    }
    gimbal_cmd_send.gimbal_mode = next_gimbal_mode;

    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中],视觉模式 + 底盘失能 + 摩擦轮开启
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        // 仅在视觉有新数据时累加diff,防止同一diff在200Hz任务中被重复累加
        if (vision_recv_data->data_updated)
        {
            if (vision_recv_data->control &&
                isfinite(vision_recv_data->yaw) &&
                isfinite(vision_recv_data->pitch))
            {
                gimbal_cmd_send.yaw += vision_recv_data->yaw * RAD2DEG;
                gimbal_cmd_send.pitch += vision_recv_data->pitch;
            }
            VisionDataConsumed();
        }
        else
        {
            int16_t yaw_ch = rc_data[TEMP].rc.rocker_l_;
            int16_t pitch_ch = rc_data[TEMP].rc.rocker_l1;

            if (yaw_ch < 3 && yaw_ch > -3)
                yaw_ch = 0;
            if (pitch_ch < PITCH_RC_DEADBAND && pitch_ch > -PITCH_RC_DEADBAND)
                pitch_ch = 0;

            gimbal_cmd_send.yaw -= 0.001f * (float)yaw_ch;
            gimbal_cmd_send.pitch -= 0.00005f * (float)pitch_ch;
        }
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left))
    { // 左侧开关状态为[下],底盘使能 + 摩擦轮关闭,按摇杆输出进行角度增量
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        int16_t yaw_ch = rc_data[TEMP].rc.rocker_l_;
        int16_t pitch_ch = rc_data[TEMP].rc.rocker_l1;

        if (yaw_ch < 3 && yaw_ch > -3)
            yaw_ch = 0;
        if (pitch_ch < PITCH_RC_DEADBAND && pitch_ch > -PITCH_RC_DEADBAND)
            pitch_ch = 0;

        gimbal_cmd_send.yaw -= 0.0015f * (float)yaw_ch;
        gimbal_cmd_send.pitch -= 0.00005f * (float)pitch_ch;
    }

    // 云台软件限位
    LIMIT_MIN_MAX(gimbal_cmd_send.pitch, PITCH_MIN_RAD, PITCH_MAX_RAD);

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_; // 前后(底盘解算中vx=前后)
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1; // 左右(底盘解算中vy=左右)
    chassis_cmd_send.chassis_power_limit = 0;
    if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE)
        ClearChassisMotionCommand();

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    else {
        {
            // 弹舱舵机控制,待添加servo_motor模块,关闭
        };
    }

    // 拨弹控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开拨弹盘
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 8;
    shoot_cmd_send.bullet_speed = (Bullet_Speed_e)22;  // 固定弹速22m/s
    shoot_cmd_send.shoot_mode = SHOOT_ON; // 开启发射系统
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    static chassis_mode_e last_chassis_mode = (chassis_mode_e)0xff;
    int16_t mouse_x = MouseDeltaFilter(rc_data[TEMP].mouse.x);
    int16_t mouse_y = MouseDeltaFilter(rc_data[TEMP].mouse.y);
    chassis_mode_e next_chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

    // 设置底盘和云台模式
    if (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键切换底盘模式
        next_chassis_mode = CHASSIS_ROTATE;

    chassis_cmd_send.chassis_mode = next_chassis_mode;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    if (last_chassis_mode == CHASSIS_ROTATE &&
        chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        RequestOffsetAngleReset();
        RecalibrateIMUMotorAlignment();
        chassis_offset_filtered = chassis_cmd_send.offset_angle;  // 同步滤波器当前值
        cmd_debug_data.rotate_to_follow_sync_count++;
        cmd_debug_data.rotate_to_follow_imu_total = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        cmd_debug_data.rotate_to_follow_motor_deg = ((float)gimbal_fetch_data.yaw_motor_single_round_angle) / 182.044444444f;
        cmd_debug_data.rotate_to_follow_offset = chassis_cmd_send.offset_angle;
        cmd_debug_data.rotate_to_follow_target_yaw = gimbal_cmd_send.yaw;
    }

    if (last_chassis_mode != chassis_cmd_send.chassis_mode && last_chassis_mode != (chassis_mode_e)0xff)
    {
        cmd_debug_data.mode_switch_count++;
    }
    last_chassis_mode = chassis_cmd_send.chassis_mode;

    int16_t mouse_pitch_delta = mouse_y;
    if (mouse_pitch_delta < MOUSE_DEADBAND && mouse_pitch_delta > -MOUSE_DEADBAND)
        mouse_pitch_delta = 0;
    gimbal_cmd_send.yaw -= (float)mouse_x / 660 * 9;  // 系数待测
    gimbal_cmd_send.pitch -= (float)mouse_pitch_delta / 660 * 0.4; // 系数待测,与摇杆方向一致(向上鼠标/摇杆都是减小)
    LIMIT_MIN_MAX(gimbal_cmd_send.pitch, PITCH_MIN_RAD, PITCH_MAX_RAD);

    shoot_cmd_send.bullet_speed = (Bullet_Speed_e)22; // 固定弹速22m/s
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) // Z键设置发射模式(单发/连发)
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) // V键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    // Q键开摩擦轮, E键关摩擦轮
    if (rc_data[TEMP].key[KEY_PRESS].q)
        shoot_cmd_send.friction_mode = FRICTION_ON;
    if (rc_data[TEMP].key[KEY_PRESS].e)
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    chassis_cmd_send.chassis_power_limit = 0;
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * KB_CHASSIS_MOVE_VALUE - rc_data[TEMP].key[KEY_PRESS].d * KB_CHASSIS_MOVE_VALUE; // 前后
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * KB_CHASSIS_MOVE_VALUE - rc_data[TEMP].key[KEY_PRESS].s * KB_CHASSIS_MOVE_VALUE; // 左右

    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }

    // 鼠标左键按住时拨弹盘开启
    if (rc_data[TEMP].mouse.press_l)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;

    shoot_cmd_send.shoot_rate = 8;  // 拨弹射频与遥控器一致(8颗/秒)
    shoot_cmd_send.shoot_mode = SHOOT_ON; // 开启发射系统
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 急停状态下仅允许在拨轮回到安全范围且右侧开关拨到[上]时恢复
    if (robot_state == ROBOT_STOP)
    {
        if (switch_is_up(rc_data[TEMP].rc.switch_right) && rc_data[TEMP].rc.dial <= 300)
        {
            robot_state = ROBOT_READY;
            ClearChassisMotionCommand();
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            LOGINFO("[CMD] reinstate, robot ready");
        }
        else
        {
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            ClearChassisMotionCommand();
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;
        }
        return;
    }

    // 拨轮向下超过阈值进入急停
    if (rc_data[TEMP].rc.dial > 300) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        ClearChassisMotionCommand();
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    static gimbal_mode_e last_sent_gimbal_mode = GIMBAL_ZERO_FORCE;
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    // 判断当前控制模式
    uint8_t current_control_mode = 0;
    if (switch_is_down(rc_data[TEMP].rc.switch_left))
        current_control_mode = CONTROL_MODE_RC;
    else if (switch_is_up(rc_data[TEMP].rc.switch_left))
        current_control_mode = CONTROL_MODE_KB;
    else
        current_control_mode = CONTROL_MODE_RC;

    // 检测模式切换,切换时同步云台角度
    if (current_control_mode != last_control_mode)
    {
        SyncGimbalTargetToCurrent();
        last_control_mode = current_control_mode;
    }
    
    // 确保gimbal_cmd_send.yaw被初始化(防止首次从0累加)
    if (last_control_mode == 0)
    {
        SyncGimbalTargetToCurrent();
        last_control_mode = current_control_mode;
    }

    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[中],视觉模式
        RemoteControlSet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    if (last_sent_gimbal_mode == GIMBAL_ZERO_FORCE &&
        gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE)
    {
        /* 与当前电机位置对齐，避免目标为 0 rad 而实际不在零位时产生巨大误差导致剧烈抖动 */
        gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_motor_position;
    }
    LIMIT_MIN_MAX(gimbal_cmd_send.pitch, PITCH_MIN_RAD, PITCH_MAX_RAD);
    last_sent_gimbal_mode = gimbal_cmd_send.gimbal_mode;
    UpdateChassisVelTarget();
    
    // 更新云台目标相关调试变量
    static float last_gimbal_yaw_target = 0;
    
    debug_gimbal_yaw_target = gimbal_cmd_send.yaw;
    debug_gimbal_yaw_target_err = gimbal_cmd_send.yaw - gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    debug_gimbal_mode = (float)gimbal_cmd_send.gimbal_mode;
    
    cmd_debug_data.gimbal_yaw_target_rad = gimbal_cmd_send.yaw * 0.01745329252f;
    cmd_debug_data.gimbal_yaw_target_err_rad = debug_gimbal_yaw_target_err * 0.01745329252f;
    cmd_debug_data.gimbal_target_delta = gimbal_cmd_send.yaw - last_gimbal_yaw_target;
    cmd_debug_data.chassis_wz = chassis_cmd_send.wz;
    
    last_gimbal_yaw_target = gimbal_cmd_send.yaw;

    // 更新视觉发送数据（仅在机器人正常工作时更新，实际发送由DecodeVision回调触发）
    if (robot_state == ROBOT_READY)
    {
        static float vision_q[4];
        Vision_Mode_e vision_mode = switch_is_mid(rc_data[TEMP].rc.switch_left) ? VISION_MODE_AUTO_AIM : VISION_MODE_IDLE;

        uint16_t vision_bullet_count = 0u;
        if (shoot_fetch_data.bullet_count > 0)
        {
            vision_bullet_count = (uint16_t)shoot_fetch_data.bullet_count;
        }

        VisionSetStatus(vision_mode, (float)shoot_cmd_send.bullet_speed, vision_bullet_count);
        EularAngleToQuaternion(gimbal_fetch_data.gimbal_imu_data.Yaw,
                               gimbal_fetch_data.gimbal_imu_data.Pitch,
                               gimbal_fetch_data.gimbal_imu_data.Roll,
                               vision_q);
        VisionSetQuaternion(vision_q);

        #define DEG2RAD_VISION 0.01745329252f
        VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle * DEG2RAD_VISION,
                          gimbal_fetch_data.pitch_motor_position,
                          gimbal_fetch_data.gimbal_imu_data.Gyro[2],
                          gimbal_fetch_data.gimbal_imu_data.Gyro[1]);
    }

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
