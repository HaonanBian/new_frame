#include "gimbal.h"
#include "motor_def.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include <assert.h>
#include <math.h>
#include <string.h>

#include "force_gimbal_core.h"

// 简单便捷的调试宏
// 0: 比赛模式 (正常接收遥控器/自瞄控制)
// 1: 测量模式 (跑 Python 扫频测 J、B 参数，纯力矩正弦波)
// 2: 验证模式 (闭环位置正弦波，用来验证调好的参数稳不稳)
#define WORK_MODE  0

// 1: 测试 Yaw 轴 (达妙 4310)
// 2: 测试 Pitch 轴 (达妙 4310)
#define TEST_AXIS  1

// 斜坡规划参数
// 限制云台最大转速，防止大角度跳变时冲击控制器导致超调和抖动
#define YAW_MAX_VEL_RAD_S     15.0f   // Yaw 最大角速度 (rad/s)，约 860°/s

static float ramp_yaw_out = 0.0f;   // Yaw 斜坡输出（弧度）

static inline void Ramp_Update(float raw_target_rad, float max_vel_rad_s, float dt, float *out)
{
    float diff = raw_target_rad - *out;
    if (diff > max_vel_rad_s * dt) {
        *out += max_vel_rad_s * dt;
    } else if (diff < -max_vel_rad_s * dt) {
        *out -= max_vel_rad_s * dt;
    } else {
        *out = raw_target_rad;
    }
}

typedef struct {
    float Kp0;
    float Ki0;
    float Kd0;
    float MaxOut;
    float DeadBand;
    float e_max;
    float ec_max;
    float Kp_min;
    float Kp_max;
    float Ki_min;
    float Ki_max;
    float Kd_min;
    float Kd_max;
    float Kp_scale;
    float Ki_scale;
    float Kd_scale;
} GimbalFuzzyPID_Init_Config_s;

typedef struct {
    float KpInertiaScale;
    float KdInertiaScale;
} GimbalFuzzyPID_Load_Adapt_Config_s;

typedef struct {
    GimbalFuzzyPID_Init_Config_s InitConfig;
    GimbalFuzzyPID_Load_Adapt_Config_s LoadAdaptConfig;
    float LoadRatio;
    float BaseKp;
    float BaseKi;
    float BaseKd;
    float Kp;
    float Ki;
    float Kd;
    float e;
    float e_last;
    float Measure;
    float Last_Measure;
    float Ref;
    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Output;
    float IntegralLimit;
    uint32_t DWT_CNT;
    float dt;
} GimbalFuzzyPIDInstance;

static float GimbalFuzzyPIDClamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static void GimbalFuzzyPIDApplyLoadAdapt(GimbalFuzzyPIDInstance *gimbal_fpid)
{
    if (gimbal_fpid == NULL)
        return;

    float load_ratio = gimbal_fpid->LoadRatio;
    float kp_scale = 1.0f + gimbal_fpid->LoadAdaptConfig.KpInertiaScale * (load_ratio - 0.5f);
    float kd_scale = 1.0f + gimbal_fpid->LoadAdaptConfig.KdInertiaScale * (load_ratio - 0.5f);
    gimbal_fpid->BaseKp = gimbal_fpid->InitConfig.Kp0 * kp_scale;
    gimbal_fpid->BaseKi = gimbal_fpid->InitConfig.Ki0;
    gimbal_fpid->BaseKd = gimbal_fpid->InitConfig.Kd0 * kd_scale;

    gimbal_fpid->Kp = GimbalFuzzyPIDClamp(gimbal_fpid->BaseKp,
                                          gimbal_fpid->InitConfig.Kp_min,
                                          gimbal_fpid->InitConfig.Kp_max);
    gimbal_fpid->Ki = GimbalFuzzyPIDClamp(gimbal_fpid->BaseKi,
                                          gimbal_fpid->InitConfig.Ki_min,
                                          gimbal_fpid->InitConfig.Ki_max);
    gimbal_fpid->Kd = GimbalFuzzyPIDClamp(gimbal_fpid->BaseKd,
                                          gimbal_fpid->InitConfig.Kd_min,
                                          gimbal_fpid->InitConfig.Kd_max);
}

static void GimbalFuzzyPIDInit(GimbalFuzzyPIDInstance *gimbal_fpid, const GimbalFuzzyPID_Init_Config_s *config)
{
    if (gimbal_fpid == NULL || config == NULL)
        return;

    memset(gimbal_fpid, 0, sizeof(GimbalFuzzyPIDInstance));
    gimbal_fpid->InitConfig = *config;
    gimbal_fpid->LoadRatio = 0.5f;
    gimbal_fpid->IntegralLimit = config->MaxOut * 0.3f;
    DWT_GetDeltaT(&gimbal_fpid->DWT_CNT);
    GimbalFuzzyPIDApplyLoadAdapt(gimbal_fpid);
}

static float GimbalFuzzyPIDCalculate(GimbalFuzzyPIDInstance *gimbal_fpid, float measure, float ref)
{
    if (gimbal_fpid == NULL)
        return 0.0f;

    gimbal_fpid->Measure = measure;
    gimbal_fpid->Ref = ref;
    gimbal_fpid->dt = DWT_GetDeltaT(&gimbal_fpid->DWT_CNT);
    if (gimbal_fpid->dt <= 0.0001f)
        gimbal_fpid->dt = 0.001f;

    gimbal_fpid->e = gimbal_fpid->Ref - gimbal_fpid->Measure;

    if (fabsf(gimbal_fpid->e) > gimbal_fpid->InitConfig.DeadBand)
    {
        float dt = (gimbal_fpid->dt > 0.0001f) ? gimbal_fpid->dt : 0.0001f;
        gimbal_fpid->Pout = gimbal_fpid->Kp * gimbal_fpid->e;
        gimbal_fpid->ITerm = gimbal_fpid->Ki * gimbal_fpid->e * dt;
        gimbal_fpid->Dout = gimbal_fpid->Kd * (gimbal_fpid->Last_Measure - gimbal_fpid->Measure) / dt;
        gimbal_fpid->Iout += gimbal_fpid->ITerm;

        float temp_output = gimbal_fpid->Pout + gimbal_fpid->Iout + gimbal_fpid->Dout;
        if (fabsf(temp_output) > gimbal_fpid->InitConfig.MaxOut)
        {
            if (gimbal_fpid->e * gimbal_fpid->Iout > 0.0f)
            {
                float excess = temp_output > 0.0f
                                   ? (temp_output - gimbal_fpid->InitConfig.MaxOut)
                                   : (temp_output + gimbal_fpid->InitConfig.MaxOut);
                gimbal_fpid->Iout -= excess;
            }
        }

        if (gimbal_fpid->Iout > gimbal_fpid->IntegralLimit)
            gimbal_fpid->Iout = gimbal_fpid->IntegralLimit;
        if (gimbal_fpid->Iout < -gimbal_fpid->IntegralLimit)
            gimbal_fpid->Iout = -gimbal_fpid->IntegralLimit;

        gimbal_fpid->Output = gimbal_fpid->Pout + gimbal_fpid->Iout + gimbal_fpid->Dout;
        gimbal_fpid->Output = GimbalFuzzyPIDClamp(gimbal_fpid->Output,
                                                  -gimbal_fpid->InitConfig.MaxOut,
                                                  gimbal_fpid->InitConfig.MaxOut);
    }
    else
    {
        gimbal_fpid->Pout = 0.0f;
        gimbal_fpid->ITerm = 0.0f;
        gimbal_fpid->Dout = 0.0f;
        gimbal_fpid->Output = gimbal_fpid->Iout;
    }

    gimbal_fpid->Output = GimbalFuzzyPIDClamp(gimbal_fpid->Output,
                                              -gimbal_fpid->InitConfig.MaxOut,
                                              gimbal_fpid->InitConfig.MaxOut);
    gimbal_fpid->Last_Measure = gimbal_fpid->Measure;
    gimbal_fpid->e_last = gimbal_fpid->e;

    return gimbal_fpid->Output;
}

static void GimbalFuzzyPIDCalculateAxis(ForceAxis_t *axis, GimbalFuzzyPIDInstance *gimbal_fpid, ForceWorkMode_e mode, float t_sec)
{
    if (axis == NULL)
        return;
    if (gimbal_fpid == NULL)
    {
        ForceAxis_Calc(axis, mode, t_sec);
        return;
    }

    if (mode == MODE_DISABLE || mode == MODE_MEASURE)
    {
        ForceAxis_Calc(axis, mode, t_sec);
        return;
    }

    ForceAxis_PrepareMode(axis, mode, t_sec);
    ForceAxis_CalcVelocityControl(axis, GimbalFuzzyPIDCalculate(gimbal_fpid, axis->current_pos, axis->target_pos));
    ForceAxis_MapOutput(axis);
}

// 力控封装结构体
typedef struct {
    ForceAxis_t          axis;
    TrajectorySmoother_t smoother;
    GimbalFuzzyPIDInstance fuzzy_pid;
} ForceGimbalAxis_t;

typedef struct {
    float J;
    float B;
    float C;
    float G_cos;
    float G_sin;
    float PosKp;
    float PosKi;
    float PosKd;
    float VelKp;
    float VelKi;
    float VelKd;
    GimbalFuzzyPID_Init_Config_s FuzzyPIDConfig;
    float SmootherWn;
    float SmootherZeta;
    float SmootherDt;
} GimbalYawParamConfig_t;

static const GimbalYawParamConfig_t gimbal_yaw_param_config = {
    .J = 0.006f,
    .B = 0.025f,
    .C = 0.015f,
    .G_cos = 0.0f,
    .G_sin = 0.0f,
    .PosKp = 30.0f,
    .PosKi = 0.0f,
    .PosKd = 0.0f,
    .VelKp = 3.0f,
    .VelKi = 0.0f,
    .VelKd = 0.0f,
    .FuzzyPIDConfig = {
        .Kp0 = 28.0f,
        .Ki0 = 0.0f,
        .Kd0 = 0.0f,
        .MaxOut = 7.0f,
        .DeadBand = 0.008f,
        .e_max = 0.5f,
        .ec_max = 5.0f,
        .Kp_min = 5.0f,
        .Kp_max = 20.0f,
        .Ki_min = 0.0f,
        .Ki_max = 0.0f,
        .Kd_min = 0.0f,
        .Kd_max = 15.0f,
        .Kp_scale = 0.0f,
        .Ki_scale = 0.0f,
        .Kd_scale = 0.0f,
    },
    .SmootherWn = 45.0f,
    .SmootherZeta = 1.05f,
    .SmootherDt = 0.001f,
};

static const GimbalYawParamConfig_t gimbal_pitch_param_config = {
    .J = 0.0f,
    .B = 0.0f,
    .C = 0.0f,
    .G_cos = 0.0f,
    .G_sin = 0.0f,
    .PosKp = 25.0f,
    .PosKi = 0.0f,
    .PosKd = 0.0f,
    .VelKp = 0.5f,
    .VelKi = 0.0f,
    .VelKd = 0.0f,
    .FuzzyPIDConfig = {0},
    .SmootherWn = 35.0f,
    .SmootherZeta = 1.0f,
    .SmootherDt = 0.001f,
};

static ForceGimbalAxis_t force_yaw;
static ForceGimbalAxis_t force_pitch;

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

float gimbal_pitch_angle_deg = 0.0f;

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

static BMI088Instance *bmi088;

// DM电机状态定义 (根据DM4310协议)
#define DM_STATE_DISABLED    0x00  // 电机未使能
#define DM_STATE_ENABLED     0x01  // 电机已使能
#define DM_STATE_FAULT       0x02  // 电机故障

// 电机使能检查间隔 (ms)
#define MOTOR_ENABLE_CHECK_INTERVAL  100

static uint32_t last_enable_check_time = 0;

static inline void ResetForceAxisPIDState(ForceAxis_t *axis)
{
    axis->pid_pos.error_sum = 0.0f;
    axis->pid_pos.last_error = 0.0f;
    axis->pid_vel.error_sum = 0.0f;
    axis->pid_vel.last_error = 0.0f;
}

static inline void SyncSmootherState(TrajectorySmoother_t *smoother, float position)
{
    smoother->target_pos = position;
    smoother->out_pos = position;
    smoother->out_vel = 0.0f;
    smoother->out_acc = 0.0f;
}

/**
 * @brief 检查并恢复DM电机使能状态
 * @param yaw_motor Yaw轴电机实例
 * @param pitch_motor Pitch轴电机实例
 * @note 当检测到电机处于失能状态时，自动发送使能命令
 */
static void DMMotorEnableCheck(DMMotorInstance *yaw_motor, DMMotorInstance *pitch_motor)
{
    uint32_t current_time = HAL_GetTick();
    
    // 按固定间隔检查，避免过于频繁
    if (current_time - last_enable_check_time < MOTOR_ENABLE_CHECK_INTERVAL)
        return;
    
    last_enable_check_time = current_time;
    
    // 检查Yaw电机状态
    if (yaw_motor->measure.state != DM_STATE_ENABLED)
    {
        DMMotorSetMode(DM_CMD_MOTOR_MODE, yaw_motor);
    }
    
    // 检查Pitch电机状态
    if (pitch_motor->measure.state != DM_STATE_ENABLED)
    {
        DMMotorSetMode(DM_CMD_MOTOR_MODE, pitch_motor);
    }
}

void GimbalInit()
{
    gimba_IMU_data = INS_Init();

    // 1. 底层电机初始化 (关闭框架自带的串级 PID)
    // yaw
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x04,
            .rx_id = 0x06,
        },
        .controller_setting_init_config = {
            .close_loop_type = 0, // 关闭框架自带 PID，走纯扭矩直通
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = DM4310
    };

    //pitch
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x03,
            .rx_id = 0x06,
        },
        .controller_setting_init_config = {
            .close_loop_type = 0, // 关闭框架自带 PID
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };

    yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    // 2. 力控核心初始化

    // Yaw 轴 (达妙 4310) - 启用模糊PID 
    // 前馈参数: J, B, C (空载初始估计值，需标定)
    ForceAxis_Init(&force_yaw.axis, MOTOR_TYPE_DM_MIT, 1.0f, 
                   gimbal_yaw_param_config.J,  
                   gimbal_yaw_param_config.B,  
                   gimbal_yaw_param_config.C,  
                   gimbal_yaw_param_config.G_cos,  
                   gimbal_yaw_param_config.G_sin);  
    
    // 普通PID参数（作为模糊PID的基准值）
    ForceAxis_SetPID(&force_yaw.axis, 
                     gimbal_yaw_param_config.PosKp, gimbal_yaw_param_config.PosKi, gimbal_yaw_param_config.PosKd, 
                     gimbal_yaw_param_config.VelKp, gimbal_yaw_param_config.VelKi, gimbal_yaw_param_config.VelKd); 
    
    // 配置模糊PID参数
    GimbalFuzzyPIDInit(&force_yaw.fuzzy_pid, &gimbal_yaw_param_config.FuzzyPIDConfig);
    
    // 轨迹平滑器（稍微提高响应速度）
    Smoother_Init(&force_yaw.smoother, gimbal_yaw_param_config.SmootherWn, gimbal_yaw_param_config.SmootherZeta, gimbal_yaw_param_config.SmootherDt);
    Smoother_SetWrap(&force_yaw.smoother, 0);

    ForceAxis_Init(&force_pitch.axis, MOTOR_TYPE_DM_MIT, 1.0f,
                   gimbal_pitch_param_config.J,
                   gimbal_pitch_param_config.B,
                   gimbal_pitch_param_config.C,
                   gimbal_pitch_param_config.G_cos,
                   gimbal_pitch_param_config.G_sin);
    ForceAxis_SetPID(&force_pitch.axis,
                     gimbal_pitch_param_config.PosKp, gimbal_pitch_param_config.PosKi, gimbal_pitch_param_config.PosKd,
                     gimbal_pitch_param_config.VelKp, gimbal_pitch_param_config.VelKi, gimbal_pitch_param_config.VelKd);
    Smoother_Init(&force_pitch.smoother, gimbal_pitch_param_config.SmootherWn, gimbal_pitch_param_config.SmootherZeta, gimbal_pitch_param_config.SmootherDt);
    Smoother_SetWrap(&force_pitch.smoother, 0);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

void GimbalTask()
{
    static uint8_t first_msg_received = 0;
    static gimbal_mode_e last_mode = GIMBAL_ZERO_FORCE;

    if (SubGetMessage(gimbal_sub, &gimbal_cmd_recv))
    {
        first_msg_received = 1;
    }

    if (!first_msg_received)
    {
        DMMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        DMMotorTorqueCtrl(yaw_motor, 0);
        DMMotorTorqueCtrl(pitch_motor, 0);

        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
        gimbal_feedback_data.yaw_motor_single_round_angle = 0;
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    const float deg2rad = 0.01745329252f;
    float time_now = HAL_GetTick() / 1000.0f;

    // 1. 获取物理反馈并更新至力控核心

    // Yaw: IMU 反馈
    float imu_yaw_feedback = gimba_IMU_data->YawTotalAngle * deg2rad;
    float imu_yaw_gyro = gimba_IMU_data->Gyro[2];
    ForceAxis_UpdateFeedback(&force_yaw.axis, imu_yaw_feedback, imu_yaw_gyro);

    // Pitch: 电机编码器反馈 (由于 IMU 无法反馈 Pitch)
    float motor_pitch_feedback = pitch_motor->measure.position;
    float motor_pitch_velocity = pitch_motor->measure.velocity;
    ForceAxis_UpdateFeedback(&force_pitch.axis, motor_pitch_feedback, motor_pitch_velocity);

    // 更新全局变量供其他任务使用
    gimbal_pitch_angle_deg = motor_pitch_feedback * 57.295779513f;

    if (gimbal_cmd_recv.gimbal_mode != last_mode)
    {
        ramp_yaw_out = imu_yaw_feedback;
        SyncSmootherState(&force_pitch.smoother, motor_pitch_feedback);
        ResetForceAxisPIDState(&force_pitch.axis);
        last_mode = gimbal_cmd_recv.gimbal_mode;
    }

    // 2. 状态机与物理结算
    switch (gimbal_cmd_recv.gimbal_mode)
    {
        case GIMBAL_ZERO_FORCE:
            DMMotorStop(yaw_motor);
            DMMotorStop(pitch_motor);
            DMMotorTorqueCtrl(yaw_motor, 0);
            DMMotorTorqueCtrl(pitch_motor, 0);

            // 同步斜坡输出，防止切回时跳变
            ramp_yaw_out   = imu_yaw_feedback;
            SyncSmootherState(&force_pitch.smoother, motor_pitch_feedback);
            ResetForceAxisPIDState(&force_pitch.axis);
            break;

        case GIMBAL_GYRO_MODE:
        case GIMBAL_FREE_MODE:
        {
            DMMotorEnable(yaw_motor);
            DMMotorEnable(pitch_motor);

            // 电机使能状态检查与恢复
            DMMotorEnableCheck(yaw_motor, pitch_motor);

            ForceWorkMode_e yaw_mode = MODE_COMPETITION;
            ForceWorkMode_e pitch_mode = MODE_COMPETITION;

            // 获取上位机原始目标 (度转弧度)
            float raw_yaw_target = gimbal_cmd_recv.yaw * deg2rad;
            float raw_pitch_target = gimbal_cmd_recv.pitch;

            // 测试模式劫持
            if (WORK_MODE > 0) {
                if (TEST_AXIS == 1) {
                    yaw_mode = (ForceWorkMode_e)WORK_MODE;
                    raw_yaw_target = 0.0f;
                }
                else if (TEST_AXIS == 2) {
                    pitch_mode = (ForceWorkMode_e)WORK_MODE;
                    raw_pitch_target = 0.0f;
                }
            }

            // 斜坡限速  — 限制最大角速度，防止大角度跳变冲击控制器
            float dt_ms = 1.0f; // 1ms 控制周期
            Ramp_Update(raw_yaw_target,   YAW_MAX_VEL_RAD_S,   dt_ms * 0.001f, &ramp_yaw_out);

            // 斜坡输出直接送入控制器 
            ForceAxis_SetTarget(&force_yaw.axis,   ramp_yaw_out,   0.0f, 0.0f);
            Smoother_Update(&force_pitch.smoother, raw_pitch_target);
            ForceAxis_SetTarget(&force_pitch.axis, force_pitch.smoother.out_pos, force_pitch.smoother.out_vel, force_pitch.smoother.out_acc);

            GimbalFuzzyPIDCalculateAxis(&force_yaw.axis, &force_yaw.fuzzy_pid, yaw_mode, time_now);
            ForceAxis_Calc(&force_pitch.axis, pitch_mode, time_now);

            float final_yaw_torque = force_yaw.axis.total_torque;
            float final_pitch_torque = force_pitch.axis.total_torque;

            if (yaw_motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) final_yaw_torque *= -1;
            if (pitch_motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) final_pitch_torque *= -1;
            LIMIT_MIN_MAX(final_pitch_torque, DM_T_MIN, DM_T_MAX);

            DMMotorTorqueCtrl(yaw_motor, final_yaw_torque);
            DMMotorTorqueCtrl(pitch_motor, final_pitch_torque);
            break;
        }
        default:
            break;
    }

    // 4. 数据回传逻辑
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.pitch_motor_position = motor_pitch_feedback;

    float yaw_position = yaw_motor->measure.position;
    float yaw_single_round_rad = yaw_position;
    while (yaw_single_round_rad < 0.0f)
        yaw_single_round_rad += 6.28318530718f;
    while (yaw_single_round_rad >= 6.28318530718f)
        yaw_single_round_rad -= 6.28318530718f;
    float yaw_single_round_deg = yaw_single_round_rad * 57.2957795131f;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)(yaw_single_round_deg * 182.044444444f);

    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
