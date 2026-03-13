#include "gimbal.h"
#include "motor_def.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include <assert.h>
#include <math.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x04,
            .rx_id = 0x06,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 1.0f,  // 力矩控制模式下需要重新调参
                .Ki = 0.1f,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5,
                .MaxOut = 10,  // DM电机力矩范围[-10, 10]
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = DM4310};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x03,
            .rx_id = 0x06,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 6, // 10->5 降低增益减少抖动

                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.01,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 0.58f,
                .Ki = 0.07f,
                .Kd = 0,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_OutputFilter,
                .IntegralLimit = 8,
                .Output_LPF_RC = 0.008f,
                .MaxOut = 10,  // DM电机力矩范围[-10, 10]
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DMMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 将角度转换为-π到π的范围
 * @param angle 输入角度
 * @return 转换后的角度
 */


/* 机器人云台控制核心任务,使用力矩控制模式(与basic_framework-master一致) */
void GimbalTask()
{
    // 获取云台控制数据，检查是否收到新消息
    static uint8_t first_msg_received = 0;
    static gimbal_mode_e last_mode = GIMBAL_ZERO_FORCE;
    static float yaw_pid_ref = 0;   // yaw角度环设定值
    static float pitch_pid_ref = 0; // pitch角度环设定值
    
    if (SubGetMessage(gimbal_sub, &gimbal_cmd_recv))
    {
        first_msg_received = 1;
    }
    
    // 如果还没收到过消息，保持停止状态
    if (!first_msg_received)
    {
        DMMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        DMMotorTorqueCtrl(yaw_motor, 0);
        DMMotorTorqueCtrl(pitch_motor, 0);
        // 仍然需要推送反馈数据
        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
        gimbal_feedback_data.yaw_motor_single_round_angle = 0;
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    const float deg2rad = 0.01745329252f;
    
    // 获取IMU反馈数据(角度制转弧度制)
    float imu_yaw_feedback = gimba_IMU_data->YawTotalAngle * deg2rad;   // 陀螺仪yaw角度反馈(rad)
    float imu_yaw_gyro = gimba_IMU_data->Gyro[2];                       // 陀螺仪yaw角速度反馈(rad/s)
    // pitch轴使用电机编码器反馈(IMU装在yaw轴上,无法反馈pitch)
    float motor_pitch_feedback = pitch_motor->measure.position;          // 电机位置反馈(rad)
    float motor_pitch_velocity = pitch_motor->measure.velocity;          // 电机速度反馈(rad/s)
    static float pitch_vel_fdb_lpf = 0.0f;
    const float pitch_vel_lpf_alpha = 0.20f;
    pitch_vel_fdb_lpf += pitch_vel_lpf_alpha * (motor_pitch_velocity - pitch_vel_fdb_lpf);

    // 模式切换时初始化pid_ref，避免突变
    if (gimbal_cmd_recv.gimbal_mode != last_mode)
    {
        yaw_pid_ref = imu_yaw_feedback;
        pitch_pid_ref = motor_pitch_feedback;
        gimbal_cmd_recv.pitch = motor_pitch_feedback;
        last_mode = gimbal_cmd_recv.gimbal_mode;
    }
    
    // 根据控制模式进行处理
    switch (gimbal_cmd_recv.gimbal_mode)
    {
        // 停止
    case GIMBAL_ZERO_FORCE:        
        DMMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        DMMotorTorqueCtrl(yaw_motor, 0);
        DMMotorTorqueCtrl(pitch_motor, 0);
        break;
        
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: 
    case GIMBAL_FREE_MODE: 
    {
        DMMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);
        
        // 更新角度设定值
        yaw_pid_ref = gimbal_cmd_recv.yaw * deg2rad;
        pitch_pid_ref = gimbal_cmd_recv.pitch;  // pitch已经是弧度(由cmd直接累加弧度增量)
        
        // ============ YAW轴串级PID控制(角度环→速度环→力矩输出) ============
        // 角度环：输入为角度目标值，用陀螺仪的角度，输出为速度设定值
        float yaw_speed_ref = PIDCalculate(&yaw_motor->angle_PID, imu_yaw_feedback, yaw_pid_ref);
        LIMIT_MIN_MAX(yaw_speed_ref, DM_V_MIN, DM_V_MAX);
        // 速度环：输入为速度环pid值，用陀螺仪角速度，输出为力矩
        float yaw_torque = PIDCalculate(&yaw_motor->speed_PID, imu_yaw_gyro, yaw_speed_ref);
        if (yaw_motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            yaw_torque *= -1;//反转
        LIMIT_MIN_MAX(yaw_torque, DM_T_MIN, DM_T_MAX);
        // 发送力矩控制指令
        DMMotorTorqueCtrl(yaw_motor, yaw_torque);
        
        // ============ PITCH轴串级PID控制(角度环→速度环→力矩输出) ============
        // pitch软件限位: 最低0.0455586rad, 最高-1.223rad
        #define PITCH_MIN_RAD (-1.223f)   // 最高位置(向上)
        #define PITCH_MAX_RAD (0.0455586f) // 最低位置(向下)
        LIMIT_MIN_MAX(pitch_pid_ref, PITCH_MIN_RAD, PITCH_MAX_RAD);
        float pitch_speed_ref = PIDCalculate(&pitch_motor->angle_PID, motor_pitch_feedback, pitch_pid_ref);
        float pitch_pos_err = pitch_pid_ref - motor_pitch_feedback;
        if (fabsf(pitch_pos_err) < 0.005f && fabsf(pitch_vel_fdb_lpf) < 0.12f)
        {
            float near_factor = fabsf(pitch_pos_err) / 0.005f;
            if (near_factor < 0.5f)
                near_factor = 0.5f;
            pitch_speed_ref *= near_factor;
        }
        LIMIT_MIN_MAX(pitch_speed_ref, DM_V_MIN, DM_V_MAX);
        
        float pitch_torque = PIDCalculate(&pitch_motor->speed_PID, pitch_vel_fdb_lpf, pitch_speed_ref);
        
        // 重力补偿前馈（已注释）
        // pitch_torque += PitchGravityCompensation(motor_pitch_feedback);

        if (pitch_motor->motor_settings.feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pitch_torque *= -1;
        // 限幅
        LIMIT_MIN_MAX(pitch_torque, DM_T_MIN, DM_T_MAX);
        // 发送力矩控制指令
        DMMotorTorqueCtrl(pitch_motor, pitch_torque);
        //可以mit的速度和位置有点问题，不是很建议使用，mit就是用力矩好了
        break;
    }
    default:
        break;
    }

    // 设置反馈数据,主要是imu和yaw电机的单圈角度
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.pitch_motor_position = pitch_motor->measure.position;
    
    // 计算yaw电机的单圈角度
    // DM_Motor_Measure_s结构体中的position字段范围是[-2π, 2π]弧度
    float yaw_position = yaw_motor->measure.position;
    
    // 将位置转换为单圈角度(0-2π弧度)
    float yaw_single_round_rad = yaw_position;
    while (yaw_single_round_rad < 0) yaw_single_round_rad += 6.28318530718f;
    while (yaw_single_round_rad >= 6.28318530718f) yaw_single_round_rad -= 6.28318530718f;
    
    // 转换为角度值(0-360度)
    float yaw_single_round_deg = yaw_single_round_rad * 57.2957795131f;
    
    // 转换为uint16_t类型,范围0-65535,对应0-360度
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)(yaw_single_round_deg * 182.044444444f); // 65535 / 360 = 182.044444444

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}