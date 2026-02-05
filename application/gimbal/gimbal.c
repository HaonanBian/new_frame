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


static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

static float WrapRadPi(float angle)
{
    while (angle > 3.14159265359f)
        angle -= 6.28318530718f;
    while (angle < -3.14159265359f)
        angle += 6.28318530718f;
    return angle;
}

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
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
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
        },
        .motor_type = DM4310};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x01,
            .rx_id = 0x03,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 350, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
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


/* 机器人云台控制核心任务,使用MIT模式控制 */
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

        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
        gimbal_feedback_data.yaw_motor_single_round_angle = 0;
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;
    }

    const float deg2rad = 0.01745329252f;
    const float imu_yaw_rad = gimba_IMU_data->YawTotalAngle * deg2rad;
    const float imu_pitch_rad = gimba_IMU_data->Pitch * deg2rad;

    float cmd_yaw_rad = gimbal_cmd_recv.yaw * deg2rad;
    float cmd_pitch_rad = gimbal_cmd_recv.pitch * deg2rad;

    float yaw_error = WrapRadPi(cmd_yaw_rad - imu_yaw_rad);
    float pitch_error = WrapRadPi(cmd_pitch_rad - imu_pitch_rad);

    if (gimbal_cmd_recv.gimbal_mode != last_mode)
    {
        if (last_mode == GIMBAL_ZERO_FORCE)
        {
            yaw_error = 0.0f;
            pitch_error = 0.0f;
        }
        last_mode = gimbal_cmd_recv.gimbal_mode;
    }
    
    // 计算角速度参考值
    float yaw_vel_ref = yaw_error * 10.0f; // 简单的比例控制，将角度误差转换为角速度指令
    float pitch_vel_ref = pitch_error * 10.0f;
    float pitch_gravity_ff = PITCH_GRAVITY_FF_COEFF * sinf(imu_pitch_rad);
    
    // 限制角速度
    LIMIT_MIN_MAX(yaw_vel_ref, -10.0f, 10.0f);
    LIMIT_MIN_MAX(pitch_vel_ref, -20.0f, 20.0f);
    LIMIT_MIN_MAX(pitch_gravity_ff, -100.0f, 100.0f);
    // 根据控制模式进行处理
    switch (gimbal_cmd_recv.gimbal_mode)
    {
        // 停止
    case GIMBAL_ZERO_FORCE:        
        DMMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: 
    case GIMBAL_FREE_MODE: 
    {
        float yaw_target_pos = yaw_motor->measure.position + yaw_error;
        float pitch_target_pos = pitch_motor->measure.position + pitch_error;
        LIMIT_MIN_MAX(yaw_target_pos, DM_P_MIN, DM_P_MAX);
        LIMIT_MIN_MAX(pitch_target_pos, DM_P_MIN, DM_P_MAX);

        DMMotorMITCtrl(yaw_motor,
                      yaw_target_pos,
                      yaw_vel_ref,
                      5.0f,
                      0.5f,
                      0.0f);

        DMMotorMITCtrl(pitch_motor,
                      pitch_target_pos,
                      pitch_vel_ref,
                      15.0f,
                      0.5f,
                      pitch_gravity_ff);
        break;
    }
    default:
        break;
    }

    // 设置反馈数据,主要是imu和yaw电机的单圈角度
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    
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