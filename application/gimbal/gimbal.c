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


void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,
            .rx_id = 0x03,
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

/* 机器人云台控制核心任务,使用MIT模式控制 */
void GimbalTask()
{
    // 获取云台控制数据
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // 计算当前姿态与目标姿态的误差
    float yaw_error = gimbal_cmd_recv.yaw - gimba_IMU_data->YawTotalAngle;
    float pitch_error = gimbal_cmd_recv.pitch - gimba_IMU_data->Pitch;
    
    // 计算角速度参考值
    float yaw_vel_ref = yaw_error * 10.0f; // 简单的比例控制，将角度误差转换为角速度指令
    float pitch_vel_ref = pitch_error * 10.0f;
    float pitch_gravity_ff = PITCH_GRAVITY_FF_COEFF * sinf(gimba_IMU_data->Pitch);
    
    // 限制角速度
    LIMIT_MIN_MAX(yaw_vel_ref, -20.0f, 20.0f);
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
        // 使用MIT模式直接控制电机
        // 参数说明：位置指令、速度指令、Kp、Kd、力矩指令
        DMMotorMITCtrl(yaw_motor, 
                      gimbal_cmd_recv.yaw,     // 目标位置
                      yaw_vel_ref,              // 目标速度
                      10.0f,                    // Kp
                      0.5f,                     // Kd
                      0.0f);                    // 力矩前馈
        
        DMMotorMITCtrl(pitch_motor, 
                      gimbal_cmd_recv.pitch,    // 目标位置
                      pitch_vel_ref,            // 目标速度
                      15.0f,                    // Kp
                      0.5f,                     // Kd
                      pitch_gravity_ff);       // 力矩前馈
        break;
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