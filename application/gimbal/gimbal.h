#ifndef GIMBAL_H
#define GIMBAL_H

#define PITCH_GRAVITY_FF_COEFF 120.0f

/**
 * @brief 全局变量：Pitch轴角度(度)，供其他任务读取
 */
extern float gimbal_pitch_angle_deg;

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H