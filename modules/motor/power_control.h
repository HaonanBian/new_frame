/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2. 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#include "dji_motor.h"

/**
 * @brief 裁判系统数据接口类型
 */
typedef struct {
    float chassis_power;       // 底盘实时功率
    float chassis_power_buffer; // 缓冲能量
    uint16_t chassis_power_limit; // 功率上限
} PowerControl_RefereeData_s;

DJIMotorInstance *PowerControlInit(Motor_Init_Config_s *config);

/**
 * @brief 电机功率控制,此时电机根据电机功率模型进行控制，不是直接的pid控制
 *
 * @param motor 电机实例指针
 * @param power 功率值
 */
void PowerControl(void);

/**
 * @brief 设置电机功率限制
 *
 * @param power_limit 功率限制值
 */
void SetPowerLimit(float power_limit);

/**
 * @brief 设置缓冲能量PID参数
 *
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void SetBufferPID(float kp, float ki, float kd);

/**
 * @brief 更新裁判系统功率数据
 *
 * @param power 当前底盘功率
 * @param buffer 当前缓冲能量
 * @param limit 功率上限
 */
void UpdatePowerControlRefereeData(float power, float buffer, uint16_t limit);

/**
 * @brief 获取底盘最大功率限制
 *
 * @return uint16_t 功率限制值
 */
uint16_t GetChassisPowerLimit(void);

/**
 * @brief 获取底盘当前功率
 *
 * @return float 当前功率值
 */
float GetChassisPower(void);

/**
 * @brief 获取底盘缓冲能量
 *
 * @return float 缓冲能量值
 */
float GetChassisPowerBuffer(void);

/**
 * @brief 重置所有电机PID的状态（积分项、微分项等）
 *        用于复活后清除累积的PID状态，防止积分释放导致抖动
 */
void ResetAllMotorPID(void);
#endif // !DJI_MOTOR_H
