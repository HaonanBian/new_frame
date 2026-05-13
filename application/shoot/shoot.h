#ifndef SHOOT_H
#define SHOOT_H

#include "robot_def.h"

/**
 * @brief 发射机构宏定义
 */
#define BULLET_HEAT_ESTIMATE 10          // 单发子弹预估热量
#define HEAT_SAFE_MARGIN 20              // 热量安全余量
#define HEAT_BURST_SLOWDOWN_REST 80
#define HEAT_BURST_SINGLE_REST 50
#define HEAT_BURST_SLOW_RATE 4.0f
// #define MAX_BULLET_COUNT    50           // 机器人最大载弹量，根据实际情况修改

#define FRIC_SPEED_PID_KP        20.0f
#define FRIC_SPEED_PID_KI        1.0f
#define FRIC_SPEED_PID_KD        0.0f
#define FRIC_PID_MAX_OUT         15000.0f
#define FRIC_PID_MAX_IOUT        10000.0f

#define BUFFER_PID_KP            0.1f
#define BUFFER_PID_KI            0.0f
#define BUFFER_PID_KD            0.0f
#define BUFFER_PID_MAX_OUT       1000.0f

#define BUFFER_TARGET_VALUE      30.0f    // 缓冲能量目标值

/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

/**
 * @brief 获取当前枪口热量限制状态
 * @return heat_limit_status_e 热量限制状态
 */
heat_limit_status_e GetHeatLimitStatus(void);

/**
 * @brief 获取当前剩余弹量 (无裁判系统时使用)
 * @return int16_t 当前剩余弹量
 */
int16_t GetBulletCount(void);

/**
 * @brief 补弹
 * @param count 补弹数量，<=0 则装满 MAX_BULLET_COUNT
 */
void ReloadBullets(int16_t count);

/**
 * @brief 强制重置弹量为满弹
 */
void ResetBulletCount(void);

#endif // SHOOT_H