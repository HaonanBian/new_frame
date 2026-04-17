/**
 ******************************************************************************
 * @file    lowpass_filter.h
 * @brief   一阶低通滤波器
 ******************************************************************************
 */

#ifndef _LOWPASS_FILTER_H
#define _LOWPASS_FILTER_H

#include <stdint.h>
#include <math.h>

/**
 * @brief 一阶低通滤波器结构体
 */
typedef struct {
    float input;        // 当前输入
    float output;       // 当前输出
    float lpf_rc;       // RC时间常数 RC = 1 / omega_c
} LowPassFilter_t;

/**
 * @brief 通过截止频率初始化低通滤波器
 * @param lpf 滤波器实例指针
 * @param fs  采样频率 (Hz)
 * @param fc  截止频率 (Hz)
 */
void LowPassFilter_Init_ByFreq(LowPassFilter_t *lpf, float fs, float fc);

/**
 * @brief 通过RC时间常数初始化低通滤波器
 * @param lpf 滤波器实例指针
 * @param rc  RC时间常数
 */
void LowPassFilter_Init(LowPassFilter_t *lpf, float rc);

/**
 * @brief 更新低通滤波器
 * @param lpf 滤波器实例指针
 * @param input 新的输入值
 * @return float 滤波后的输出值
 */
float LowPassFilter_Update(LowPassFilter_t *lpf, float input);

#endif /* _LOWPASS_FILTER_H */
