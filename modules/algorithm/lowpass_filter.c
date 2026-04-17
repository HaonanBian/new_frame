/**
 ******************************************************************************
 * @file    lowpass_filter.c
 * @brief   一阶低通滤波器实现
 ******************************************************************************
 */

#include "lowpass_filter.h"

void LowPassFilter_Init_ByFreq(LowPassFilter_t *lpf, float fs, float fc)
{
    lpf->lpf_rc = 1.0f / (2.0f * 3.1415926f * fc);
    lpf->input = 0.0f;
    lpf->output = 0.0f;
}

void LowPassFilter_Init(LowPassFilter_t *lpf, float rc)
{
    lpf->lpf_rc = rc;
    lpf->input = 0.0f;
    lpf->output = 0.0f;
}

float LowPassFilter_Update(LowPassFilter_t *lpf, float input)
{
    lpf->input = input;
    lpf->output = lpf->output + (input - lpf->output) * (1.0f / (1.0f + lpf->lpf_rc));
    return lpf->output;
}
