/**
 * @file fuzzy_pid.h
 * @brief 模糊PID控制器定义 (适配版)
 * @version beta
 * @date 2026-04-03
 */
#ifndef _FUZZY_PID_H
#define _FUZZY_PID_H

#include "stdint.h"
#include <math.h>
#include "stm32f407xx.h"  // 必须在 arm_math.h 之前包含，定义 __FPU_PRESENT
#include "arm_math.h"
#include "lowpass_filter.h"
#include "kalman_filter.h"
#include "bsp_dwt.h"
#include "user_lib.h"

#define FUZZY_TABLE_SIZE   7      // 模糊规则表大小 (7x7)
#define FUZZY_L            3.0f   // 归一化论域半径

typedef enum
{
    FUZZYPID_IMPROVE_NONE                 = 0u,
    FUZZYPID_Integral_Limit               = 1u << 0,
    FUZZYPID_Derivative_On_Measurement    = 1u << 1,
    FUZZYPID_Trapezoid_Intergral          = 1u << 2,
    FUZZYPID_Proportional_On_Measurement  = 1u << 3,
    FUZZYPID_OutputFilter                 = 1u << 4,
    FUZZYPID_ChangingIntegrationRate      = 1u << 5,
    FUZZYPID_Derivative_Filter            = 1u << 6,
    FUZZYPID_ErrorHandle                  = 1u << 7,
    FUZZYPID_Integral_Filter               = 1u << 8,
} FUZZYPID_Improvement_e;

typedef enum
{
   TRIANGULAR=0b000001,
   TRAPEZOIDAL=0b000010,
   GAUSSIAN=0b000100,
   SIGMOID=0b001000,
   BELL_SHAPED=0b010000,
   CUSTOM =0b100000,
} FUZZYPID_function_e;

typedef enum {
    FUZZYPID_DEFUZZ_WEIGHTED_AVERAGE = 0,
    FUZZYPID_DEFUZZ_CENTROID
} FUZZYPID_DefuzzMethod_e;

typedef enum f_errorType_e
{
    FUZZYPID_ERROR_NONE = 0x00U,
    FUZZYPID_MOTOR_BLOCKED_ERROR = 0x01U
} FUZZYPID_ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    FUZZYPID_ErrorType_e ERRORType;
} FUZZYPIDErrorHandler_t;

typedef enum {
   NB = 0,
   NM,
   NS,
   ZO,
   PS,
   PM,
   PB
} FuzzySet_e;

typedef struct {
    // PID原始参数
    float Kp0;
    float Ki0;
    float Kd0;

    float Kp;
    float Ki;
    float Kd;

    float MaxOut;
    float DeadBand;

    // 归一化参数
    float e_max;
    float ec_max;

    // 模糊调整限制
    float Kp_min, Kp_max;
    float Ki_min, Ki_max;
    float Kd_min, Kd_max;

    // 模糊规则表
    FuzzySet_e Kp_rule_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE];
    FuzzySet_e Ki_rule_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE];
    FuzzySet_e Kd_rule_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE];

    // 调整比例系数
    float Kp_scale, Ki_scale, Kd_scale;

    // 实时数据
    float e, e_norm;
    float ec, ec_norm;
    float e_last;

    float Delta_Kp, Delta_Ki, Delta_Kd;

    float Measure, Last_Measure, Last_ITerm;
    float Pout, Iout, Dout, ITerm;
    float Output, Last_Output, Last_Dout;
    float Ref;

    FUZZYPID_Improvement_e Improve;
    FUZZYPID_function_e  FuzzyFunction;
    FUZZYPID_DefuzzMethod_e DefuzzMethod;

    float CoefA, CoefB;
    float Output_LPF_RC, Derivative_LPF_RC, Integral_LPF_RC;
    float IntegralLimit;

    LowPassFilter_t Output_LPF;
    LowPassFilter_t Derivative_LPF;
    LowPassFilter_t Integral_LPF;

    uint8_t Enabled;
    FUZZYPIDErrorHandler_t ErrorHandler;
    KalmanFilter_t *Kalman;

    float KF_X, KF_P, KF_Q, KF_R, KF_K;

    uint32_t DWT_CNT;
    float dt;
} FuzzyPIDInstance_t;

typedef FuzzyPIDInstance_t FuzzyPIDInstance;

typedef struct {
    float Kp0, Ki0, Kd0;
    float e_max, ec_max;
    float Kp_min, Kp_max, Ki_min, Ki_max, Kd_min, Kd_max;
    float Kp_scale, Ki_scale, Kd_scale;
    float MaxOut, DeadBand;
    FUZZYPID_Improvement_e Improve;
    FUZZYPID_function_e  FuzzyFunction;
    FUZZYPID_DefuzzMethod_e DefuzzMethod;
    float CoefA, CoefB;
    float Output_LPF_RC, Derivative_LPF_RC, Integral_LPF_RC;
    float IntegralLimit;
    KalmanFilter_t *Kalman;
    uint8_t Enabled;
} FuzzyPID_Init_Config_s;

void FuzzyPIDInit(FuzzyPIDInstance *fpid, FuzzyPID_Init_Config_s *config);
float FuzzyPIDCalculate(FuzzyPIDInstance *fpid, float measure, float ref);
float DJI2006FuzzyPIDCalculate(FuzzyPIDInstance *fpid, float measure, float ref);
float DMFuzzyPIDCalculate(FuzzyPIDInstance *fpid, float measure, float ref);
void FuzzyPIDReset(FuzzyPIDInstance *fpid);
void FuzzyPIDSetEnable(FuzzyPIDInstance *fpid, uint8_t enable);
void FuzzyPIDUpdateParams(FuzzyPIDInstance *fpid, float kp, float ki, float kd);
void FuzzyPIDAdjust(FuzzyPIDInstance *fpid, float measure, float ref);
void FuzzyPIDGetCurrentParams(FuzzyPIDInstance *fpid, float *kp, float *ki, float *kd);
void FuzzyPIDLoadDefaultRules(FuzzyPIDInstance *fpid);
void FuzzyPIDLoadCustomRules(FuzzyPIDInstance *fpid,
                             FuzzySet_e kp_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE],
                             FuzzySet_e ki_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE],
                             FuzzySet_e kd_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE],
                             uint8_t *sign);

#endif /* _FUZZY_PID_H */
