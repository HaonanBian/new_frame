/**
 * @file fuzzy_pid.c
 * @brief 模糊PID控制器实现 (适配版)
 */

#include "fuzzy_pid.h"
#include <string.h>
#include <stdint.h>

// --- PID 优化环节实现 ---
static void f_Trapezoid_Intergral(FuzzyPIDInstance *fpid)
{
    fpid->ITerm = fpid->Ki * ((fpid->e + fpid->e_last) / 2) * fpid->dt;
}

static void f_Changing_Integration_Rate(FuzzyPIDInstance *fpid)
{
    if (fpid->e * fpid->Iout > 0) {
        if (fabsf(fpid->e) <= fpid->CoefB)
            return;
        if (fabsf(fpid->e) <= (fpid->CoefA + fpid->CoefB))
            fpid->ITerm *= (fpid->CoefA - fabsf(fpid->e) + fpid->CoefB) / fpid->CoefA;
        else
            fpid->ITerm = 0;
    }
}

static void f_Integral_Limit(FuzzyPIDInstance *fpid)
{
    fpid->Iout += fpid->ITerm;

    float temp_Output = fpid->Pout + fpid->Iout + fpid->Dout;
    if (fabsf(temp_Output) > fpid->MaxOut) {
        if (fpid->e * fpid->Iout > 0) {
            float excess = temp_Output > 0 ? (temp_Output - fpid->MaxOut) : (temp_Output + fpid->MaxOut);
            fpid->Iout -= excess;
        }
    }
    if (fpid->Iout > fpid->IntegralLimit) {
        fpid->Iout = fpid->IntegralLimit;
    }
    if (fpid->Iout < -fpid->IntegralLimit) {
        fpid->Iout = -fpid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(FuzzyPIDInstance *fpid)
{
    fpid->Dout = fpid->Kd * (fpid->Last_Measure - fpid->Measure) / fpid->dt;
}

static void f_Derivative_Filter(FuzzyPIDInstance *fpid)
{
    fpid->Dout = LowPassFilter_Update(&fpid->Derivative_LPF, fpid->Dout);
}

static void f_Integral_Filter(FuzzyPIDInstance *fpid)
{
    fpid->Iout = LowPassFilter_Update(&fpid->Integral_LPF, fpid->Iout);
}

static void f_Output_Filter(FuzzyPIDInstance *fpid)
{
    fpid->Output = LowPassFilter_Update(&fpid->Output_LPF, fpid->Output);
}

static void f_PID_ErrorHandle(FuzzyPIDInstance *fpid)
{
    if (fabsf(fpid->Output) < fpid->MaxOut * 0.001f || fabsf(fpid->Ref) < 0.0001f)
        return;
    if ((fabsf(fpid->Ref - fpid->Measure) / fabsf(fpid->Ref)) > 0.95f) {
        fpid->ErrorHandler.ERRORCount++;
    } else {
        fpid->ErrorHandler.ERRORCount = 0;
    }
    if (fpid->ErrorHandler.ERRORCount > 500) {
        fpid->ErrorHandler.ERRORType = FUZZYPID_MOTOR_BLOCKED_ERROR;
    }
}

// --- 内部函数声明 ---
static float f_Linear_Normalize(float value, float max_abs);
static float f_Fuzzy_Membership_Triangle(float norm_value, FuzzySet_e set);
static float f_Fuzzy_Membership_Trapezoidal(float norm_value, FuzzySet_e set);
static float f_Fuzzy_Membership_Gaussian(float norm_value, FuzzySet_e set);
static float f_Fuzzy_Membership_sigmoid(float norm_value, FuzzySet_e set);
static float f_Fuzzy_Membership_Bell(float norm_value, FuzzySet_e set);
static float f_Fuzzy_Inference(FuzzyPIDInstance *fpid, FuzzySet_e (*rule_table)[FUZZY_TABLE_SIZE], float scale);
static void  f_Update_PID_Params(FuzzyPIDInstance *fpid);

static float f_Fuzzy_Membership(float norm_value, FuzzySet_e set, FUZZYPID_function_e function)
{
    if (function & TRIANGULAR)
        return f_Fuzzy_Membership_Triangle(norm_value, set);
    if (function & TRAPEZOIDAL)
        return f_Fuzzy_Membership_Trapezoidal(norm_value, set);
    if (function & GAUSSIAN)
        return f_Fuzzy_Membership_Gaussian(norm_value, set);
    if (function & SIGMOID)
        return f_Fuzzy_Membership_sigmoid(norm_value, set);
    if (function & BELL_SHAPED)
        return f_Fuzzy_Membership_Bell(norm_value, set);
    return f_Fuzzy_Membership_Triangle(norm_value, set);
}

// --- 模糊规则表 ---
static const FuzzySet_e DEFAULT_KP_RULES[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE] = {
    /* e\ec:  NB    NM    NS    ZO    PS    PM    PB */
    /* NB */ {PB,   PB,   PM,   PM,   PS,   ZO,   ZO},
    /* NM */ {PB,   PB,   PM,   PS,   PS,   ZO,   ZO},
    /* NS */ {PM,   PM,   PM,   PS,   ZO,   NS,   NM},
    /* ZO */ {PM,   PS,   PS,   ZO,   NS,   NS,   NM},
    /* PS */ {PM,   PS,   ZO,   NS,   NM,   NM,   NM},
    /* PM */ {ZO,   ZO,   NS,   NS,   NM,   NB,   NB},
    /* PB */ {ZO,   ZO,   NM,   NM,   NM,   NB,   NB}
};

static const FuzzySet_e DEFAULT_KI_RULES[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE] = {
    /* NB */ {NB,   NB,   NB,   NM,   NM,   ZO,   ZO},
    /* NM */ {NB,   NB,   NM,   NM,   NS,   ZO,   ZO},
    /* NS */ {NB,   NM,   NS,   NS,   ZO,   PS,   PS},
    /* ZO */ {NM,   NS,   NS,   ZO,   PS,   PS,   PM},
    /* PS */ {NM,   NM,   ZO,   PS,   PS,   PM,   PB},
    /* PM */ {ZO,   ZO,   PS,   PM,   PM,   PB,   PB},
    /* PB */ {ZO,   ZO,   PM,   PM,   PB,   PB,   PB}
};

static const FuzzySet_e DEFAULT_KD_RULES[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE] = {
    /* NB */ {PB,   PB,   PM,   PS,   ZO,   NS,   ZO},
    /* NM */ {PB,   PM,   PM,   PS,   ZO,   NS,   ZO},
    /* NS */ {PM,   PM,   PS,   ZO,   NS,   NS,   ZO},
    /* ZO */ {PB,   PM,   PS,   ZO,   PS,   PM,   PB},
    /* PS */ {ZO,   NS,   NS,   ZO,   PS,   PM,   PM},
    /* PM */ {ZO,   NS,   ZO,   PS,   PM,   PM,   PB},
    /* PB */ {ZO,   NS,   ZO,   PS,   PM,   PB,   PB}
};

// --- 内部函数实现 ---
static float f_Linear_Normalize(float value, float max_abs)
{
    if (max_abs <= 0.0001f)
        return 0.0f;
    const float x_min = -max_abs;
    const float x_max =  max_abs;
    const float y_min = -FUZZY_L;
    const float y_max =  FUZZY_L;
    if (value < x_min)  value = x_min;
    else if (value > x_max) value = x_max;
    if (fabsf(x_max - x_min) < 1e-6f)
        return 0.0f;
    return y_min + (value - x_min) * (y_max - y_min) / (x_max - x_min);
}

static float f_Fuzzy_Membership_Triangle(float norm_value, FuzzySet_e set)
{
    typedef struct { float left, center, right; } TriangleMF;
    static const TriangleMF membership_funcs[] = {
        { -3.0f, -3.0f, -2.0f },
        { -3.0f, -2.0f, -1.0f },
        { -2.0f, -1.0f,  0.0f },
        { -1.0f,  0.0f,  1.0f },
        {  0.0f,  1.0f,  2.0f },
        {  1.0f,  2.0f,  3.0f },
        {  2.0f,  3.0f,  3.0f }
    };
    if ((int)set < 0 || set >= FUZZY_TABLE_SIZE)
        return 0.0f;
    const TriangleMF *mf = &membership_funcs[set];
    float degree = 0.0f;
    if (norm_value < mf->left || norm_value > mf->right) {
        degree = 0.0f;
    } else if (norm_value <= mf->center) {
        degree = (mf->center - mf->left <= 0.0001f) ? 1.0f
            : (norm_value - mf->left) / (mf->center - mf->left);
    } else {
        degree = (mf->right - mf->center <= 0.0001f) ? 1.0f
            : (mf->right - norm_value) / (mf->right - mf->center);
    }
    return degree;
}

static float f_Fuzzy_Membership_Trapezoidal(float norm_value, FuzzySet_e set)
{
    typedef struct { float left, left_high, right_high, right; } TrapezoidalMF;
    static const TrapezoidalMF membership_funcs[] = {
        { -3.0f, -3.0f, -2.0f, -1.0f },
        { -2.5f, -2.0f, -1.0f,  0.0f },
        { -1.5f, -1.0f,  0.0f,  1.0f },
        { -1.0f, -0.5f,  0.5f,  1.0f },
        { -1.0f,  0.0f,  1.0f,  1.5f },
        {  0.0f,  1.0f,  2.0f,  2.5f },
        {  1.0f,  2.0f,  3.0f,  3.0f }
    };
    if ((int)set < 0 || set >= FUZZY_TABLE_SIZE)
        return 0.0f;
    const TrapezoidalMF *mf = &membership_funcs[set];
    float degree = 0.0f;
    if (norm_value < mf->left || norm_value > mf->right) {
        degree = 0.0f;
    } else if (norm_value >= mf->left_high && norm_value <= mf->right_high) {
        degree = 1.0f;
    } else if (norm_value < mf->left_high) {
        degree = (mf->left_high - mf->left <= 0.0001f) ? 1.0f
            : (norm_value - mf->left) / (mf->left_high - mf->left);
    } else {
        degree = (mf->right - mf->right_high <= 0.0001f) ? 1.0f
            : (mf->right - norm_value) / (mf->right - mf->right_high);
    }
    return degree;
}

static float f_Fuzzy_Membership_Gaussian(float norm_value, FuzzySet_e set)
{
    typedef struct { float mean, sigma; } GaussianMF;
    static const GaussianMF membership_funcs[] = {
        { -3.0f, 0.5f }, { -2.0f, 0.5f }, { -1.0f, 0.5f },
        {  0.0f, 0.5f }, {  1.0f, 0.5f }, {  2.0f, 0.5f }, {  3.0f, 0.5f }
    };
    const GaussianMF *mf = &membership_funcs[set];
    return expf(-0.5f * powf((norm_value - mf->mean) / mf->sigma, 2));
}

static float f_Fuzzy_Membership_sigmoid(float norm_value, FuzzySet_e set)
{
    typedef struct { float a, c; } SigmoidMF;
    static const SigmoidMF membership_funcs[] = {
        { -5.0f, -2.0f }, { -5.0f, -1.0f }, { -5.0f,  0.0f },
        {  5.0f,  0.0f }, {  5.0f,  1.0f }, {  5.0f,  2.0f }, {  5.0f,  3.0f }
    };
    const SigmoidMF *mf = &membership_funcs[set];
    return 1.0f / (1.0f + expf(-mf->a * (norm_value - mf->c)));
}

static float f_Fuzzy_Membership_Bell(float norm_value, FuzzySet_e set)
{
    typedef struct { float a, b, c; } BellMF;
    static const BellMF membership_funcs[] = {
        { 1.0f, 2.0f, -3.0f }, { 1.0f, 2.0f, -2.0f }, { 1.0f, 2.0f, -1.0f },
        { 1.0f, 2.0f,  0.0f }, { 1.0f, 2.0f,  1.0f }, { 1.0f, 2.0f,  2.0f }, { 1.0f, 2.0f,  3.0f }
    };
    const BellMF *mf = &membership_funcs[set];
    return 1.0f / (1.0f + powf(fabsf((norm_value - mf->c) / mf->a), 2 * mf->b));
}

static float f_Fuzzy_Inference(FuzzyPIDInstance *fpid, FuzzySet_e (*rule_table)[FUZZY_TABLE_SIZE], float scale)
{
    float deg_e[FUZZY_TABLE_SIZE];
    float deg_ec[FUZZY_TABLE_SIZE];
    for (int i = 0; i < FUZZY_TABLE_SIZE; i++) {
        deg_e[i]  = f_Fuzzy_Membership(fpid->e_norm,  (FuzzySet_e)i, fpid->FuzzyFunction);
        deg_ec[i] = f_Fuzzy_Membership(fpid->ec_norm, (FuzzySet_e)i, fpid->FuzzyFunction);
    }

    float out_degree[FUZZY_TABLE_SIZE];
    for (int k = 0; k < FUZZY_TABLE_SIZE; k++) out_degree[k] = 0.0f;

    for (int i = 0; i < FUZZY_TABLE_SIZE; i++) {
        for (int j = 0; j < FUZZY_TABLE_SIZE; j++) {
            float strength = deg_e[i] < deg_ec[j] ? deg_e[i] : deg_ec[j];
            if (strength <= 0.0f) continue;
            FuzzySet_e out_set = rule_table[i][j];
            if ((int)out_set < 0 || out_set >= FUZZY_TABLE_SIZE) continue;
            if (strength > out_degree[out_set]) out_degree[out_set] = strength;
        }
    }

    float max_out_deg = 0.0f;
    for (int k = 0; k < FUZZY_TABLE_SIZE; k++)
        if (out_degree[k] > max_out_deg) max_out_deg = out_degree[k];
    if (max_out_deg <= 1e-6f)
        return 0.0f;

    if (fpid->DefuzzMethod == FUZZYPID_DEFUZZ_WEIGHTED_AVERAGE) {
        static const float centers[FUZZY_TABLE_SIZE] = { -3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f };
        float numerator = 0.0f, denominator = 0.0f;
        for (int k = 0; k < FUZZY_TABLE_SIZE; k++) {
            numerator   += out_degree[k] * centers[k];
            denominator += out_degree[k];
        }
        if (denominator <= 1e-6f) return 0.0f;
        return (numerator / denominator) * scale;
    }

    const int SAMPLES = 31;
    const float x_min = -FUZZY_L;
    const float x_max =  FUZZY_L;
    const float step = (x_max - x_min) / (SAMPLES - 1);
    float numerator = 0.0f, denominator = 0.0f;
    for (int s = 0; s < SAMPLES; s++) {
        float x = x_min + s * step;
        float agg_mu = 0.0f;
        for (int k = 0; k < FUZZY_TABLE_SIZE; k++) {
            float mu_k = f_Fuzzy_Membership(x, (FuzzySet_e)k, fpid->FuzzyFunction);
            float clipped = out_degree[k] < mu_k ? out_degree[k] : mu_k;
            if (clipped > agg_mu) agg_mu = clipped;
        }
        if (agg_mu > 0.0f) {
            numerator   += x * agg_mu;
            denominator += agg_mu;
        }
    }
    if (denominator <= 1e-6f) return 0.0f;
    return (numerator / denominator) * scale;
}

static void f_Update_PID_Params(FuzzyPIDInstance *fpid)
{
    fpid->Delta_Kp = f_Fuzzy_Inference(fpid, fpid->Kp_rule_table, fpid->Kp_scale);
    fpid->Kp = fpid->Kp0 + fpid->Delta_Kp;
    if (fpid->Kp < fpid->Kp_min) fpid->Kp = fpid->Kp_min;
    if (fpid->Kp > fpid->Kp_max) fpid->Kp = fpid->Kp_max;

    fpid->Delta_Ki = f_Fuzzy_Inference(fpid, fpid->Ki_rule_table, fpid->Ki_scale);
    fpid->Ki = fpid->Ki0 + fpid->Delta_Ki;
    if (fpid->Ki < fpid->Ki_min) fpid->Ki = fpid->Ki_min;
    if (fpid->Ki > fpid->Ki_max) fpid->Ki = fpid->Ki_max;

    fpid->Delta_Kd = f_Fuzzy_Inference(fpid, fpid->Kd_rule_table, fpid->Kd_scale);
    fpid->Kd = fpid->Kd0 + fpid->Delta_Kd;
    if (fpid->Kd < fpid->Kd_min) fpid->Kd = fpid->Kd_min;
    if (fpid->Kd > fpid->Kd_max) fpid->Kd = fpid->Kd_max;
}

// --- 外部接口 ---
void FuzzyPIDInit(FuzzyPIDInstance *fpid, FuzzyPID_Init_Config_s *config)
{
    if (!fpid || !config) return;
    memset(fpid, 0, sizeof(FuzzyPIDInstance));

    fpid->Kp0 = config->Kp0;
    fpid->Ki0 = config->Ki0;
    fpid->Kd0 = config->Kd0;
    fpid->e_max  = config->e_max;
    fpid->ec_max = config->ec_max;
    fpid->Kp_min = config->Kp_min;  fpid->Kp_max = config->Kp_max;
    fpid->Ki_min = config->Ki_min;  fpid->Ki_max = config->Ki_max;
    fpid->Kd_min = config->Kd_min;  fpid->Kd_max = config->Kd_max;
    fpid->Kp_scale = config->Kp_scale;
    fpid->Ki_scale = config->Ki_scale;
    fpid->Kd_scale = config->Kd_scale;
    fpid->MaxOut   = config->MaxOut;
    fpid->DeadBand = config->DeadBand;
    fpid->Improve  = config->Improve;
    fpid->CoefA    = config->CoefA;
    fpid->CoefB    = config->CoefB;
    fpid->Output_LPF_RC     = config->Output_LPF_RC;
    fpid->Derivative_LPF_RC = config->Derivative_LPF_RC;
    fpid->Integral_LPF_RC   = config->Integral_LPF_RC;
    fpid->IntegralLimit    = config->IntegralLimit;
    LowPassFilter_Init(&fpid->Output_LPF, fpid->Output_LPF_RC);
    LowPassFilter_Init(&fpid->Derivative_LPF, fpid->Derivative_LPF_RC);
    LowPassFilter_Init(&fpid->Integral_LPF, fpid->Integral_LPF_RC);
    fpid->FuzzyFunction = config->FuzzyFunction;
    fpid->DefuzzMethod  = config->DefuzzMethod;
    fpid->Kalman   = config->Kalman;
    fpid->Enabled  = config->Enabled;
    fpid->Kp = fpid->Kp0;
    fpid->Ki = fpid->Ki0;
    fpid->Kd = fpid->Kd0;

    fpid->Last_Measure = 0.0f;
    fpid->Last_Output  = 0.0f;
    fpid->Last_Dout    = 0.0f;
    fpid->e_last       = 0.0f;
    fpid->Last_ITerm   = 0.0f;
    fpid->Iout = 0.0f; fpid->Output = 0.0f;
    fpid->Pout = 0.0f; fpid->Dout = 0.0f; fpid->ITerm = 0.0f;
    fpid->e = 0.0f; fpid->e_norm = 0.0f;
    fpid->ec = 0.0f; fpid->ec_norm = 0.0f;
    fpid->Delta_Kp = 0.0f; fpid->Delta_Ki = 0.0f; fpid->Delta_Kd = 0.0f;
    fpid->Measure = 0.0f; fpid->Ref = 0.0f;
    fpid->dt = 0.0f;

    FuzzyPIDLoadDefaultRules(fpid);
    DWT_GetDeltaT(&fpid->DWT_CNT);
}

void FuzzyPIDAdjust(FuzzyPIDInstance *fpid, float measure, float ref)
{
    if (!fpid)
        return;
    fpid->Measure = measure;
    fpid->Ref = ref;
    fpid->dt = DWT_GetDeltaT(&fpid->DWT_CNT);
    if (fpid->dt <= 0.0001f)
        fpid->dt = 0.001f;
    fpid->e = fpid->Ref - fpid->Measure;
    fpid->ec = (fpid->e - fpid->e_last) / fpid->dt;
    fpid->e_norm  = f_Linear_Normalize(fpid->e,  fpid->e_max);
    fpid->ec_norm = f_Linear_Normalize(fpid->ec, fpid->ec_max);
    if (!fpid->Enabled)
        return;
    f_Update_PID_Params(fpid);
}

float FuzzyPIDCalculate(FuzzyPIDInstance *fpid, float measure, float ref)
{
    if (!fpid)
        return 0.0f;
    if (fpid->Improve & FUZZYPID_ErrorHandle)
        f_PID_ErrorHandle(fpid);
    FuzzyPIDAdjust(fpid, measure, ref);

    if (fabsf(fpid->e) > fpid->DeadBand) {
        float dt = (fpid->dt > 0.0001f) ? fpid->dt : 0.0001f;
        fpid->Pout = fpid->Kp * fpid->e;
        fpid->ITerm = fpid->Ki * fpid->e * dt;
        fpid->Dout = fpid->Kd * (fpid->e - fpid->e_last) / dt;

        if (fpid->Improve & FUZZYPID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(fpid);
        if (fpid->Improve & FUZZYPID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(fpid);
        if (fpid->Improve & FUZZYPID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(fpid);
        if (fpid->Improve & FUZZYPID_Derivative_Filter)
            f_Derivative_Filter(fpid);
        if (fpid->Improve & FUZZYPID_Integral_Limit)
            f_Integral_Limit(fpid);
        else
            fpid->Iout += fpid->ITerm;
        if (fpid->Improve & FUZZYPID_Integral_Filter)
            f_Integral_Filter(fpid);
        fpid->Output = fpid->Pout + fpid->Iout + fpid->Dout;

        if (fpid->Improve & FUZZYPID_OutputFilter)
            f_Output_Filter(fpid);

        fpid->Output = fminf(fmaxf(fpid->Output, -fpid->MaxOut), fpid->MaxOut);
    } else {
        fpid->Pout = 0.0f;
        fpid->ITerm = 0.0f;
        fpid->Dout = 0.0f;
        fpid->Output = fpid->Iout;
    }

    fpid->Output = fminf(fmaxf(fpid->Output, -fpid->MaxOut), fpid->MaxOut);

    fpid->Last_Measure = fpid->Measure;
    fpid->Last_Output  = fpid->Output;
    fpid->Last_Dout    = fpid->Dout;
    fpid->e_last       = fpid->e;
    fpid->Last_ITerm   = fpid->ITerm;
    return fpid->Output;
}

float DMFuzzyPIDCalculate(FuzzyPIDInstance *fpid, float measure, float ref)
{
    if (!fpid)
        return 0.0f;
    if (fpid->Improve & FUZZYPID_ErrorHandle)
        f_PID_ErrorHandle(fpid);
    FuzzyPIDAdjust(fpid, measure, ref);

    float raw_err = fpid->Ref - fpid->Measure;
    if (raw_err > PI)
        raw_err -= 2.0f * PI;
    else if (raw_err < -PI)
        raw_err += 2.0f * PI;
    fpid->e = raw_err;
    fpid->ec = (fpid->e - fpid->e_last) / ((fpid->dt > 0.0001f) ? fpid->dt : 0.001f);
    fpid->e_norm  = f_Linear_Normalize(fpid->e, fpid->e_max);
    fpid->ec_norm = f_Linear_Normalize(fpid->ec, fpid->ec_max);
    f_Update_PID_Params(fpid);

    if (fabsf(fpid->e) > fpid->DeadBand) {
        float dt = (fpid->dt > 0.0001f) ? fpid->dt : 0.0001f;
        fpid->Pout = fpid->Kp * fpid->e;
        fpid->ITerm = fpid->Ki * fpid->e * dt;
        fpid->Dout = fpid->Kd * (fpid->e - fpid->e_last) / dt;

        if (fpid->Improve & FUZZYPID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(fpid);
        if (fpid->Improve & FUZZYPID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(fpid);
        if (fpid->Improve & FUZZYPID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(fpid);
        if (fpid->Improve & FUZZYPID_Derivative_Filter)
            f_Derivative_Filter(fpid);
        if (fpid->Improve & FUZZYPID_Integral_Limit)
            f_Integral_Limit(fpid);
        else
            fpid->Iout += fpid->ITerm;
        if (fpid->Improve & FUZZYPID_Integral_Filter)
            f_Integral_Filter(fpid);
        fpid->Output = fpid->Pout + fpid->Iout + fpid->Dout;

        if (fpid->Improve & FUZZYPID_OutputFilter)
            f_Output_Filter(fpid);

        fpid->Output = fminf(fmaxf(fpid->Output, -fpid->MaxOut), fpid->MaxOut);
    } else {
        fpid->Pout = 0.0f;
        fpid->ITerm = 0.0f;
        fpid->Dout = 0.0f;
        fpid->Output = fpid->Iout;
    }

    fpid->Output = fminf(fmaxf(fpid->Output, -fpid->MaxOut), fpid->MaxOut);

    fpid->Last_Measure = fpid->Measure;
    fpid->Last_Output  = fpid->Output;
    fpid->Last_Dout    = fpid->Dout;
    fpid->e_last       = fpid->e;
    fpid->Last_ITerm   = fpid->ITerm;
    return fpid->Output;
}

float DJI2006FuzzyPIDCalculate(FuzzyPIDInstance *fpid, float measure, float ref)
{
    if (!fpid)
        return 0.0f;
    if (fpid->Improve & FUZZYPID_ErrorHandle)
        f_PID_ErrorHandle(fpid);
    FuzzyPIDAdjust(fpid, measure, ref);

    if (fabsf(fpid->e) > fpid->DeadBand) {
        float dt = (fpid->dt > 0.0001f) ? fpid->dt : 0.0001f;
        fpid->Pout = fpid->Kp * fpid->e;
        fpid->ITerm = fpid->Ki * fpid->e * dt;
        fpid->Dout = fpid->Kd * (fpid->e - fpid->e_last) / dt;

        if (fpid->Improve & FUZZYPID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(fpid);
        if (fpid->Improve & FUZZYPID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(fpid);
        if (fpid->Improve & FUZZYPID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(fpid);
        if (fpid->Improve & FUZZYPID_Derivative_Filter)
            f_Derivative_Filter(fpid);
        if (fpid->Improve & FUZZYPID_Integral_Limit)
            f_Integral_Limit(fpid);
        else
            fpid->Iout += fpid->ITerm;
        if (fpid->Improve & FUZZYPID_Integral_Filter)
            f_Integral_Filter(fpid);
        fpid->Output = fpid->Pout + fpid->Iout + fpid->Dout;

        if (fpid->e > 5.0f || fpid->e < -5.0f)
            fpid->Output *= -1.0f;

        if (fpid->Improve & FUZZYPID_OutputFilter)
            f_Output_Filter(fpid);

        fpid->Output = fminf(fmaxf(fpid->Output, -fpid->MaxOut), fpid->MaxOut);
    } else {
        fpid->Pout = 0.0f;
        fpid->ITerm = 0.0f;
        fpid->Dout = 0.0f;
        fpid->Output = fpid->Iout;
    }

    fpid->Output = fminf(fmaxf(fpid->Output, -fpid->MaxOut), fpid->MaxOut);

    fpid->Last_Measure = fpid->Measure;
    fpid->Last_Output  = fpid->Output;
    fpid->Last_Dout    = fpid->Dout;
    fpid->e_last       = fpid->e;
    fpid->Last_ITerm   = fpid->ITerm;
    return fpid->Output;
}

void FuzzyPIDReset(FuzzyPIDInstance *fpid)
{
    if (!fpid)
        return;
    fpid->Kp = fpid->Kp0;
    fpid->Ki = fpid->Ki0;
    fpid->Kd = fpid->Kd0;
    fpid->e = 0.0f; fpid->e_norm = 0.0f;
    fpid->ec = 0.0f; fpid->ec_norm = 0.0f;
    fpid->e_last = 0.0f;
    fpid->Delta_Kp = 0.0f; fpid->Delta_Ki = 0.0f; fpid->Delta_Kd = 0.0f;
    fpid->Iout = 0.0f; fpid->ITerm = 0.0f; fpid->Last_ITerm = 0.0f;
    fpid->Pout = 0.0f; fpid->Dout = 0.0f; fpid->Last_Dout = 0.0f;
    fpid->Output = 0.0f; fpid->Last_Output = 0.0f;
    fpid->Measure = 0.0f; fpid->Last_Measure = 0.0f; fpid->Ref = 0.0f;
    fpid->dt = 0.0f;
    fpid->ErrorHandler.ERRORCount = 0;
    fpid->ErrorHandler.ERRORType = FUZZYPID_ERROR_NONE;
    LowPassFilter_Init(&fpid->Output_LPF, fpid->Output_LPF_RC);
    LowPassFilter_Init(&fpid->Derivative_LPF, fpid->Derivative_LPF_RC);
    LowPassFilter_Init(&fpid->Integral_LPF, fpid->Integral_LPF_RC);
    DWT_GetDeltaT(&fpid->DWT_CNT);
}

void FuzzyPIDSetEnable(FuzzyPIDInstance *fpid, uint8_t enable)
{
    if (!fpid)
        return;
    fpid->Enabled = enable;
    if (!enable) {
        fpid->Kp = fpid->Kp0;
        fpid->Ki = fpid->Ki0;
        fpid->Kd = fpid->Kd0;
    }
}

void FuzzyPIDUpdateParams(FuzzyPIDInstance *fpid, float kp, float ki, float kd)
{
    if (!fpid)
        return;
    fpid->Kp0 = kp; fpid->Ki0 = ki; fpid->Kd0 = kd;
    if (!fpid->Enabled) {
        fpid->Kp = kp; fpid->Ki = ki; fpid->Kd = kd;
    }
}

void FuzzyPIDGetCurrentParams(FuzzyPIDInstance *fpid, float *kp, float *ki, float *kd)
{
    if (!fpid)
        return;
    if (kp) *kp = fpid->Kp;
    if (ki) *ki = fpid->Ki;
    if (kd) *kd = fpid->Kd;
}

void FuzzyPIDLoadDefaultRules(FuzzyPIDInstance *fpid)
{
    if (!fpid)
        return;
    for (int i = 0; i < FUZZY_TABLE_SIZE; i++) {
        for (int j = 0; j < FUZZY_TABLE_SIZE; j++) {
            fpid->Kp_rule_table[i][j] = DEFAULT_KP_RULES[i][j];
            fpid->Ki_rule_table[i][j] = DEFAULT_KI_RULES[i][j];
            fpid->Kd_rule_table[i][j] = DEFAULT_KD_RULES[i][j];
        }
    }
}

void FuzzyPIDLoadCustomRules(FuzzyPIDInstance *fpid,
                             FuzzySet_e kp_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE],
                             FuzzySet_e ki_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE],
                             FuzzySet_e kd_table[FUZZY_TABLE_SIZE][FUZZY_TABLE_SIZE],
                             uint8_t *sign)
{
    if (!fpid || !kp_table || !ki_table || !kd_table) {
        if (sign) *sign = 0;
        return;
    }
    for (int i = 0; i < FUZZY_TABLE_SIZE; i++) {
        for (int j = 0; j < FUZZY_TABLE_SIZE; j++) {
            if ((int)kp_table[i][j] < 0 || kp_table[i][j] >= FUZZY_TABLE_SIZE) {
                if (sign) *sign = 0;
                return;
            }
            if ((int)ki_table[i][j] < 0 || ki_table[i][j] >= FUZZY_TABLE_SIZE) {
                if (sign) *sign = 0;
                return;
            }
            if ((int)kd_table[i][j] < 0 || kd_table[i][j] >= FUZZY_TABLE_SIZE) {
                if (sign) *sign = 0;
                return;
            }
            fpid->Kp_rule_table[i][j] = kp_table[i][j];
            fpid->Ki_rule_table[i][j] = ki_table[i][j];
            fpid->Kd_rule_table[i][j] = kd_table[i][j];
        }
    }
    if (sign) *sign = 1;
}
