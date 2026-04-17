#include "force_gimbal_core.h"
#include <string.h> // memset

// 内部工具
static float fsgn(float x) {
    return (x > 0.001f) ? 1.0f : ((x < -0.001f) ? -1.0f : 0.0f);
}

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (uint16_t) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// PID 计算
static float PID_Calc(ForcePID_t *pid, float target, float measure) {
    float error = target - measure;
    pid->error_sum += error;

    if (pid->error_sum > pid->max_iout) pid->error_sum = pid->max_iout;
    else if (pid->error_sum < -pid->max_iout) pid->error_sum = -pid->max_iout;

    float output = pid->kp * error + pid->ki * pid->error_sum + pid->kd * (error - pid->last_error);
    pid->last_error = error;

    if (output > pid->max_out) output = pid->max_out;
    else if (output < -pid->max_out) output = -pid->max_out;
    return output;
}

// 接口实现

void ForceAxis_Init(ForceAxis_t *axis, ForceMotorType_e type, float scale,
                    float j, float b, float c, float g_cos, float g_sin) {
    memset(axis, 0, sizeof(ForceAxis_t));
    axis->motor_type = type;
    axis->output_scale = scale;
    axis->J = j; axis->B = b; axis->C = c;
    axis->G_cos = g_cos; axis->G_sin = g_sin;
    axis->load_cfg.full_load_torque = 0.0f;
    axis->load_cfg.empty_load_torque = 0.0f;
    axis->load_cfg.compensation_min = 0.0f;
    axis->load_cfg.compensation_max = 0.0f;
    axis->load_ratio = 0.5f;  // 默认 50% 载弹量
}

void ForceAxis_SetPID(ForceAxis_t *axis, float p_kp, float p_ki, float p_kd,
                      float v_kp, float v_ki, float v_kd) {
    axis->pid_pos.kp = p_kp; axis->pid_pos.ki = p_ki; axis->pid_pos.kd = p_kd;
    axis->pid_vel.kp = v_kp; axis->pid_vel.ki = v_ki; axis->pid_vel.kd = v_kd;
    // 默认限幅，可按需修改
    axis->pid_pos.max_out = 20000.0f; axis->pid_pos.max_iout = 1000.0f;
    axis->pid_vel.max_out = 30000.0f; axis->pid_vel.max_iout = 5000.0f;
}

void ForceAxis_UpdateFeedback(ForceAxis_t *axis, float pos_rad, float vel_rads) {
    axis->current_pos = pos_rad;
    axis->current_vel = vel_rads;
    if (axis->start_pos_inited == 0) {
        axis->start_pos = pos_rad;
        axis->start_pos_inited = 1;
    }
}

void ForceAxis_SetTarget(ForceAxis_t *axis, float pos, float vel, float acc) {
    axis->target_pos = pos;
    axis->target_vel = vel;
    axis->target_acc = acc;
}

void ForceAxis_PrepareMode(ForceAxis_t *axis, ForceWorkMode_e mode, float t_sec) {
    if (mode == MODE_VALIDATE) {
        float freq = 1.0f;
        float amp_rad = 0.6f;
        float omega = 2.0f * 3.14159f * freq;
        axis->target_pos = amp_rad * sinf(omega * t_sec) + axis->start_pos;
        axis->target_vel = amp_rad * omega * cosf(omega * t_sec);
        axis->target_acc = -amp_rad * omega * omega * sinf(omega * t_sec);
    }
}

void ForceAxis_CalcVelocityControl(ForceAxis_t *axis, float pos_loop_vel_cmd) {
    axis->ff_torque =
        axis->J * axis->target_acc +
        axis->B * axis->target_vel +
        axis->C * fsgn(axis->target_vel) +
        axis->G_cos * cosf(axis->current_pos) +
        axis->G_sin * sinf(axis->current_pos);

    axis->pid_torque = PID_Calc(&axis->pid_vel, pos_loop_vel_cmd + axis->target_vel, axis->current_vel);
    axis->total_torque = axis->ff_torque + axis->pid_torque + axis->load_torque;
}

void ForceAxis_MapOutput(ForceAxis_t *axis) {
    if (axis->motor_type == MOTOR_TYPE_DM_MIT) {
        if (axis->total_torque > 10.0f) axis->total_torque = 10.0f;
        if (axis->total_torque < -10.0f) axis->total_torque = -10.0f;

        uint16_t p = float_to_uint(0, MIT_P_MIN, MIT_P_MAX, 16);
        uint16_t v = float_to_uint(0, MIT_V_MIN, MIT_V_MAX, 12);
        uint16_t kp = float_to_uint(0, MIT_KP_MIN, MIT_KP_MAX, 12);
        uint16_t kd = float_to_uint(0, MIT_KD_MIN, MIT_KD_MAX, 12);
        uint16_t t = float_to_uint(axis->total_torque * axis->output_scale, MIT_T_MIN, MIT_T_MAX, 12);

        axis->mit_frame.data[0] = (p >> 8);
        axis->mit_frame.data[1] = p;
        axis->mit_frame.data[2] = (v >> 4);
        axis->mit_frame.data[3] = ((v & 0xF) << 4) | (kp >> 8);
        axis->mit_frame.data[4] = kp;
        axis->mit_frame.data[5] = (kd >> 4);
        axis->mit_frame.data[6] = ((kd & 0xF) << 4) | (t >> 8);
        axis->mit_frame.data[7] = t;

        axis->output_raw = (int16_t)(axis->total_torque * 100.0f);
    }
    else {
        float out_val = axis->total_torque * axis->output_scale;

        if (out_val > 29000.0f) out_val = 29000.0f;
        if (out_val < -29000.0f) out_val = -29000.0f;

        axis->output_raw = (int16_t)out_val;
    }
}

// 核心计算入口
void ForceAxis_Calc(ForceAxis_t *axis, ForceWorkMode_e mode, float t_sec) {
    if (mode == MODE_DISABLE) {
        axis->output_raw = 0;
        axis->total_torque = 0;
        return;
    }

    if (mode == MODE_MEASURE) {
        float amp = 0.2f;
        float freq = 1.0f;

        if (axis->G_cos != 0.0f)
        {
            float offset = axis->G_cos;
            float spring_torque = -1.0f * axis->current_pos;
            axis->total_torque = offset + amp * sinf(freq * 2.0f * 3.14159f * t_sec) + spring_torque;
        }
        else
        {
            axis->total_torque = amp * sinf(freq * 2.0f * 3.14159f * t_sec);
        }

        axis->ff_torque = 0;
        axis->pid_torque = 0;
        ForceAxis_MapOutput(axis);
        return;
    }

    ForceAxis_PrepareMode(axis, mode, t_sec);

    float pos_loop_vel_cmd = PID_Calc(&axis->pid_pos, axis->target_pos, axis->current_pos);
    ForceAxis_CalcVelocityControl(axis, pos_loop_vel_cmd);
    ForceAxis_MapOutput(axis);
}

// 初始化平滑器
void Smoother_Init(TrajectorySmoother_t *smoother, float w_n, float zeta, float dt) {
    smoother->target_pos = 0.0f;
    smoother->out_pos = 0.0f;
    smoother->out_vel = 0.0f;
    smoother->out_acc = 0.0f;
    smoother->w_n = w_n;
    smoother->zeta = zeta;
    smoother->dt = dt;
    smoother->wrap_enable = 1;
}

void Smoother_SetWrap(TrajectorySmoother_t *smoother, uint8_t wrap_enable) {
    smoother->wrap_enable = wrap_enable;
}

#define FORCE_PI 3.1415926535f

void Smoother_Update(TrajectorySmoother_t *smoother, float raw_target_pos) {
    smoother->target_pos = raw_target_pos;

    float error = smoother->target_pos - smoother->out_pos;
    if (smoother->wrap_enable) {
        if (error > FORCE_PI) error -= 2.0f * FORCE_PI;
        else if (error < -FORCE_PI) error += 2.0f * FORCE_PI;
    }

    smoother->out_acc = (smoother->w_n * smoother->w_n) * error
                      - (2.0f * smoother->zeta * smoother->w_n) * smoother->out_vel;

    smoother->out_vel += smoother->out_acc * smoother->dt;
    smoother->out_pos += smoother->out_vel * smoother->dt;

    if (smoother->wrap_enable) {
        while (smoother->out_pos > FORCE_PI)  smoother->out_pos -= 2.0f * FORCE_PI;
        while (smoother->out_pos < -FORCE_PI) smoother->out_pos += 2.0f * FORCE_PI;
    }
}

uint8_t* ForceAxis_GetMITData(ForceAxis_t *axis) {
    return axis->mit_frame.data;
}

// 载弹量补偿支持函数

void ForceAxis_InitLoadCompensation(ForceAxis_t *axis, LoadCompensationConfig_s *cfg)
{
    if (axis == NULL || cfg == NULL)
        return;
    axis->load_cfg = *cfg;
}

void ForceAxis_SetLoadCompensation(ForceAxis_t *axis, float heat_remain, float heat_limit)
{
    if (axis == NULL || heat_limit <= 0.0f)
        return;
    // 热量百分比 -> 载弹比例
    float ratio = heat_remain / heat_limit;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    axis->load_ratio = ratio;

    float compensation = axis->load_cfg.empty_load_torque
                      + (axis->load_cfg.full_load_torque - axis->load_cfg.empty_load_torque) * ratio;
    if (compensation > axis->load_cfg.compensation_max) compensation = axis->load_cfg.compensation_max;
    if (compensation < axis->load_cfg.compensation_min) compensation = axis->load_cfg.compensation_min;
    axis->load_torque = compensation;
}

void ForceAxis_SetLoadRatio(ForceAxis_t *axis, float load_ratio)
{
    if (axis == NULL)
        return;
    if (load_ratio < 0.0f) load_ratio = 0.0f;
    if (load_ratio > 1.0f) load_ratio = 1.0f;
    axis->load_ratio = load_ratio;

    // 仅计算有配置的静态力矩补偿（pitch轴用），yaw轴配置为0则不补偿
    // 如果 full_load_torque 和 empty_load_torque 都为0，则 load_torque 为0
    float compensation = axis->load_cfg.empty_load_torque
                      + (axis->load_cfg.full_load_torque - axis->load_cfg.empty_load_torque) * load_ratio;
    if (compensation > axis->load_cfg.compensation_max) compensation = axis->load_cfg.compensation_max;
    if (compensation < axis->load_cfg.compensation_min) compensation = axis->load_cfg.compensation_min;
    axis->load_torque = compensation;
}
