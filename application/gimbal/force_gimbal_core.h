#ifndef FORCE_GIMBAL_CORE_H
#define FORCE_GIMBAL_CORE_H

#include <stdint.h>
#include <math.h>

// 1. 参数定义
// 达妙 MIT 模式的限制范围 (必须与上位机一致)
#define MIT_P_MIN -12.5f
#define MIT_P_MAX 12.5f
#define MIT_V_MIN -30.0f
#define MIT_V_MAX 30.0f
#define MIT_KP_MIN 0.0f
#define MIT_KP_MAX 500.0f
#define MIT_KD_MIN 0.0f
#define MIT_KD_MAX 5.0f
#define MIT_T_MIN -10.0f
#define MIT_T_MAX 10.0f

// 2. 类型定义
// 电机驱动类型
typedef enum {
    FORCE_MOTOR_NONE = 0,
    MOTOR_TYPE_GM6020_VOLTAGE,  // GM6020 电压模式 (工程派/旧框架默认)
    MOTOR_TYPE_GM6020_CURRENT,  // GM6020 电流模式 (物理派)
    MOTOR_TYPE_DM_MIT           // 达妙 MIT 模式 (纯力控)
} ForceMotorType_e;

// 工作模式 (决定算法行为)
typedef enum {
    MODE_DISABLE = 0,    // 失能/放松
    MODE_MEASURE,        // 测量模式 (开环输出，用于 Python 辨识)
    MODE_VALIDATE,       // 验证模式 (闭环自生轨迹，用于 验证参数)
    MODE_COMPETITION     // 比赛模式 (闭环外部目标，全功能)
} ForceWorkMode_e;

// PID 对象
typedef struct {
    float kp, ki, kd;
    float error_sum;
    float last_error;
    float max_out;       // 输出限幅
    float max_iout;      // 积分限幅
} ForcePID_t;

// 轨迹平滑器
typedef struct {
    float target_pos; // 原始目标位置 (上位机下发的阶跃信号)

    float out_pos;    // 平滑后的连续位置
    float out_vel;    // 平滑后的连续速度
    float out_acc;    // 平滑后的连续加速度

    float w_n;        // 自然频率 (决定追踪速度/带宽，越大越快)
    float zeta;       // 阻尼比 (设为 1.0 为临界阻尼，绝对不过冲)
    float dt;         // 控制周期 (默认 1ms = 0.001f)
    uint8_t wrap_enable;
} TrajectorySmoother_t;

// 平滑器函数声明
void Smoother_Init(TrajectorySmoother_t *smoother, float w_n, float zeta, float dt);
void Smoother_Update(TrajectorySmoother_t *smoother, float raw_target_pos);
void Smoother_SetWrap(TrajectorySmoother_t *smoother, uint8_t wrap_enable);

// 载弹量补偿配置结构体
typedef struct {
    float full_load_torque;      // 满弹时的额外重力补偿力矩 (Nm)
    float empty_load_torque;     // 空弹时的重力补偿力矩 (Nm)
    float compensation_min;       // 补偿限幅下限 (Nm)
    float compensation_max;      // 补偿限幅上限 (Nm)
} LoadCompensationConfig_s;

// 核心单轴对象
typedef struct {
    // 配置项
    ForceMotorType_e motor_type;
    float output_scale;  // 1.0(达妙) 或 25000(电压) 或 7370(电流)

    // 物理参数
    float J, B, C;        // 惯量、粘滞、摩擦
    float G_cos, G_sin;  // 重力项

    // 运行时状态
    float current_pos;    // rad
    float current_vel;   // rad/s

    // 控制目标
    float target_pos;
    float target_vel;
    float target_acc;

    // 计算结果 (只读)
    float ff_torque;      // 前馈贡献
    float pid_torque;     // PID反馈贡献
    float load_torque;    // 载弹量补偿力矩
    float total_torque;   // 总力矩

    int16_t output_raw;   // CAN 原始输出

    // 内部组件
    ForcePID_t pid_pos;
    ForcePID_t pid_vel;
    float start_pos;
    uint8_t start_pos_inited;
    uint32_t last_tick;

    // 载弹量补偿
    LoadCompensationConfig_s load_cfg;
    float load_ratio;     // 载弹比例 0.0f~1.0f

    // 达妙专用
    struct {
        uint8_t id;
        uint8_t data[8];
    } mit_frame;

} ForceAxis_t;

// 4. 函数接口

// 基础初始化
void ForceAxis_Init(ForceAxis_t *axis, ForceMotorType_e type, float scale,
                    float j, float b, float c, float g_cos, float g_sin);

// 普通 PID 设置
void ForceAxis_SetPID(ForceAxis_t *axis, float p_kp, float p_ki, float p_kd,
                                         float v_kp, float v_ki, float v_kd);

// 反馈更新
void ForceAxis_UpdateFeedback(ForceAxis_t *axis, float pos_rad, float vel_rads);

// 目标设置
void ForceAxis_SetTarget(ForceAxis_t *axis, float pos, float vel, float acc);

// 核心计算 (每 1ms 调用一次)
void ForceAxis_Calc(ForceAxis_t *axis, ForceWorkMode_e mode, float t_sec);

// 辅助函数
void ForceAxis_PrepareMode(ForceAxis_t *axis, ForceWorkMode_e mode, float t_sec);
void ForceAxis_CalcVelocityControl(ForceAxis_t *axis, float pos_loop_vel_cmd);
void ForceAxis_MapOutput(ForceAxis_t *axis);

// 达妙数据获取
uint8_t* ForceAxis_GetMITData(ForceAxis_t *axis);

// 载弹量补偿
void ForceAxis_InitLoadCompensation(ForceAxis_t *axis, LoadCompensationConfig_s *cfg);
void ForceAxis_SetLoadCompensation(ForceAxis_t *axis, float heat_remain, float heat_limit);
void ForceAxis_SetLoadRatio(ForceAxis_t *axis, float load_ratio);

#endif
