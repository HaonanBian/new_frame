#ifndef FORCE_GIMBAL_CORE_H
#define FORCE_GIMBAL_CORE_H

#include <stdint.h>
#include <math.h>

// ==================== 1. 参数定义 ====================
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

// ==================== 2. 类型定义 ====================

// 电机驱动类型
typedef enum {
    FORCE_MOTOR_TYPE_NONE = 0,
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

// ==========================================
// 轨迹平滑器 (二阶线性 TD)
// 用于将上位机 100Hz 的跳变信号，转换为 1000Hz 丝滑的 位置、速度、加速度
// ==========================================
typedef struct {
    float target_pos; // 原始目标位置 (上位机下发的阶跃信号)
    
    float out_pos;    // 平滑后的连续位置
    float out_vel;    // 平滑后的连续速度
    float out_acc;    // 平滑后的连续加速度

    float w_n;        // 自然频率 (决定追踪速度/带宽，越大越快)
    float zeta;       // 阻尼比 (设为 1.0 为临界阻尼，绝对不过冲)
    float dt;         // 控制周期 (默认 1ms = 0.001f)
} TrajectorySmoother_t;

// 平滑器函数声明
void Smoother_Init(TrajectorySmoother_t *smoother, float w_n, float zeta, float dt);
void Smoother_Update(TrajectorySmoother_t *smoother, float raw_target_pos);

// [核心] 单轴对象
typedef struct {
    // --- 配置项 (Init时设置) ---
    ForceMotorType_e motor_type; // 电机类型
    float output_scale;          // ★核心变革★: 1.0(达妙) 或 25000(电压) 或 7370(电流)
    
    // --- 物理参数 (Python辨识填入) ---
    float J;      // 惯量 (或 电压惯量)
    float B;      // 粘滞 (或 反电动势补偿)
    // float C;      // 摩擦 (或 电压死区)
    float C_pos;  // 正向库伦摩擦
    float C_neg;  // 反向库伦摩擦
    float G_cos;  // 重力余弦项
    float G_sin;  // 重力正弦项 (解决重心偏移)

    // --- 运行时状态 (Update时填入) ---
    float current_pos; // rad
    float current_vel; // rad/s
    
    // --- 控制目标 (SetTarget时填入) ---
    float target_pos;
    float target_vel;
    float target_acc;

    // --- 计算结果 (只读) ---
    float ff_torque;     // 前馈贡献
    float pid_torque;    // 反馈贡献
    float total_torque;  // 总计算值 (Nm 或 归一化电压)
    int16_t output_raw;  // 最终发给 CAN 的整数值
    
    // --- 内部组件 ---
    ForcePID_t pid_pos;
    ForcePID_t pid_vel;
    float start_pos;     // 初始位置记录
    uint8_t start_pos_inited;// 初始化标志位
    uint32_t last_tick;  // 上次计算时间
    
    // --- 达妙专用数据缓存 (用于发送) ---
    struct {
        uint8_t id;
        uint8_t data[8]; // 运算完后生成的8字节数据，直接发这个
    } mit_frame;

} ForceAxis_t;

// ==================== 3. 函数接口 ====================

// 初始化一个轴
void ForceAxis_Init(ForceAxis_t *axis, ForceMotorType_e type, float scale, 
                    float j, float b, float c, float g_cos, float g_sin);

// 设置 PID
void ForceAxis_SetPID(ForceAxis_t *axis, float p_kp, float p_ki, float p_kd, 
                                         float v_kp, float v_ki, float v_kd);

// 喂数据：更新电机反馈 (在 CAN 回调里调用)
void ForceAxis_UpdateFeedback(ForceAxis_t *axis, float pos_rad, float vel_rads);

// 设目标：设置期望运动 (在 比赛模式 下调用)
void ForceAxis_SetTarget(ForceAxis_t *axis, float pos, float vel, float acc);

// 核心运算：每毫秒调用一次 (在 任务循环 里调用)
// mode: 当前模式; t_sec: 当前时间(秒)
void ForceAxis_Calc(ForceAxis_t *axis, ForceWorkMode_e mode, float t_sec);

// 新增设置非对称摩擦力的接口
void ForceAxis_SetDirectionC(ForceAxis_t *axis, float c_pos, float c_neg);

// 辅助：获取要发送给达妙的数据指针
uint8_t* ForceAxis_GetMITData(ForceAxis_t *axis);
void ForceAxis_PrepareMode(ForceAxis_t *axis, ForceWorkMode_e mode, float t_sec);
void ForceAxis_CalcVelocityControl(ForceAxis_t *axis, float pos_loop_vel_cmd);
void ForceAxis_MapOutput(ForceAxis_t *axis);

#endif