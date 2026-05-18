#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "referee_task.h"
#include "referee_UI.h"
#include "controller.h"
#include "arm_math.h"
#include "super_cap.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

// 载弹量计数器 (无裁判系统时使用)
// #define MAX_BULLET_COUNT    50  // 根据机器人实际最大载弹量修改
// static int16_t bullet_count = MAX_BULLET_COUNT; // 当前剩余弹量
// static int16_t last_loader_angle = 0;            // 上一次拨盘累计角度，用于检测拨盘转动
static uint8_t loading_in_progress = 0;         // 当前是否处于拨盘中
static uint8_t loader_angle_motion_active = 0;
static float loader_start_angle = 0.0f;
static float loader_target_angle = 0.0f;
// static float loader_count_ref_angle = 0.0f;
static loader_mode_e last_load_mode = LOAD_STOP;
static uint8_t heat_single_shot_active = 0;
static uint8_t loader_hold_active = 0;

// 热量限制相关变量
static heat_limit_status_e heat_status = HEAT_OK;
static referee_info_t *referee_data; // 裁判系统数据指针

static uint16_t GetShooterRestHeat(void)
{
    if (referee_data == NULL) {
        referee_data = GetRefereeData();
    }

    if (referee_data == NULL || referee_data->GameRobotState.robot_id == 0) {
        return 0;
    }

    uint16_t heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    uint16_t current_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    return heat_limit > current_heat ? heat_limit - current_heat : 0;
}

static uint8_t IsLowLevelHeatLimit(void)
{
    if (referee_data == NULL) {
        referee_data = GetRefereeData();
    }

    if (referee_data == NULL || referee_data->GameRobotState.robot_id == 0) {
        return 0;
    }

    return referee_data->GameRobotState.robot_level >= 1 && referee_data->GameRobotState.robot_level <= 2;
}

static float LoaderOneBulletMotorAngle(void)
{
    return ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;
}

static float LoaderDirectionSign(void)
{
    return loader->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -1.0f : 1.0f;
}

static float LoaderForwardProgress(float start_angle, float current_angle)
{
    return (current_angle - start_angle) * LoaderDirectionSign();
}

static float LoaderRawTargetToRef(float raw_target)
{
    return loader->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE ? -raw_target : raw_target;
}

static void LoaderResetRuntimeState(void)
{
    loading_in_progress = 0;
    loader_angle_motion_active = 0;
    loader_hold_active = 0;
}

static void LoaderStartAngleMotion(uint8_t bullet_num)
{
    float delta = LoaderOneBulletMotorAngle() * (float)bullet_num;
    loader_start_angle = loader->measure.total_angle;
    loader_target_angle = loader_start_angle + LoaderDirectionSign() * delta;
    // loader_count_ref_angle = loader_start_angle;
    loader_angle_motion_active = 1;
    loading_in_progress = 1;
    loader_hold_active = 0;
}

static uint8_t LoaderAngleMotionDone(void)
{
    float target_delta = loader_target_angle - loader->measure.total_angle;
    float progress = LoaderForwardProgress(loader_start_angle, loader->measure.total_angle);
    float target_progress = LoaderForwardProgress(loader_start_angle, loader_target_angle);
    if (target_delta < 0.0f)
        target_delta = -target_delta;
    return target_delta <= LoaderOneBulletMotorAngle() * 0.05f || progress >= target_progress;
}

static void LoaderHoldCurrentPosition(void)
{
    if (!loader_hold_active)
    {
        loader_target_angle = loader->measure.total_angle;
        loader_start_angle = loader_target_angle;
        loader_hold_active = 1;
    }
    loading_in_progress = 0;
    loader_angle_motion_active = 0;
    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    DJIMotorSetRef(loader, LoaderRawTargetToRef(loader_target_angle));
}

static void LoaderRunAngleMotion(void)
{
    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    DJIMotorSetRef(loader, LoaderRawTargetToRef(loader_target_angle));
    if (LoaderAngleMotionDone())
    {
        LoaderResetRuntimeState();
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 0);
    }
}

static float GetHeatLimitedShootRate(float shoot_rate)
{
    uint16_t rest_heat = GetShooterRestHeat();
    uint16_t slowdown_rest = HEAT_BURST_SLOWDOWN_REST;

    if (IsLowLevelHeatLimit()) {
        uint16_t heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
        if (rest_heat == 0 || heat_limit == 0 || (uint32_t)rest_heat * 100U > (uint32_t)heat_limit * 80U) {
            return shoot_rate;
        }
        return shoot_rate > HEAT_BURST_SLOW_RATE ? HEAT_BURST_SLOW_RATE : shoot_rate;
    }

    if (rest_heat == 0 || rest_heat > slowdown_rest) {
        return shoot_rate;
    }

    return shoot_rate > HEAT_BURST_SLOW_RATE ? HEAT_BURST_SLOW_RATE : shoot_rate;
}

static uint8_t ShouldBurstSwitchToSingle(void)
{
    uint16_t rest_heat = GetShooterRestHeat();
    if (IsLowLevelHeatLimit()) {
        return 0;
    }
    uint16_t single_rest = HEAT_BURST_SINGLE_REST;
    return rest_heat != 0 && rest_heat <= single_rest;
}

// static void LoaderUpdateBulletCount(void)
// {
//     if (!loading_in_progress || bullet_count <= 0)
//         return;
//     float progress = LoaderForwardProgress(loader_count_ref_angle, loader->measure.total_angle);
//     if (progress >= LoaderOneBulletMotorAngle())
//     {
//         int16_t fed = (int16_t)(progress / LoaderOneBulletMotorAngle());
//         if (fed > bullet_count)
//             fed = bullet_count;
//         bullet_count -= fed;
//         loader_count_ref_angle += LoaderDirectionSign() * LoaderOneBulletMotorAngle() * (float)fed;
//     }
// }
//
// static void LoaderCompleteAngleMotionCount(void)
// {
//     if (!loader_angle_motion_active || bullet_count <= 0)
//         return;
//     float progress = LoaderForwardProgress(loader_count_ref_angle, loader_target_angle);
//     if (progress >= LoaderOneBulletMotorAngle())
//     {
//         int16_t fed = (int16_t)(progress / LoaderOneBulletMotorAngle());
//         if (fed > bullet_count)
//             fed = bullet_count;
//         bullet_count -= fed;
//         loader_count_ref_angle += LoaderDirectionSign() * LoaderOneBulletMotorAngle() * (float)fed;
//     }
// }

// 热量限制检测:当前热量 + 两发预估热量 > 热量上限 则禁止发射
static heat_limit_status_e ShootHeatLimitCheck(void)
{
    if (referee_data == NULL) {
        referee_data = GetRefereeData();
    }

    // 空指针检查
    if (referee_data == NULL) {
        return HEAT_OK; // 默认允许发射
    }
    
    // 从裁判系统获取当前热量和热量上限
    uint16_t current_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    uint16_t heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    if (referee_data->GameRobotState.robot_id == 0 || heat_limit == 0) {
        return HEAT_OK;
    }
    
    uint16_t heat_margin = HEAT_SAFE_MARGIN;
    
    // 当前热量 + 预估两发热量 > 上限，禁止发射
    if (current_heat + 2 * BULLET_HEAT_ESTIMATE + heat_margin > heat_limit) {
        return HEAT_LIMITED;
    }
    return HEAT_OK;
}

heat_limit_status_e GetHeatLimitStatus(void)
{
    return heat_status;
}

void ShootInit()
{
    // 注意：裁判系统已在ChassisInit()中初始化，此处通过GetRefereeData()获取指针
    referee_data = GetRefereeData();
    
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = FRIC_SPEED_PID_KP,
                .Ki = FRIC_SPEED_PID_KI,
                .Kd = FRIC_SPEED_PID_KD,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = FRIC_PID_MAX_IOUT,
                .MaxOut = FRIC_PID_MAX_OUT,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp =20, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 12000,
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 8000,
            },
            .current_PID = {
                .Kp = 0.95, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 热量限制检测
    heat_status = ShootHeatLimitCheck();

    // 反馈数据更新(带空指针检查)
    if (referee_data == NULL) {
        referee_data = GetRefereeData();
    }
    if (referee_data != NULL && referee_data->GameRobotState.robot_id != 0) {
        uint16_t heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
        uint16_t current_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
        shoot_feedback_data.rest_heat = heat_limit > current_heat ? heat_limit - current_heat : 0;
    } else {
        shoot_feedback_data.rest_heat = 0;
    }
    
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        LoaderResetRuntimeState();
        heat_single_shot_active = 0;
        last_load_mode = LOAD_STOP;
        shoot_feedback_data.bullet_count = 0;
        shoot_feedback_data.heat_status = heat_status;
        PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
        return;
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    if (heat_status != HEAT_LIMITED)
    {
        switch (shoot_cmd_recv.load_mode)
        {
        case LOAD_STOP:
            heat_single_shot_active = 0;
            if (loader_angle_motion_active)
                LoaderRunAngleMotion();
            else
            {
                LoaderResetRuntimeState();
                DJIMotorOuterLoop(loader, SPEED_LOOP);
                DJIMotorSetRef(loader, 0);
            }
            break;
        case LOAD_1_BULLET:
            heat_single_shot_active = 0;
            if (!loader_angle_motion_active && last_load_mode != LOAD_1_BULLET)
                LoaderStartAngleMotion(1);
            if (loader_angle_motion_active)
                LoaderRunAngleMotion();
            break;
        case LOAD_3_BULLET:
            heat_single_shot_active = 0;
            if (!loader_angle_motion_active && last_load_mode != LOAD_3_BULLET)
                LoaderStartAngleMotion(3);
            if (loader_angle_motion_active)
                LoaderRunAngleMotion();
            break;
        case LOAD_BURSTFIRE:
            if (ShouldBurstSwitchToSingle())
            {
                if (!loader_angle_motion_active && !heat_single_shot_active)
                {
                    LoaderStartAngleMotion(1);
                    heat_single_shot_active = 1;
                }
                if (loader_angle_motion_active)
                    LoaderRunAngleMotion();
                else
                    LoaderHoldCurrentPosition();
            }
            else
            {
                heat_single_shot_active = 0;
                loader_angle_motion_active = 0;
                loader_hold_active = 0;
                if (!loading_in_progress)
                {
                    loading_in_progress = 1;
                }
                DJIMotorOuterLoop(loader, SPEED_LOOP);
                DJIMotorSetRef(loader, GetHeatLimitedShootRate(shoot_cmd_recv.shoot_rate) * 360.0f * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
            }
            break;
        case LOAD_REVERSE:
            heat_single_shot_active = 0;
            LoaderResetRuntimeState();
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, -shoot_cmd_recv.shoot_rate * 360.0f * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
            break;
        default:
            heat_single_shot_active = 0;
            LoaderResetRuntimeState();
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, 0);
            break;
        }
    }
    else
    {
        heat_single_shot_active = 0;
        LoaderHoldCurrentPosition();
    }
    last_load_mode = shoot_cmd_recv.load_mode;
    shoot_feedback_data.bullet_count = 0;

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        DJIMotorSetRef(friction_l, 35000); // 固定22m/s弹速(实测)
        DJIMotorSetRef(friction_r, 35000);
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    // 热量状态传递给其他模块
    shoot_feedback_data.heat_status = heat_status;
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

// 载弹量查询与补弹接口 (供其他模块调用)

/**
 * @brief 获取当前剩余弹量
 * @return int16_t 当前剩余弹量 (0 ~ MAX_BULLET_COUNT)
 */
int16_t GetBulletCount(void)
{
    return 0;
}

/**
 * @brief 补弹 (装填新弹)
 * @param count 补弹数量，传入 <= 0 则装满
 */
void ReloadBullets(int16_t count)
{
    (void)count;
}

/**
 * @brief 强制重置弹量为满弹 (用于测试)
 */
void ResetBulletCount(void)
{
}