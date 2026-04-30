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
#define MAX_BULLET_COUNT    50  // 根据机器人实际最大载弹量修改
static int16_t bullet_count = MAX_BULLET_COUNT; // 当前剩余弹量
static int16_t last_loader_angle = 0;            // 上一次拨盘累计角度，用于检测拨盘转动
static uint8_t loading_in_progress = 0;         // 当前是否处于拨盘中

// 热量限制相关变量
static heat_limit_status_e heat_status = HEAT_OK;
static referee_info_t *referee_data; // 裁判系统数据指针

// 热量限制检测:当前热量 + 单发预估热量 > 热量上限 则禁止发射
static heat_limit_status_e ShootHeatLimitCheck(void)
{
    // 空指针检查
    if (referee_data == NULL) {
        return HEAT_OK; // 默认允许发射
    }
    
    // 从裁判系统获取当前热量和热量上限
    uint16_t current_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    uint16_t heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    
    // 当前热量 + 预估单发热量 > 上限，禁止发射
    if (current_heat + BULLET_HEAT_ESTIMATE > heat_limit) {
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
                .MaxOut = 200,
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
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
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

    // 载弹量计数 (无裁判系统时使用)
    // 原理: 拨盘每转动 ONE_BULLET_DELTA_ANGLE 角度 = 成功送出一发弹
    static int16_t bullet_ref_angle = 0; // 每次发射开始时的拨盘累计角度
    static int8_t  bullet_count_per_shot = 0; // 本次发射累计送出的弹数

    // 记录每次发射指令开始时的拨盘位置 (单次触发，只在进入发射模式瞬间抓取)
    switch (shoot_cmd_recv.load_mode) {
        case LOAD_1_BULLET:
            // 单发: 只记录一次基准
            if (!loading_in_progress) {
                bullet_ref_angle = (int16_t)loader->measure.total_angle;
                bullet_count_per_shot = 0;
                loading_in_progress = 1;
            }
            break;
        case LOAD_3_BULLET:
            if (!loading_in_progress) {
                bullet_ref_angle = (int16_t)loader->measure.total_angle;
                bullet_count_per_shot = 0;
                loading_in_progress = 1;
            }
            break;
        case LOAD_BURSTFIRE:
            // 连发: 持续累加计数
            if (!loading_in_progress) {
                bullet_ref_angle = (int16_t)loader->measure.total_angle;
                bullet_count_per_shot = 0;
                loading_in_progress = 1;
            }
            break;
        default:
            loading_in_progress = 0;
            break;
    }

    // 拨盘转动时，累加计数
    if (loading_in_progress) {
        int16_t delta = (int16_t)loader->measure.total_angle - bullet_ref_angle;
        // 防止第一次刚进入时的瞬间跳变
        if (delta > ONE_BULLET_DELTA_ANGLE / 2) {
            int8_t new_bullets = delta / ONE_BULLET_DELTA_ANGLE;
            if (new_bullets > bullet_count_per_shot) {
                int8_t fed = new_bullets - bullet_count_per_shot;
                // 连发模式下，每送出一发就扣弹量
                if (shoot_cmd_recv.load_mode == LOAD_BURSTFIRE) {
                    if (bullet_count >= fed) {
                        bullet_count -= fed;
                        bullet_count_per_shot = new_bullets;
                    }
                }
                // 单发/三发模式: 等拨盘完全转到位再扣 (在下面的拨盘控制里处理)
            }
        }
    }

    // 单发/三发模式: 拨盘转到目标角度后扣弹 (拨盘停止时)
    switch (shoot_cmd_recv.load_mode) {
        case LOAD_1_BULLET:
            if (loading_in_progress) {
                int16_t delta = (int16_t)loader->measure.total_angle - bullet_ref_angle;
                if (delta >= ONE_BULLET_DELTA_ANGLE - 5 && bullet_count > 0) {
                    bullet_count--;
                    loading_in_progress = 0;
                }
            }
            break;
        case LOAD_3_BULLET:
            if (loading_in_progress) {
                int16_t delta = (int16_t)loader->measure.total_angle - bullet_ref_angle;
                if (delta >= 3 * ONE_BULLET_DELTA_ANGLE - 5 && bullet_count >= 3) {
                    bullet_count -= 3;
                    loading_in_progress = 0;
                } else if (delta >= ONE_BULLET_DELTA_ANGLE - 5 && bullet_count > 0 && bullet_count < 3) {
                    bullet_count = 0;
                    loading_in_progress = 0;
                }
            }
            break;
        default:
            break;
    }

    // 上传到反馈数据，供 gimbal 等模块使用
    shoot_feedback_data.bullet_count = bullet_count;

    // 补弹: 复位弹量 (在 cmd 或 UI 中调用，或通过遥控器某个通道触发)
    // 目前暂时用热量作为补弹检测 (有裁判系统时), 无裁判系统时请在 UI/遥控器中手动补弹
    // 也可通过以下宏开启自动补弹 (上电自动装满):
    // if (bullet_count < 0) bullet_count = MAX_BULLET_COUNT;

    // 反馈数据更新(带空指针检查)
    if (referee_data != NULL) {
        shoot_feedback_data.rest_heat = referee_data->GameRobotState.shooter_barrel_heat_limit 
                                       - referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    } else {
        shoot_feedback_data.rest_heat = 0;
    }
    
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    // 热量限制:当热量受限时不执行拨盘动作
    if (heat_status != HEAT_LIMITED)
    {
        switch (shoot_cmd_recv.load_mode)
        {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
        case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = 150;                                                                    // 完成1发弹丸发射的时间
            break;
        // 三连发,如果不需要后续可能删除
        case LOAD_3_BULLET:
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
            DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
            hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
            dead_time = 300;                                                                        // 完成3发弹丸发射的时间
            break;
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
        // 也有可能需要从switch-case中独立出来
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            // ...
            break;
        default:
            while (1)
                ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
        }
    }
    else
    {
        // 热量受限时停止拨盘
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 0);
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    // 热量限制:当热量受限时关闭摩擦轮
    if (shoot_cmd_recv.friction_mode == FRICTION_ON && heat_status != HEAT_LIMITED)
    {
        DJIMotorSetRef(friction_l, 35000); // 固定22m/s弹速(实测)
        DJIMotorSetRef(friction_r, 35000);
    }
    else // 关闭摩擦轮或热量受限
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
    if (bullet_count < 0) return 0;
    if (bullet_count > MAX_BULLET_COUNT) return MAX_BULLET_COUNT;
    return bullet_count;
}

/**
 * @brief 补弹 (装填新弹)
 * @param count 补弹数量，传入 <= 0 则装满
 */
void ReloadBullets(int16_t count)
{
    if (count <= 0) {
        bullet_count = MAX_BULLET_COUNT;
    } else {
        bullet_count += count;
        if (bullet_count > MAX_BULLET_COUNT)
            bullet_count = MAX_BULLET_COUNT;
    }
}

/**
 * @brief 强制重置弹量为满弹 (用于测试)
 */
void ResetBulletCount(void)
{
    bullet_count = MAX_BULLET_COUNT;
}