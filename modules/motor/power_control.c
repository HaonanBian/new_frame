#include "power_control.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

#define TORQUE_COEF 0.0003662109375f        // (20/16384)*(0.3), 电机转矩系数，与电流和转矩相关
#define POWER_COEF 187.0f / 3591.0f / 9.55f // 电机机械功率系数，与力矩和转速相关，注意框架中速度单位为aps
const float K1[4] = {1.23e-07, 1.23e-07, 1.23e-07, 1.23e-07};
const float K2[4] = {1.453e-07, 1.453e-07, 1.453e-07, 1.453e-07};
const float constant[4] = {4.081f, 4.081f, 4.081f, 4.081f};

// 功率控制相关宏定义
#define WARNING_POWER         40.0f       // 警告功率阈值
#define WARNING_POWER_BUFF    50.0f       // 警告功率缓冲阈值
#define BUFFER_TARGET        30.0f       // 缓冲能量目标值
#define POWER_SCALE_MIN      0.1f        // 最小功率缩放系数
#define POWER_CONTROL_MOTOR_CNT 4

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算
static float initial_torque[POWER_CONTROL_MOTOR_CNT];                // 电机输出轴实际转矩，单位N·m
static float A, B, C;                                                // 测试用
static float power_control_out[POWER_CONTROL_MOTOR_CNT], initial_give_power[POWER_CONTROL_MOTOR_CNT];            // 电机输出功率
static float chassis_max_power, chassis_power, initial_total_power = 0.0f;

// 缓冲能量PID
static PIDInstance buffer_PID;
// 裁判系统数据
static PowerControl_RefereeData_s referee_power_data = {0};

// 缓冲能量PID参数
static float buffer_kp = 0.1f, buffer_ki = 0.0f, buffer_kd = 0.0f;
/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[6] = {0};

/**
 * @brief 设置底盘的最大功率限制
 *
 * @param power_limit 限制的功率值
 */
void SetPowerLimit(float power_limit)
{
    chassis_max_power = power_limit;
}

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    default: // other motors should not be registered here
        while (1)
            LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor."); // 其他电机不应该在这里注册
    }
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *PowerControlInit(Motor_Init_Config_s *config)
{
    if (idx >= POWER_CONTROL_MOTOR_CNT)
    {
        while (1)
            LOGERROR("[power_control] too many motors registered.");
    }

    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    if (instance == NULL)
    {
        while (1)
            LOGERROR("[power_control] malloc failed.");
    }
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器(速度和电流)

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    
    // 初始化缓冲能量PID(仅在第一个电机初始化时执行)
    if (idx == 1) {
        PID_Init_Config_s buffer_pid_config = {
            .Kp = buffer_kp,
            .Ki = buffer_ki,
            .Kd = buffer_kd,
            .IntegralLimit = 1000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 1000,
        };
        PIDInit(&buffer_PID, &buffer_pid_config);
    }
    
    return instance;
}

// 为所有电机实例计算三环PID,发送控制报文
void PowerControl()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 缓冲能量PID计算
    float buffer_pid_out = 0.0f;
    float available_power = chassis_max_power; // 默认使用底盘最大功率限制
    
    // 只有当裁判系统已连接（chassis_power_limit > 0）且有有效数据时才使用裁判系统数据
    if (referee_power_data.chassis_power_limit > 0 && chassis_max_power > 0) {
        PIDCalculate(&buffer_PID, referee_power_data.chassis_power_buffer, BUFFER_TARGET);
        buffer_pid_out = buffer_PID.Output;
        
        // 计算实际可用功率限制
        available_power = chassis_max_power - buffer_pid_out;
        if (available_power < chassis_max_power * POWER_SCALE_MIN) {
            available_power = chassis_max_power * POWER_SCALE_MIN;
        }
    }
    
    initial_total_power = 0.0f;
    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i) // idx实际上是4个
    {                                // 减小访存开销,先保存指针引用
        initial_torque[i] = 0.0f;
        power_control_out[i] = 0.0f;
        initial_give_power[i] = 0.0f;
        motor = dji_motor_instance[i];
        if (motor == NULL)
            continue;
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if ((motor_setting->feedforward_flag & SPEED_FEEDFORWARD) && motor_controller->speed_feedforward_ptr != NULL)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED && motor_controller->other_speed_feedback_ptr != NULL)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps / 6.0f; // 电机的速度单位是度每秒,转换为rpm
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
            initial_torque[i] = pid_ref;
            power_control_out[i] = pid_ref;
            A = K1[i] * initial_torque[i] * initial_torque[i];
            B = K2[i] * pid_measure * pid_measure;
            C = POWER_COEF * pid_measure * initial_torque[i] * TORQUE_COEF;
            initial_give_power[i] = A + B + C + constant[i];
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        power_control_out[i] = pid_ref;
    }
    
    // 计算总功率
    for (uint8_t i = 0; i < idx; i++)
    {
        if (initial_give_power[i] < 0)
        {
            continue;
        }
        initial_total_power += initial_give_power[i];
    }
    
    // 功率限制核心算法
    if (initial_total_power > available_power)
    {
        float ratio = available_power / initial_total_power; // 根据允许的最大功率进行放缩
        for (uint8_t i = 0; i < idx; i++)
        {
            motor = dji_motor_instance[i];
            if (motor == NULL)
                continue;
            measure = &motor->measure;
            pid_measure = measure->speed_aps / 6.0f; // 电机的速度单位是度每秒,转换为rpm
            initial_give_power[i] *= ratio;
            if (initial_give_power[i] < 0) // 功率小于零，表明依靠发电减速，因此不算入功率统计
            {
                continue;
            }
            float a = K1[i];
            float b = TORQUE_COEF * POWER_COEF * pid_measure;
            float c = K2[i] * pid_measure * pid_measure - initial_give_power[i] + constant[i];
            
            // 根据原始输出方向选择计算公式
            if (power_control_out[i] > 0)
            {
                float discriminant = b * b - 4 * a * c;
                if (discriminant >= 0)
                {
                    power_control_out[i] = (-b + sqrt(discriminant)) / (2 * a);
                    if (power_control_out[i] > 15000)
                    {
                        power_control_out[i] = 15000;
                    }
                }
                else
                {
                    power_control_out[i] = 0;
                }
            }
            else
            {
                float discriminant = b * b - 4 * a * c;
                if (discriminant >= 0)
                {
                    power_control_out[i] = (-b - sqrt(discriminant)) / (2 * a);
                    if (power_control_out[i] < -15000)
                    {
                        power_control_out[i] = -15000;
                    }
                }
                else
                {
                    power_control_out[i] = 0;
                }
            }
        }
    }

    for (uint8_t i = 0; i < idx; i++)
    {
        motor = dji_motor_instance[i];
        if (motor == NULL)
            continue;
        set = (int16_t)power_control_out[i];
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i)
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}

/**
 * @brief 设置缓冲能量PID参数
 *
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void SetBufferPID(float kp, float ki, float kd)
{
    buffer_kp = kp;
    buffer_ki = ki;
    buffer_kd = kd;
    
    // 更新PID参数
    buffer_PID.Kp = kp;
    buffer_PID.Ki = ki;
    buffer_PID.Kd = kd;
}

/**
 * @brief 更新裁判系统功率数据
 *
 * @param power 当前底盘功率
 * @param buffer 当前缓冲能量
 * @param limit 功率上限
 */
void UpdatePowerControlRefereeData(float power, float buffer, uint16_t limit)
{
    referee_power_data.chassis_power = power;
    referee_power_data.chassis_power_buffer = buffer;
    referee_power_data.chassis_power_limit = limit;
}

/**
 * @brief 获取底盘最大功率限制
 *
 * @return uint16_t 功率限制值
 */
uint16_t GetChassisPowerLimit(void)
{
    return referee_power_data.chassis_power_limit;
}

/**
 * @brief 获取底盘当前功率
 *
 * @return float 当前功率值
 */
float GetChassisPower(void)
{
    return referee_power_data.chassis_power;
}

/**
 * @brief 获取底盘缓冲能量
 *
 * @return float 缓冲能量值
 */
float GetChassisPowerBuffer(void)
{
    return referee_power_data.chassis_power_buffer;
}

/**
 * @brief 重置所有电机PID的状态（积分项、微分项等）
 *        用于复活后清除累积的PID状态，防止积分释放导致抖动
 */
void ResetAllMotorPID(void)
{
    // 遍历所有注册的电机，清除其三环PID的状态
    for (uint8_t i = 0; i < idx; i++) {
        DJIMotorInstance *motor = dji_motor_instance[i];
        if (motor != NULL) {
            // 清除速度环PID
            ResetPIDRuntimeState(&motor->motor_controller.speed_PID);
            // 清除电流环PID
            ResetPIDRuntimeState(&motor->motor_controller.current_PID);
            // 清除角度环PID
            ResetPIDRuntimeState(&motor->motor_controller.angle_PID);
        }
    }
}
