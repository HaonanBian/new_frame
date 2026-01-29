#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "motor_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    // 解析电机ID和状态
    measure->id = rxbuff[0] & 0x0F;
    measure->state = rxbuff[0] >> 4;

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    // 计算总圈数
    if (measure->position - measure->last_position > (DM_P_MAX - DM_P_MIN) / 2)
        measure->total_round--;
    else if (measure->last_position - measure->position > (DM_P_MAX - DM_P_MIN) / 2)
        measure->total_round++;

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    
    // 电机通信丢失时，设置停止标志
    motor->stop_flag = MOTOR_STOP;
    
    // 增加丢失计数
    motor->lost_cnt++;
    
    // 可以在这里添加报警或其他处理逻辑
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    // 初始化PID参数，但不使用内部PID计算
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    
    // 不添加到实例列表，不创建任务，直接在应用层使用MIT模式控制
    // dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}


void DMMotorTask(void const *argument)
{
    float pid_ref, position_ref, velocity_ref, torque_ref;
    float position_fdb, velocity_fdb;
    float current_ff = 0, speed_ff = 0;
    
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;
    
    while (1)
    {
        pid_ref = motor->pid_ref;
        
        // 获取反馈值，优先使用外部反馈
        position_fdb = (motor->other_angle_feedback_ptr != NULL) ? (*motor->other_angle_feedback_ptr) : measure->position;
        velocity_fdb = (motor->other_speed_feedback_ptr != NULL) ? (*motor->other_speed_feedback_ptr) : measure->velocity;
        
        // 获取前馈值
        if (motor->speed_feedforward_ptr != NULL)
            speed_ff = *motor->speed_feedforward_ptr;
        if (motor->current_feedforward_ptr != NULL)
            current_ff = *motor->current_feedforward_ptr;
        
        // 根据控制模式选择不同的PID计算
        switch (setting->outer_loop_type)
        {
            case ANGLE_LOOP:
                // 位置环计算，输出速度指令
                position_ref = pid_ref;
                velocity_ref = PIDCalculate(&motor->angle_PID, position_ref, position_fdb) + speed_ff;
                
                // 速度环计算，输出力矩指令
                torque_ref = PIDCalculate(&motor->speed_PID, velocity_ref, velocity_fdb);
                break;
                
            case SPEED_LOOP:
                // 速度环计算，输出力矩指令
                velocity_ref = pid_ref;
                torque_ref = PIDCalculate(&motor->speed_PID, velocity_ref, velocity_fdb) + current_ff;
                break;
                
            case CURRENT_LOOP:
                // 电流环计算，直接输出力矩指令
                torque_ref = pid_ref;
                break;
                
            default:
                torque_ref = 0;
                break;
        }
        
        // 处理电机方向反转
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        {
            position_ref *= -1;
            velocity_ref *= -1;
            torque_ref *= -1;
        }
        
        // 限幅
        LIMIT_MIN_MAX(torque_ref, DM_T_MIN, DM_T_MAX);
        LIMIT_MIN_MAX(velocity_ref, DM_V_MIN, DM_V_MAX);
        
        // 填充CAN发送数据
        motor_send_mailbox.position_des = float_to_uint(position_ref, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(velocity_ref, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(torque_ref, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 2;
        motor_send_mailbox.Kd = 1;
        // 电机停止处理
        if (motor->stop_flag == MOTOR_STOP)
        {
            motor_send_mailbox.position_des = float_to_uint(position_fdb, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
        }
        
        // 填充CAN发送缓冲区
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
        
        // 发送CAN数据
        CANTransmit(motor->motor_can_instace, 1);
        
        osDelay(2);
    }
}
void DMMotorMITCtrl(DMMotorInstance *motor, float pos, float vel, float kp, float kd, float torq)
{
    DMMotor_Send_s motor_send_mailbox;
    
    // 处理电机方向反转
    if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        pos *= -1;
        vel *= -1;
        torq *= -1;
    }
    
    // 限幅
    LIMIT_MIN_MAX(torq, DM_T_MIN, DM_T_MAX);
    LIMIT_MIN_MAX(vel, DM_V_MIN, DM_V_MAX);
    
    // 填充CAN发送数据
    motor_send_mailbox.position_des = float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    motor_send_mailbox.velocity_des = float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    motor_send_mailbox.torque_des = float_to_uint(torq, DM_T_MIN, DM_T_MAX, 12);
    motor_send_mailbox.Kp = float_to_uint(kp, 0.0f, 500.0f, 12);
    motor_send_mailbox.Kd = float_to_uint(kd, 0.0f, 5.0f, 12);
    
    // 填充CAN发送缓冲区
    motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
    motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
    motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
    motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
    motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
    motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
    motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
    motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
    
    // 发送CAN数据
    CANTransmit(motor->motor_can_instace, 1);
}

void DMMotorControlInit()
{
    // 当使用MIT模式控制时，不需要创建周期性的电机控制任务
    // 电机控制直接在应用层（如GimbalTask）中使用DMMotorMITCtrl函数进行
    // 此函数保留仅用于向后兼容
    if (idx > 0)
    {
        char dm_task_name[5] = "dm";
        // 遍历所有电机实例,创建任务
        for (size_t i = 0; i < idx; i++)
        {
            char dm_id_buff[2] = {0};
            __itoa(i, dm_id_buff, 10);
            strcat(dm_task_name, dm_id_buff);
            osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
            dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
        }
    }
}