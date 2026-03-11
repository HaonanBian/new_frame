/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "infantry_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
#ifdef VISION_USE_UART
static USARTInstance *vision_usart_instance;
#endif

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    Infantry_Cmd_Packet_s cmd_packet;
    DaemonReload(vision_daemon_instance); // 喂狗
    if (!InfantryProtocolDecodeCmd(vision_usart_instance->recv_buff, VISION_RECV_SIZE, &cmd_packet))
        return;

    recv_data.fire_mode = cmd_packet.fire ? AUTO_FIRE : NO_FIRE;
    recv_data.target_state = cmd_packet.fire ? READY_TO_FIRE : NO_TARGET;
    recv_data.target_type = NO_TARGET_NUM;
    recv_data.pitch = cmd_packet.pitch_diff;
    recv_data.yaw = cmd_packet.yaw_diff;
    recv_data.distance = cmd_packet.distance;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // 广播发送：固定周期主动上报姿态，不依赖上位机请求
    // 使用IT发送，避免和DMA接收互相影响
    static uint32_t last_send_ms = 0;
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_send_ms) < 5u) // 200Hz
        return;
    if (!USARTIsReady(vision_usart_instance))
        return;

    // buff必须为static，确保异步发送期间内存有效
    static uint8_t send_buff[VISION_SEND_SIZE];
    Infantry_Feedback_Packet_s feedback_packet = {
        .mode = (uint8_t)send_data.work_mode,
        .roll = send_data.roll,
        .pitch = send_data.pitch,
        .yaw = send_data.yaw,
    };

    InfantryProtocolEncodeFeedback(&feedback_packet, send_buff);
    USARTSend(vision_usart_instance, send_buff, VISION_SEND_SIZE, USART_TRANSFER_IT);
    last_send_ms = now_ms;
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    Infantry_Cmd_Packet_s cmd_packet;
    if (!InfantryProtocolDecodeCmd(vis_recv_buff, recv_len, &cmd_packet))
        return;

    recv_data.fire_mode = cmd_packet.fire ? AUTO_FIRE : NO_FIRE;
    recv_data.target_state = cmd_packet.fire ? READY_TO_FIRE : NO_TARGET;
    recv_data.target_type = NO_TARGET_NUM;
    recv_data.pitch = cmd_packet.pitch_diff;
    recv_data.yaw = cmd_packet.yaw_diff;
    recv_data.distance = cmd_packet.distance;
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    Infantry_Feedback_Packet_s feedback_packet = {
        .mode = (uint8_t)send_data.work_mode,
        .roll = send_data.roll,
        .pitch = send_data.pitch,
        .yaw = send_data.yaw,
    };

    InfantryProtocolEncodeFeedback(&feedback_packet, send_buff);
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif // VISION_USE_VCP
