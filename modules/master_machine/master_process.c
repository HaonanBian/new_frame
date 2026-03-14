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
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include <string.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
#ifdef VISION_USE_UART
static USARTInstance *vision_usart_instance;
static uint8_t vision_rx_stream[USART_RXBUFF_LIMIT];
static uint16_t vision_rx_stream_len;
#endif

void VisionSetQuaternion(const float *q)
{
    send_data.q[0] = q[0];
    send_data.q[1] = q[1];
    send_data.q[2] = q[2];
    send_data.q[3] = q[3];
}

void VisionSetAltitude(float yaw, float pitch, float yaw_vel, float pitch_vel)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch_vel = pitch_vel;
}

void VisionSetStatus(Vision_Mode_e mode, float bullet_speed, uint16_t bullet_count)
{
    send_data.mode = (uint8_t)mode;
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

uint8_t VisionIsOnline(void)
{
    return DaemonIsOnline(vision_daemon_instance);
}

static void PackGimbalToVision(uint8_t *buf)
{
    buf[0] = VISION_FRAME_HEAD_0;
    buf[1] = VISION_FRAME_HEAD_1;
    buf[2] = send_data.mode;
    memcpy(&buf[3], send_data.q, 16);
    memcpy(&buf[19], &send_data.yaw, 4);
    memcpy(&buf[23], &send_data.yaw_vel, 4);
    memcpy(&buf[27], &send_data.pitch, 4);
    memcpy(&buf[31], &send_data.pitch_vel, 4);
    memcpy(&buf[35], &send_data.bullet_speed, 4);
    buf[39] = (uint8_t)(send_data.bullet_count & 0xFFu);
    buf[40] = (uint8_t)(send_data.bullet_count >> 8);
    buf[41] = VISION_FRAME_TAIL_0;
    buf[42] = VISION_FRAME_TAIL_1;
}

static uint8_t UnpackVisionToGimbal(const uint8_t *buf)
{
    if (buf[0] != VISION_FRAME_HEAD_0 || buf[1] != VISION_FRAME_HEAD_1)
        return 0;

    if (buf[VISION_RECV_SIZE - 2] != VISION_FRAME_TAIL_0 ||
        buf[VISION_RECV_SIZE - 1] != VISION_FRAME_TAIL_1)
        return 0;

    uint8_t mode = buf[2];
    recv_data.control = (mode >= 1u) ? 1u : 0u;
    recv_data.shoot = (mode == 2u) ? 1u : 0u;
    memcpy(&recv_data.yaw, &buf[3], 4);
    memcpy(&recv_data.yaw_vel, &buf[7], 4);
    memcpy(&recv_data.yaw_acc, &buf[11], 4);
    memcpy(&recv_data.pitch, &buf[15], 4);
    memcpy(&recv_data.pitch_vel, &buf[19], 4);
    memcpy(&recv_data.pitch_acc, &buf[23], 4);
    recv_data.data_updated = 1u;
    return 1;
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

static uint32_t rx_valid_cnt = 0;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    uint16_t recv_size;

    recv_size = USARTGetLastRecvSize(vision_usart_instance);
    if (recv_size == 0)
        return;

    if (recv_size > (uint16_t)vision_usart_instance->recv_buff_size)
        recv_size = vision_usart_instance->recv_buff_size;

    if ((uint16_t)(vision_rx_stream_len + recv_size) > (uint16_t)sizeof(vision_rx_stream))
    {
        if (recv_size >= (uint16_t)sizeof(vision_rx_stream))
        {
            memcpy(vision_rx_stream,
                   vision_usart_instance->recv_buff + recv_size - sizeof(vision_rx_stream),
                   sizeof(vision_rx_stream));
            vision_rx_stream_len = sizeof(vision_rx_stream);
        }
        else
        {
            uint16_t keep_len = (uint16_t)(sizeof(vision_rx_stream) - recv_size);
            memmove(vision_rx_stream, vision_rx_stream + vision_rx_stream_len - keep_len, keep_len);
            memcpy(vision_rx_stream + keep_len, vision_usart_instance->recv_buff, recv_size);
            vision_rx_stream_len = (uint16_t)(keep_len + recv_size);
        }
    }
    else
    {
        memcpy(vision_rx_stream + vision_rx_stream_len, vision_usart_instance->recv_buff, recv_size);
        vision_rx_stream_len = (uint16_t)(vision_rx_stream_len + recv_size);
    }

    while (vision_rx_stream_len >= VISION_RECV_SIZE)
    {
        if (vision_rx_stream[0] != VISION_FRAME_HEAD_0 ||
            vision_rx_stream[1] != VISION_FRAME_HEAD_1)
        {
            memmove(vision_rx_stream, vision_rx_stream + 1, vision_rx_stream_len - 1);
            vision_rx_stream_len--;
            continue;
        }

        if (vision_rx_stream[VISION_RECV_SIZE - 2] != VISION_FRAME_TAIL_0 ||
            vision_rx_stream[VISION_RECV_SIZE - 1] != VISION_FRAME_TAIL_1)
        {
            memmove(vision_rx_stream, vision_rx_stream + 1, vision_rx_stream_len - 1);
            vision_rx_stream_len--;
            continue;
        }

        if (UnpackVisionToGimbal(vision_rx_stream))
        {
            rx_valid_cnt++;
            DaemonReload(vision_daemon_instance);
            memmove(vision_rx_stream, vision_rx_stream + VISION_RECV_SIZE, vision_rx_stream_len - VISION_RECV_SIZE);
            vision_rx_stream_len -= VISION_RECV_SIZE;
        }
        else
        {
            memmove(vision_rx_stream, vision_rx_stream + 1, vision_rx_stream_len - 1);
            vision_rx_stream_len--;
        }
    }

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
    static uint8_t send_buff[VISION_SEND_SIZE];
    if (!USARTIsReady(vision_usart_instance))
        return;
    PackGimbalToVision(send_buff);
    USARTSend(vision_usart_instance, send_buff, VISION_SEND_SIZE, USART_TRANSFER_IT);
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len < VISION_RECV_SIZE)
        return;

    if (UnpackVisionToGimbal(vis_recv_buff))
        DaemonReload(vision_daemon_instance);
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
    PackGimbalToVision(send_buff);
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif // VISION_USE_VCP

void VisionDataConsumed(void)
{
    recv_data.data_updated = 0;
}
