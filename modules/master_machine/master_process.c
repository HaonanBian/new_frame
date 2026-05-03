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

static const uint16_t CRC16_INIT = 0xffff;
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

static uint16_t GetVisionCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc16 = CRC16_INIT;
    uint8_t byte;
    uint8_t i;

    while (len--)
    {
        byte = *data++;
        i = (crc16 ^ byte) & 0x00ff;
        crc16 = (crc16 >> 8) ^ CRC16_TABLE[i];
    }

    return crc16;
}

static uint8_t CheckVisionCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc16 = (uint16_t)(((uint16_t)data[len - 1] << 8) | data[len - 2]);
    return GetVisionCRC16(data, len - 2) == crc16;
}

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
    uint16_t crc16;

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
    crc16 = GetVisionCRC16(buf, VISION_SEND_SIZE - 2u);
    buf[41] = (uint8_t)(crc16 & 0xFFu);
    buf[42] = (uint8_t)(crc16 >> 8);
}

static uint8_t UnpackVisionToGimbal(const uint8_t *buf)
{
    if (buf[0] != VISION_FRAME_HEAD_0 || buf[1] != VISION_FRAME_HEAD_1)
        return 0;

    if (!CheckVisionCRC16(buf, VISION_RECV_SIZE))
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
