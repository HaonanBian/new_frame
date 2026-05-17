/**
 * @file sp_vision.c
 * @brief sp_vision 视觉协议收发模块
 *
 * 接收: DMA + Idle 中断驱动的字节级状态机
 * 发送: 双缓冲 IT 模式，避免竞态
 *
 * 协议帧格式:
 *   Vision → Gimbal (接收): HEAD[5A A5] + payload(25B) + TAIL[7F FE] = 29B
 *   Gimbal → Vision (发送): HEAD[5A A5] + payload(39B) + TAIL[7F FE] = 43B
 */
#include "sp_vision.h"
#include "daemon.h"
#include "bsp_usart.h"
#include "bsp_usb.h"
#include "bsp_log.h"
#include "string.h"

/* ============================================================
 * 内部静态变量
 * ============================================================ */
static SP_Vision_RecvData_t recv_data;
static SP_Vision_SendPayload_t send_payload;
static SP_Vision_SendFrame_t tx_frame;

static USARTInstance *vision_usart_instance;
static uint8_t *vision_usb_rx_buffer;

static DaemonInstance *vision_daemon_instance;

static uint8_t vision_online = 0;
volatile uint8_t sp_vision_use_vcp = 0U;

#define SP_VISION_RX_BUFFER_SIZE 64U

/* ============================================================
 * 接收解析状态机 (字节级)
 * ============================================================ */
typedef enum {
    SP_VISION_ST_WAIT_H0 = 0,
    SP_VISION_ST_WAIT_H1,
    SP_VISION_ST_PAYLOAD,
    SP_VISION_ST_WAIT_T0,
    SP_VISION_ST_WAIT_T1,
} SP_Vision_ParserState_e;

static SP_Vision_ParserState_e parser_state = SP_VISION_ST_WAIT_H0;
static uint8_t payload_buf[SP_VISION_RECV_PAYLOAD_SIZE];
static uint16_t payload_idx = 0;
static volatile uint8_t payload_ready = 0;

static void SP_VisionResetParser(void)
{
    parser_state = SP_VISION_ST_WAIT_H0;
    payload_idx = 0U;
    payload_ready = 0U;
}

static void SP_VisionApplyRecvPayload(const SP_Vision_RecvPayload_t *payload)
{
    if (!payload)
        return;

    recv_data.mode = payload->mode;
    recv_data.control = (payload->mode >= 1U) ? 1U : 0U;
    recv_data.shoot  = (payload->mode == 2U) ? 1U : 0U;
    recv_data.yaw         = payload->yaw;
    recv_data.pitch       = payload->pitch;
    recv_data.yaw_speed   = payload->yaw_vel;
    recv_data.pitch_speed = payload->pitch_vel;
    recv_data.yaw_acc     = payload->yaw_acc;
    recv_data.pitch_acc   = payload->pitch_acc;
    recv_data.data_updated = 1U;

    vision_online = 1;
    if (vision_daemon_instance)
        DaemonReload(vision_daemon_instance);
}

/**
 * @brief 向解析器喂入一个字节
 * @return int 1 = 完整帧已就绪 (等待消费), 0 = 仍在解析中
 *
 * @note 当 payload_ready == 1 时, 解析器拒绝接收新字节,
 *       直到应用层调用 SP_VisionGetRecvPayload() 消费掉当前帧.
 *       这样可避免覆盖尚未处理的旧数据.
 */
int SP_VisionFeedByte(uint8_t byte)
{
    if (payload_ready)
        return 1;  /* 旧帧尚未消费, 丢弃新字节 */

    switch (parser_state) {
    case SP_VISION_ST_WAIT_H0:
        if (byte == SP_VISION_HEAD0)
            parser_state = SP_VISION_ST_WAIT_H1;
        break;

    case SP_VISION_ST_WAIT_H1:
        if (byte == SP_VISION_HEAD1) {
            payload_idx = 0;
            parser_state = SP_VISION_ST_PAYLOAD;
        } else {
            parser_state = SP_VISION_ST_WAIT_H0;  /* HEAD0 之后的非 HEAD1 字节 -> 重新找帧 */
        }
        break;

    case SP_VISION_ST_PAYLOAD:
        payload_buf[payload_idx++] = byte;
        if (payload_idx >= SP_VISION_RECV_PAYLOAD_SIZE)
            parser_state = SP_VISION_ST_WAIT_T0;
        break;

    case SP_VISION_ST_WAIT_T0:
        if (byte == SP_VISION_TAIL0)
            parser_state = SP_VISION_ST_WAIT_T1;
        else
            parser_state = SP_VISION_ST_WAIT_H0;  /* TAIL0 匹配失败 -> 重新找帧 */
        break;

    case SP_VISION_ST_WAIT_T1:
        if (byte == SP_VISION_TAIL1) {
            payload_ready = 1;
            parser_state = SP_VISION_ST_WAIT_H0;
            return 1;
        } else {
            parser_state = SP_VISION_ST_WAIT_H0;
        }
        break;

    default:
        parser_state = SP_VISION_ST_WAIT_H0;
        break;
    }
    return 0;
}

/**
 * @brief 取出最近一帧解析好的接收载荷
 * @return int 1=成功消费, 0=无数据或参数错误
 */
int SP_VisionGetRecvPayload(SP_Vision_RecvPayload_t *out)
{
    if (!payload_ready || !out)
        return 0;
    memcpy(out, payload_buf, sizeof(SP_Vision_RecvPayload_t));
    payload_ready = 0;
    return 1;
}

static void SP_VisionConsumeBytes(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0U)
        return;

    for (uint16_t i = 0; i < len; ++i) {
        if (SP_VisionFeedByte(data[i])) {
            SP_Vision_RecvPayload_t payload;
            if (SP_VisionGetRecvPayload(&payload))
                SP_VisionApplyRecvPayload(&payload);
        }
    }
}

/* ============================================================
 * DMA Idle 回调 —— 由 bsp_usart.c 的 HAL_UARTEx_RxEventCallback 触发
 * ============================================================ */
static void DecodeVisionCallback(void)
{
    if (sp_vision_use_vcp)
        return;

    uint16_t recv_size = USARTGetLastRecvSize(vision_usart_instance);
    if (recv_size == 0 || recv_size > vision_usart_instance->recv_buff_size)
        return;

    SP_VisionConsumeBytes(vision_usart_instance->recv_buff, recv_size);
}

static void DecodeVisionUSBCallback(uint16_t recv_len)
{
    if (!sp_vision_use_vcp || !vision_usb_rx_buffer)
        return;

    SP_VisionConsumeBytes(vision_usb_rx_buffer, recv_len);
}

/* ============================================================
 * 离线回调 —— 由 daemon.c 的 DaemonTask 调用
 * ============================================================ */
static void VisionOfflineCallback(void *id)
{
    (void)id;
    vision_online = 0;
    LOGWARNING("[sp_vision] vision offline, restart DMA receive.");
    if (!sp_vision_use_vcp && vision_usart_instance)
        USARTServiceInit(vision_usart_instance);
}

/* ============================================================
 * 发送底层 —— 由 SP_VisionSendFrame 调用
 * ============================================================ */
static void SP_VisionTxRaw(const uint8_t *data, uint16_t len)
{
    if (sp_vision_use_vcp) {
        USBTransmit((uint8_t *)data, len);
        return;
    }

    if (!vision_usart_instance)
        return;
    USARTSend(vision_usart_instance, (uint8_t *)data, len, USART_TRANSFER_IT);
}

/* ============================================================
 * 公开 API 实现
 * ============================================================ */

SP_Vision_RecvData_t *SP_VisionInit(void *uart_handle)
{
    /* 初始化发送载荷默认值 */
    memset(&send_payload, 0, sizeof(send_payload));
    memset(&recv_data, 0, sizeof(recv_data));
    send_payload.q[0] = 1.0f;
    SP_VisionResetParser();

    /* 注册 USART 实例 (DMA 接收 + Idle 中断) */
    USART_Init_Config_s usart_conf = {
        .module_callback  = DecodeVisionCallback,
        .recv_buff_size  = SP_VISION_RX_BUFFER_SIZE,
        .usart_handle    = (UART_HandleTypeDef *)uart_handle,
    };
    vision_usart_instance = USARTRegister(&usart_conf);

    USB_Init_Config_s usb_conf = {
        .tx_cbk = NULL,
        .rx_cbk = DecodeVisionUSBCallback,
    };
    vision_usb_rx_buffer = USBInit(usb_conf);

    /* 注册 daemon (10ms 周期 × 10 = 100ms 超时) */
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback,
        .owner_id     = vision_usart_instance,
        .reload_count = 10,  /* 10 × 10ms = 100ms */
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

SP_Vision_RecvData_t *SP_VisionGetRecvData(void)
{
    return &recv_data;
}

void SP_VisionSetTransport(uint8_t use_vcp)
{
    uint8_t next_transport = use_vcp ? 1U : 0U;

    if (sp_vision_use_vcp == next_transport)
        return;

    __disable_irq();
    sp_vision_use_vcp = next_transport;
    recv_data.data_updated = 0U;
    SP_VisionResetParser();
    __enable_irq();
}

uint8_t SP_VisionGetTransport(void)
{
    return sp_vision_use_vcp ? 1U : 0U;
}

uint8_t SP_VisionGetAndClearRecvData(SP_Vision_RecvData_t *out)
{
    uint8_t updated;

    if (!out)
        return 0;

    __disable_irq();
    updated = recv_data.data_updated;
    if (updated) {
        *out = recv_data;
        recv_data.data_updated = 0;
    }
    __enable_irq();

    return updated;
}

uint8_t SP_VisionIsOnline(void)
{
    return vision_online;
}

void SP_VisionDataConsumed(void)
{
    __disable_irq();
    recv_data.data_updated = 0;
    __enable_irq();
}

void SP_Vision_SetGimbalAttitude(float yaw, float pitch,
                                  float yaw_speed, float pitch_speed,
                                  float roll)
{
    send_payload.yaw         = yaw;
    send_payload.pitch       = pitch;
    send_payload.yaw_vel     = yaw_speed;
    send_payload.pitch_vel   = pitch_speed;
    (void)roll;
}

void SP_Vision_SetStatus(uint8_t mode, float bullet_speed, uint16_t bullet_count)
{
    send_payload.mode = mode;
    send_payload.bullet_speed = bullet_speed;
    send_payload.bullet_count = bullet_count;
}

void SP_Vision_SetQuaternion(const float *q)
{
    if (!q)
        return;
    memcpy(send_payload.q, q, sizeof(send_payload.q));
}

void SP_VisionSendFrame(const SP_Vision_SendFrame_t *frame)
{
    if (!frame)
        return;

    if (!sp_vision_use_vcp) {
        if (!vision_usart_instance)
            return;

        /* 检查串口是否就绪, 防止覆盖正在进行的 DMA 传输 */
        if (!USARTIsReady(vision_usart_instance))
            return;
    }

    tx_frame = *frame;
    SP_VisionTxRaw((const uint8_t *)&tx_frame, sizeof(tx_frame));
}

/**
 * @brief 组装并发送一帧 Gimbal → Vision 数据
 *
 * 调用方式:
 *   SP_Vision_SetGimbalAttitude(yaw, pitch, yaw_v, pitch_v, roll);
 *   SP_Vision_SetStatus(mode, bullet_speed, bullet_count);
 *   SP_VisionSend();
 */
void SP_VisionSend(void)
{
    /* 原子地复制待发送数据, 防止在中断中读写时被主循环修改 */
    SP_Vision_SendPayload_t payload_copy;
    __disable_irq();
    payload_copy = send_payload;
    __enable_irq();

    SP_Vision_SendFrame_t frame;
    frame.head[0] = SP_VISION_HEAD0;
    frame.head[1] = SP_VISION_HEAD1;
    frame.p = payload_copy;
    frame.tail[0] = SP_VISION_TAIL0;
    frame.tail[1] = SP_VISION_TAIL1;

    SP_VisionSendFrame(&frame);
}
