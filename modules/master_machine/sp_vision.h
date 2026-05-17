/**
 * @file sp_vision.h
 * @brief 视觉串口通信模块 —— 基于 sp_vision 协议
 *
 * 协议格式 (Vision → Gimbal, 接收):
 *   HEAD  : 0x5A 0xA5
 *   载荷   : 25 字节 (小端序 float)
 *   TAIL  : 0x7F 0xFE
 *   总帧长 : 29 字节
 *
 * 协议格式 (Gimbal → Vision, 发送):
 *   HEAD  : 0x5A 0xA5
 *   载荷   : 39 字节 (小端序 float)
 *   TAIL  : 0x7F 0xFE
 *   总帧长 : 43 字节
 *
 * 解析器为字节级状态机，对噪声有较强鲁棒性。
 */
#ifndef SP_VISION_H
#define SP_VISION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

/* 帧定界符 */
#define SP_VISION_HEAD0  0x5AU
#define SP_VISION_HEAD1  0xA5U
#define SP_VISION_TAIL0  0x7FU
#define SP_VISION_TAIL1  0xFEU

/* 帧各部分字节数 */
#define SP_VISION_FRAME_HEAD_SIZE   2U
#define SP_VISION_FRAME_TAIL_SIZE   2U
#define SP_VISION_RECV_PAYLOAD_SIZE 25U  /* VisionToGimbal payload */
#define SP_VISION_SEND_PAYLOAD_SIZE 39U  /* GimbalToVision payload */
#define SP_VISION_RECV_FRAME_SIZE   (SP_VISION_FRAME_HEAD_SIZE + SP_VISION_RECV_PAYLOAD_SIZE + SP_VISION_FRAME_TAIL_SIZE)  /* 29 */
#define SP_VISION_SEND_FRAME_SIZE   (SP_VISION_FRAME_HEAD_SIZE + SP_VISION_SEND_PAYLOAD_SIZE + SP_VISION_FRAME_TAIL_SIZE)  /* 43 */

/* ===================== 接收帧结构 (Vision → Gimbal) ===================== */
/**
 * @brief 视觉接收载荷 —— 从上位机/sp_vision 发来的数据
 *        帧格式: HEAD(2) + payload(25) + TAIL(2) = 29 字节
 */
typedef struct {
    uint8_t mode;       /* 控制模式: 0=不控制, 1=控制云台不开火, 2=控制云台且开火 */
    float yaw;          /* 偏航目标角, 弧度 */
    float yaw_vel;      /* 偏航角速度, rad/s */
    float yaw_acc;      /* 偏航角加速度, rad/s^2 */
    float pitch;        /* 俯仰目标角, 弧度 */
    float pitch_vel;    /* 俯仰角速度, rad/s */
    float pitch_acc;    /* 俯仰角加速度, rad/s^2 */
} SP_Vision_RecvPayload_t;

typedef struct {
    uint8_t head[2];
    SP_Vision_RecvPayload_t p;
    uint8_t tail[2];
} SP_Vision_RecvFrame_t;

/* ===================== 发送帧结构 (Gimbal → Vision) ===================== */
/**
 * @brief 视觉发送载荷 —— 回传给上位机/sp_vision 的云台状态
 *        帧格式: HEAD(2) + payload(39) + TAIL(2) = 43 字节
 */
typedef struct {
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
} SP_Vision_SendPayload_t;

typedef struct {
    uint8_t head[2];
    SP_Vision_SendPayload_t p;
    uint8_t tail[2];
} SP_Vision_SendFrame_t;

#pragma pack(pop)

/* ===================== 接收数据外部接口 ===================== */
typedef struct {
    uint8_t control;          /* 是否控制云台 */
    uint8_t shoot;            /* 是否开火 */
    float yaw;                /* 偏航角增量, 弧度 */
    float pitch;              /* 俯仰角增量, 弧度 */
    float yaw_speed;          /* 偏航角速度, rad/s */
    float pitch_speed;        /* 俯仰角速度, rad/s */
    float yaw_acc;            /* 偏航角加速度, rad/s^2 */
    float pitch_acc;          /* 俯仰角加速度, rad/s^2 */
    uint8_t mode;             /* 原始模式值 */
    volatile uint8_t data_updated;  /* 数据更新标志 */
} SP_Vision_RecvData_t;

#ifdef __cplusplus
static_assert(sizeof(SP_Vision_RecvPayload_t) == SP_VISION_RECV_PAYLOAD_SIZE, "SP vision recv payload size mismatch");
static_assert(sizeof(SP_Vision_SendPayload_t) == SP_VISION_SEND_PAYLOAD_SIZE, "SP vision send payload size mismatch");
static_assert(sizeof(SP_Vision_RecvFrame_t) == SP_VISION_RECV_FRAME_SIZE, "SP vision recv frame size mismatch");
static_assert(sizeof(SP_Vision_SendFrame_t) == SP_VISION_SEND_FRAME_SIZE, "SP vision send frame size mismatch");
#else
_Static_assert(sizeof(SP_Vision_RecvPayload_t) == SP_VISION_RECV_PAYLOAD_SIZE, "SP vision recv payload size mismatch");
_Static_assert(sizeof(SP_Vision_SendPayload_t) == SP_VISION_SEND_PAYLOAD_SIZE, "SP vision send payload size mismatch");
_Static_assert(sizeof(SP_Vision_RecvFrame_t) == SP_VISION_RECV_FRAME_SIZE, "SP vision recv frame size mismatch");
_Static_assert(sizeof(SP_Vision_SendFrame_t) == SP_VISION_SEND_FRAME_SIZE, "SP vision send frame size mismatch");
#endif

/* ===================== 运行时 API ===================== */

/**
 * @brief 初始化视觉串口通信
 *
 * @param uart_handle HAL 串口句柄 (C板一般传 &huart1)
 * @return SP_Vision_RecvData_t* 接收数据结构体指针, 用于读取视觉数据
 */
SP_Vision_RecvData_t *SP_VisionInit(void *uart_handle);

/**
 * @brief 发送云台状态数据给上位机
 *
 *        实际发送由 bsp_usart 层的 IT 模式完成.
 */
void SP_VisionSendFrame(const SP_Vision_SendFrame_t *frame);

extern volatile uint8_t sp_vision_use_vcp;

void SP_VisionSetTransport(uint8_t use_vcp);

uint8_t SP_VisionGetTransport(void);

/**
 * @brief 设置待发送帧的云台姿态数据
 *
 * @param yaw        偏航角, 弧度
 * @param pitch      俯仰角, 弧度
 * @param yaw_speed  偏航角速度, rad/s
 * @param pitch_speed 俯仰角速度, rad/s
 * @param roll       横滚角, 弧度
 */
void SP_Vision_SetGimbalAttitude(float yaw, float pitch,
                                  float yaw_speed, float pitch_speed,
                                  float roll);

/**
 * @brief 设置待发送帧的机器人状态
 *
 * @param mode          视觉模式 (0=空闲, 1=自瞄, 2=小符, 3=大符)
 * @param bullet_speed  弹速, m/s
 * @param bullet_count  剩余弹量
 */
void SP_Vision_SetStatus(uint8_t mode, float bullet_speed, uint16_t bullet_count);

void SP_Vision_SetQuaternion(const float *q);

void SP_VisionSend(void);

/**
 * @brief 获取最近一次接收到的视觉数据
 *
 * @return SP_Vision_RecvData_t* 接收数据结构体指针
 */
SP_Vision_RecvData_t *SP_VisionGetRecvData(void);

uint8_t SP_VisionGetAndClearRecvData(SP_Vision_RecvData_t *out);

/**
 * @brief 判断视觉通信是否在线
 *
 * @return uint8_t 1=在线, 0=离线
 */
uint8_t SP_VisionIsOnline(void);

/**
 * @brief 标记视觉数据已被消费, 防止同一帧被重复处理
 */
void SP_VisionDataConsumed(void);

#ifdef __cplusplus
}
#endif

#endif /* SP_VISION_H */
