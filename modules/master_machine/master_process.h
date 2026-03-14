#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include <stdint.h>

#define VISION_FRAME_HEAD_0 0x5Au
#define VISION_FRAME_HEAD_1 0xA5u
#define VISION_FRAME_TAIL_0 0x7Fu
#define VISION_FRAME_TAIL_1 0xFEu

#define VISION_RECV_SIZE 29u
#define VISION_SEND_SIZE 43u

#pragma pack(1)
typedef enum
{
	VISION_MODE_IDLE = 0,
	VISION_MODE_AUTO_AIM = 1,
	VISION_MODE_SMALL_BUFF = 2,
	VISION_MODE_BIG_BUFF = 3,
} Vision_Mode_e;

typedef struct
{
	uint8_t control;
	uint8_t shoot;
	float yaw;
	float yaw_vel;
	float yaw_acc;
	float pitch;
	float pitch_vel;
	float pitch_acc;
	volatile uint8_t data_updated;
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	uint8_t mode;
	float q[4];
	float yaw;
	float yaw_vel;
	float pitch;
	float pitch_vel;
	float bullet_speed;
	uint16_t bullet_count;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置发送数据的四元数
 */
void VisionSetQuaternion(const float *q);

/**
 * @brief 设置发送数据的姿态和角速度
 */
void VisionSetAltitude(float yaw, float pitch, float yaw_vel, float pitch_vel);

/**
 * @brief 设置发送状态
 */
void VisionSetStatus(Vision_Mode_e mode, float bullet_speed, uint16_t bullet_count);

/**
 * @brief 判断视觉通信是否在线
 */
uint8_t VisionIsOnline(void);

/**
 * @brief 标记视觉数据已被消费,防止diff值被重复累加
 */
void VisionDataConsumed(void);
#endif // !MASTER_PROCESS_H
