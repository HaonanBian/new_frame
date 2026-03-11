#ifndef INFANTRY_PROTOCOL_H
#define INFANTRY_PROTOCOL_H

#include <stdint.h>

#define INFANTRY_FRAME_LEN 16u
#define INFANTRY_FRAME_HEAD 0xFFu
#define INFANTRY_FRAME_TAIL 0x0Du

typedef struct
{
    uint8_t fire;
    float pitch_diff;
    float yaw_diff;
    float distance;
} Infantry_Cmd_Packet_s;

typedef struct
{
    uint8_t mode;
    float roll;
    float pitch;
    float yaw;
} Infantry_Feedback_Packet_s;

/**
 * @brief 解析上位机下发的 16 字节步兵协议帧
 *
 * @param rx_buf 原始接收缓存
 * @param rx_len 缓存有效长度
 * @param cmd_out 解析输出
 * @return uint8_t 1:成功 0:失败
 */
uint8_t InfantryProtocolDecodeCmd(const uint8_t *rx_buf, uint16_t rx_len, Infantry_Cmd_Packet_s *cmd_out);

/**
 * @brief 打包下位机上报给上位机的 16 字节步兵协议帧
 *
 * @param feedback 待发送反馈
 * @param tx_buf 输出缓存(长度至少16)
 */
void InfantryProtocolEncodeFeedback(const Infantry_Feedback_Packet_s *feedback, uint8_t *tx_buf);

#endif // INFANTRY_PROTOCOL_H
