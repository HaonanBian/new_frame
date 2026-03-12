#include "infantry_protocol.h"

#include <math.h>
#include <string.h>

static uint8_t is_valid_frame(const uint8_t *frame)
{
    return (frame[0] == INFANTRY_FRAME_HEAD) &&
           (frame[INFANTRY_FRAME_LEN - 2] == INFANTRY_CMD_RESERVED) &&
           (frame[INFANTRY_FRAME_LEN - 1] == INFANTRY_FRAME_TAIL);
}

static uint8_t is_reasonable_cmd(const Infantry_Cmd_Packet_s *cmd)
{
    if (cmd->fire > 1u)
        return 0;

    if (!isfinite(cmd->pitch_diff) || !isfinite(cmd->yaw_diff) || !isfinite(cmd->distance))
        return 0;

    if (fabsf(cmd->pitch_diff) > 90.0f || fabsf(cmd->yaw_diff) > 180.0f)
        return 0;

    if (cmd->distance < 0.0f || cmd->distance > 100.0f)
        return 0;

    return 1;
}

uint8_t InfantryProtocolDecodeCmd(const uint8_t *rx_buf, uint16_t rx_len, Infantry_Cmd_Packet_s *cmd_out)
{
    if (rx_buf == NULL || cmd_out == NULL || rx_len < INFANTRY_FRAME_LEN)
        return 0;

    for (uint16_t i = 0; (uint16_t)(i + INFANTRY_FRAME_LEN) <= rx_len; ++i)
    {
        const uint8_t *frame = rx_buf + i;
        if (!is_valid_frame(frame))
            continue;

        Infantry_Cmd_Packet_s tmp_cmd;
        tmp_cmd.fire = frame[1];
        memcpy(&tmp_cmd.pitch_diff, frame + 2, sizeof(float));
        memcpy(&tmp_cmd.yaw_diff, frame + 6, sizeof(float));
        memcpy(&tmp_cmd.distance, frame + 10, sizeof(float));

        if (!is_reasonable_cmd(&tmp_cmd))
            continue;

        *cmd_out = tmp_cmd;
        return 1;
    }

    return 0;
}

void InfantryProtocolEncodeFeedback(const Infantry_Feedback_Packet_s *feedback, uint8_t *tx_buf)
{
    if (feedback == NULL || tx_buf == NULL)
        return;

    memset(tx_buf, 0, INFANTRY_FRAME_LEN);
    tx_buf[0] = INFANTRY_FRAME_HEAD;
    tx_buf[1] = feedback->mode;
    memcpy(tx_buf + 2, &feedback->roll, sizeof(float));
    memcpy(tx_buf + 6, &feedback->pitch, sizeof(float));
    memcpy(tx_buf + 10, &feedback->yaw, sizeof(float));
    tx_buf[INFANTRY_FRAME_LEN - 2] = INFANTRY_FEEDBACK_RESERVED;
    tx_buf[INFANTRY_FRAME_LEN - 1] = INFANTRY_FRAME_TAIL;
}
