/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小
#define REF_PROTOCOL_HEADER_SIZE LEN_HEADER
#define REF_FRAME_MIN_SIZE (LEN_HEADER + LEN_CMDID + LEN_TAIL)
#define REF_FRAME_MAX_SIZE RE_RX_BUFFER_SIZE

typedef enum
{
	STEP_HEADER_SOF = 0,
	STEP_LENGTH_LOW,
	STEP_LENGTH_HIGH,
	STEP_FRAME_SEQ,
	STEP_HEADER_CRC8,
	STEP_DATA_CRC16,
} RefereeUnpackStep_e;

typedef struct
{
	uint8_t protocol_packet[REF_FRAME_MAX_SIZE];
	uint16_t data_len;
	uint16_t index;
	RefereeUnpackStep_e unpack_step;
} RefereeUnpackObj_t;

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据
static RefereeUnpackObj_t referee_unpack_obj;

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadFrameData(uint8_t *buff)
{
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 2个8位拼成16位int
		referee_info.CmdID = (buff[6] << 8 | buff[5]);
		// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
		// 第8个字节开始才是数据 data=7
		switch (referee_info.CmdID)
		{
		case ID_game_state: // 0x0001
			memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
			break;
		case ID_game_result: // 0x0002
			memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
			break;
		case ID_game_robot_survivors: // 0x0003
			memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
			break;
		case ID_event_data: // 0x0101
			memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
			break;
		case ID_supply_projectile_action: // 0x0102
			memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
			break;
		case ID_game_robot_state: // 0x0201
			memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
			break;
		case ID_power_heat_data: // 0x0202
			memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
			break;
		case ID_game_robot_pos: // 0x0203
			memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
			break;
		case ID_buff_musk: // 0x0204
			memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
			break;
		case ID_aerial_robot_energy: // 0x0205
			memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
			break;
		case ID_robot_hurt: // 0x0206
			memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
			break;
		case ID_shoot_data: // 0x0207
			memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
			break;
		case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
			memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
			break;
		default:
			break;
		}
	}
}

static void JudgeReadData(uint8_t *buff, uint16_t len)
{
	if (buff == NULL || len == 0)
		return;

	for (uint16_t i = 0; i < len; i++)
	{
		uint8_t byte = buff[i];
		switch (referee_unpack_obj.unpack_step)
		{
		case STEP_HEADER_SOF:
			if (byte == REFEREE_SOF)
			{
				referee_unpack_obj.unpack_step = STEP_LENGTH_LOW;
				referee_unpack_obj.index = 0;
				referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
			}
			break;

		case STEP_LENGTH_LOW:
			referee_unpack_obj.data_len = byte;
			referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
			referee_unpack_obj.unpack_step = STEP_LENGTH_HIGH;
			break;

		case STEP_LENGTH_HIGH:
			referee_unpack_obj.data_len |= ((uint16_t)byte << 8);
			referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;

			if (referee_unpack_obj.data_len <= (REF_FRAME_MAX_SIZE - REF_FRAME_MIN_SIZE))
			{
				referee_unpack_obj.unpack_step = STEP_FRAME_SEQ;
			}
			else
			{
				referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
				referee_unpack_obj.index = 0;
			}
			break;

		case STEP_FRAME_SEQ:
			referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
			referee_unpack_obj.unpack_step = STEP_HEADER_CRC8;
			break;

		case STEP_HEADER_CRC8:
			referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
			if (referee_unpack_obj.index == REF_PROTOCOL_HEADER_SIZE)
			{
				if (Verify_CRC8_Check_Sum(referee_unpack_obj.protocol_packet, REF_PROTOCOL_HEADER_SIZE) == TRUE)
				{
					referee_unpack_obj.unpack_step = STEP_DATA_CRC16;
				}
				else
				{
					referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
					referee_unpack_obj.index = 0;
				}
			}
			break;

		case STEP_DATA_CRC16:
		{
			uint16_t frame_len = REF_FRAME_MIN_SIZE + referee_unpack_obj.data_len;
			if (referee_unpack_obj.index < frame_len)
			{
				referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
			}

			if (referee_unpack_obj.index >= frame_len)
			{
				referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
				referee_unpack_obj.index = 0;

				if (Verify_CRC16_Check_Sum(referee_unpack_obj.protocol_packet, frame_len) == TRUE)
				{
					JudgeReadFrameData(referee_unpack_obj.protocol_packet);
				}
			}
		}
		break;

		default:
			referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
			referee_unpack_obj.index = 0;
			break;
		}
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff, USARTGetLastRecvSize(referee_usart_instance));
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);
	memset(&referee_unpack_obj, 0, sizeof(referee_unpack_obj));
	referee_unpack_obj.unpack_step = STEP_HEADER_SOF;

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}
