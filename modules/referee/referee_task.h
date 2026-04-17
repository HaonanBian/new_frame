#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 裁判系统数据初始化（不包含UI）
 *        适用于不需要UI显示，但需要获取裁判系统功率/热量数据的场景
 *
 * @param referee_usart_handle 裁判系统串口句柄
 * @return referee_info_t* 裁判系统数据指针
 */
referee_info_t *RefereeDataInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit();

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask();

/**
 * @brief 获取裁判系统数据指针，供其他模块使用
 *
 * @return referee_info_t* 裁判系统数据指针
 */
referee_info_t *GetRefereeData(void);

#endif // REFEREE_H
