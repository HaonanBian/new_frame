//
// Created by bismarckkk on 2024/2/17.
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include <stdio.h>
#include "ui_types.h"
#include "rm_referee.h"

extern int ui_self_id;

void print_message(const uint8_t* message, int length);
void ui_request_refresh(void);
uint8_t ui_take_refresh_request(void);

// User Code Begin
#define SEND_MESSAGE(message, length) RefereeSend((uint8_t *)(message), (uint16_t)(length))
// User Code End

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);

#endif //UI_INTERFACE_H
