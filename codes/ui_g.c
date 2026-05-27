//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_2_frame_t ui_g_Ungroup_0;

ui_interface_round_t *ui_g_Ungroup_5m = (ui_interface_round_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_round_t *ui_g_Ungroup_2m = (ui_interface_round_t*)&(ui_g_Ungroup_0.data[1]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_5m->figure_type = 2;
    ui_g_Ungroup_5m->operate_type = 1;
    ui_g_Ungroup_5m->layer = 0;
    ui_g_Ungroup_5m->color = 6;
    ui_g_Ungroup_5m->start_x = 982;
    ui_g_Ungroup_5m->start_y = 509;
    ui_g_Ungroup_5m->width = 3;
    ui_g_Ungroup_5m->r = 10;

    ui_g_Ungroup_2m->figure_type = 2;
    ui_g_Ungroup_2m->operate_type = 1;
    ui_g_Ungroup_2m->layer = 0;
    ui_g_Ungroup_2m->color = 4;
    ui_g_Ungroup_2m->start_x = 1004;
    ui_g_Ungroup_2m->start_y = 535;
    ui_g_Ungroup_2m->width = 3;
    ui_g_Ungroup_2m->r = 10;


    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}


void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
}

