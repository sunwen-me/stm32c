/*
* bsp_sbus.h
 *
 *  Created on: Mar 8, 2022
 *      Author: Administrator
 */

#ifndef BSP_SBUS_H_
#define BSP_SBUS_H_

#include "stdint.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_ALL_CHANNELS       0x00

typedef enum {
 CTRL_MODE_RC,   // 遥控优先
 CTRL_MODE_PC    // 上位机优先
} ctrl_mode_t;

extern ctrl_mode_t g_ctrl_mode;

void SBUS_Reveive(uint8_t data);
void SBUS_Handle(void);


#endif /* BSP_SBUS_H_ */
