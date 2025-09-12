/*
* bsp_uart_servo.h
 *
 *  Created on: Mar 5, 2022
 *      Author: Administrator
 */

#ifndef BSP_UART_SERVO_H_
#define BSP_UART_SERVO_H_

#include "stdint.h"
#include "bsp_beep.h"

#define MEDIAN_VALUE         0
#define MID_VAL_ID6          4000

#define MID_ID5_MAX          3700
#define MID_ID5_MIN          380
// (uint16_t)((MID_ID5_MAX-MID_ID5_MIN)/3+MID_ID5_MIN)
#define MID_VAL_ID5          1486


#define RX_MAX_BUF           8
#define MAX_SERVO_NUM        6

// 限制串口舵机最大和最小脉冲输入值
// Limits the maximum and minimum pulse input values of the serial servo
#define MAX_PULSE            2341
#define MIN_PULSE            1659


void UartServo_Ctrl(uint8_t id, uint16_t value, uint16_t time);
void UartServo_Set_Snyc_Buffer(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6);
void UartServo_Sync_Write(uint16_t sync_time);
void UartServo_Set_Torque(uint8_t enable);
void UartServo_Set_ID(uint8_t id);


void UartServo_Get_Angle(uint8_t id);
void UartServo_Revice(uint8_t Rx_Temp);
uint8_t UartServo_Rx_Parse(void);



#endif /* BSP_UART_SERVO_H_ */
