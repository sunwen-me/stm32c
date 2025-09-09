/*
* bsp_uart.h
 *
 *  Created on: Mar 4, 2022
 *      Author: Administrator
 */

#ifndef BSP_UART_H_
#define BSP_UART_H_

#include "stdint.h"
#define UART_BUFFER_QUEUE (5)
#define UART_BUFFER_SIZE (256)


void USART1_Init(void);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void USART1_DMAHandler(void);
typedef struct {
 uint8_t buffer[UART_BUFFER_SIZE];
 uint16_t size;

}UART_RX_TypeDef;

#endif /* BSP_UART_H_ */
