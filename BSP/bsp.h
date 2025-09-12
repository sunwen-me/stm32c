#ifndef __BSP_H__
#define __BSP_H__

/* Import HAL related library  导入HAL相关库 */
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "tim.h"
#include "stm32f1xx_hal.h"
#include "stm32f103xe.h"


/* Import device driver library  导入设备驱动库 */
#include "bsp_uart.h"
#include "bsp_sbus.h"
#include "bsp_pwmServo.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "stdio.h"


/* DEFINE */
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_5
#define BEEP_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOD
#define LED_ON()         HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET)
#define LED_OFF()        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET)
#define LED_TOGGLE()     HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)


/* functions */
void Bsp_Init(void);
void Bsp_Loop(void);


#endif /* __BSP_H__ */
