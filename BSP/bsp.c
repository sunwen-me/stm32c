#include "bsp.h"

#include "bsp_beep.h"
#include "bsp_key.h"
#include "bsp_uart_servo.h"
#include "bsp_motion.h"
#include "bsp_pid.h"
#include "cmsis_os2.h"
extern osMutexId_t servocMutexHandle;
extern uint16_t Servo_Ctrl;
// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.  
static void Bsp_Led_Show_State_Handle(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count > 20)
	{
		led_count = 0;
		LED_TOGGLE();
	}
}

int car_state = 0;
// The peripheral device is initialized  外设设备初始化
void Bsp_Init(void)
{

	USART1_Init();
	USART3_Init();
	Motor_Init();
	Encoder_Init();
	Motor_Init();
	PID_Param_Init();
}


// main.c中循环调用此函数，避免多次修改main.c文件。
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file
uint8_t servo_id = 0x01;
uint8_t servo_state = 0;
void Bsp_Loop(void)
{

	SBUS_Handle();
	// UartServo_Get_Angle(0x01);
	// osDelay(12);
	// UartServo_Ctrl(0x01,4000, 1000);
	// UartServo_Rx_Parse();
	// osDelay(10);

		// static uint8_t direction = 0;  // 0: 正向, 1: 反向
		// static uint16_t target_angle = 1000;  // 初始目标角度
	 //
		// // 1. 控制舵机转动
		// UartServo_Ctrl(0x01, target_angle, 0);  // 100ms 内完成运动
	 //  UartServo_Rx_Parse();
		// // 2. 切换方向
		// if (direction == 0)
		// {
		// 	UartServo_Get_Angle(0x01);
		// 	target_angle += 1000;  // 每次增加 1000
		// 	if (target_angle >= 4000)
		// 		direction = 1;     // 到达上限后反向
		// }
		// else
		// {
		// 	target_angle -= 1000;  // 每次减少 1000
		// 	if (target_angle <= 1000)
		// 		direction = 0;     // 到达下限后反向
		// }
	// if (Key1_State(KEY_MODE_ONE_TIME))
	// {
	// 	//Beep_On_Time(50);
	// 	static int press = 0;
	// 	press++;
	//
	// 	//UartServo_Get_Angle(servo_id);
	// 	//osDelay(12);
	// 	if (press%2)
	// 	{
	// 		UartServo_Ctrl(servo_id, 2000, 1000);
	// 	}
	// 	else
	// 	{
	// 		UartServo_Ctrl(servo_id, 3000, 500);
	// 	}
	// }
	// if (servo_state == 0)
	// {
	// 	if (osMutexAcquire(servocMutexHandle, 1) == osOK)
	// 	{
	// 		Servo_Ctrl=0;
	// 		osMutexRelease(servocMutexHandle);
	// 	}
	// 	servo_state =1;
	// }
	// else if (servo_state == 1)
	// {
	// 	if (osMutexAcquire(servocMutexHandle, 1) == osOK)
	// 	{
	// 		Servo_Ctrl=2000;
	// 		osMutexRelease(servocMutexHandle);
	// 	}
	// 	servo_state =0;
	// }
	//UartServo_Rx_Parse();
	//Beep_Timeout_Close_Handle();
	osDelay(10);

}
