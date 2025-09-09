#include "bsp.h"

#include "bsp_motion.h"
#include "bsp_pid.h"

// LED显示当前运行状态，每10毫秒调用一次，LED灯每200毫秒闪烁一次。
// The LED displays the current operating status, which is invoked every 10 milliseconds, and the LED blinks every 200 milliseconds.  
static void Bsp_Led_Show_State_Handle(void)
{
	static uint8_t led_count = 0;
	led_count++;
	if (led_count > 20)
	{
		led_count = 0;
	}
}

int car_state = 0;
// The peripheral device is initialized  外设设备初始化
void Bsp_Init(void)
{

	USART1_Init();
	Motor_Init();
	Encoder_Init();
	Motor_Init();
	PID_Param_Init();
}


// main.c中循环调用此函数，避免多次修改main.c文件。
// This function is called in a loop in main.c to avoid multiple modifications to the main.c file
void Bsp_Loop(void)
{

	SBUS_Handle();

}
