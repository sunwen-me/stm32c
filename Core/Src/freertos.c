/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "bsp.h"
#include "bsp_motion.h"
#include "bsp_uart_servo.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_BUFFER_QUEUE (5)
#define UART_BUFFER_SIZE (256)
extern car_data_t car_data;
extern car_data_t car_data_ta;
UART_RX_TypeDef uart_rx_data_t[UART_BUFFER_QUEUE];
extern uint8_t uart_buff_ctrl;
extern uint16_t Servo_Ctrl;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for reportTask */
osThreadId_t reportTaskHandle;
const osThreadAttr_t reportTask_attributes = {
  .name = "reportTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for receiveTask */
osThreadId_t receiveTaskHandle;
const osThreadAttr_t receiveTask_attributes = {
  .name = "receiveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uart1RxQueue */
osMessageQueueId_t uart1RxQueueHandle;
const osMessageQueueAttr_t uart1RxQueue_attributes = {
  .name = "uart1RxQueue"
};
/* Definitions for sbusQueue */
osMessageQueueId_t sbusQueueHandle;
const osMessageQueueAttr_t sbusQueue_attributes = {
  .name = "sbusQueue"
};
/* Definitions for reportMutex */
osMutexId_t reportMutexHandle;
const osMutexAttr_t reportMutex_attributes = {
  .name = "reportMutex"
};
/* Definitions for revMutex */
osMutexId_t revMutexHandle;
const osMutexAttr_t revMutex_attributes = {
  .name = "revMutex"
};
/* Definitions for servoMutex */
osMutexId_t servoMutexHandle;
const osMutexAttr_t servoMutex_attributes = {
  .name = "servoMutex"
};
/* Definitions for servocMutex */
osMutexId_t servocMutexHandle;
const osMutexAttr_t servocMutex_attributes = {
  .name = "servocMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotionHandleTask(void *argument);
void ReportHandleTask(void *argument);
void ReceiveHandleTask(void *argument);
void ServoHandleTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of reportMutex */
  reportMutexHandle = osMutexNew(&reportMutex_attributes);

  /* creation of revMutex */
  revMutexHandle = osMutexNew(&revMutex_attributes);

  /* creation of servoMutex */
  servoMutexHandle = osMutexNew(&servoMutex_attributes);

  /* creation of servocMutex */
  servocMutexHandle = osMutexNew(&servocMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart1RxQueue */
  uart1RxQueueHandle = osMessageQueueNew (5, sizeof(void*), &uart1RxQueue_attributes);

  /* creation of sbusQueue */
  sbusQueueHandle = osMessageQueueNew (25, 25, &sbusQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(MotionHandleTask, NULL, &controlTask_attributes);

  /* creation of reportTask */
  reportTaskHandle = osThreadNew(ReportHandleTask, NULL, &reportTask_attributes);

  /* creation of receiveTask */
  receiveTaskHandle = osThreadNew(ReceiveHandleTask, NULL, &receiveTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(ServoHandleTask, NULL, &ServoTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  Bsp_Init();
  /* Infinite loop */
  for(;;)
  {
    Bsp_Loop();

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotionHandleTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotionHandleTask */
void MotionHandleTask(void *argument)
{
  /* USER CODE BEGIN MotionHandleTask */
  /* Infinite loop */
  for(;;)
  {
    Motion_Handle();
    osDelay(10);
  }
  /* USER CODE END MotionHandleTask */
}

/* USER CODE BEGIN Header_ReportHandleTask */
/**
* @brief Function implementing the reportTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReportHandleTask */
void ReportHandleTask(void *argument)
{
  /* USER CODE BEGIN ReportHandleTask */
  /* Infinite loop */
  // for(;;){    osDelay(10);};
  for(;;)
  {
    car_data_t temp;
    if (osMutexAcquire(reportMutexHandle, 10) == osOK)
    {
      temp = car_data; // 先拷贝
      osMutexRelease(reportMutexHandle); // 立刻释放
    }

    // 封装数据为协议帧
    uint8_t buf[11];
    int16_t vx_i = (int16_t)temp.Vx;
    int16_t vy_i = (int16_t)temp.Vy;
    int16_t vz_i = (int16_t)(temp.Vz*1000);

    buf[0] = 0xAA;      // SOF
    buf[1] = 6;         // 数据长度
    buf[2] = 0x10;      // CMD
    buf[3] = vx_i & 0xFF;
    buf[4] = (vx_i >> 8) & 0xFF;
    buf[5] = vy_i & 0xFF;
    buf[6] = (vy_i >> 8) & 0xFF;
    buf[7] = vz_i & 0xFF;
    buf[8] = (vz_i >> 8) & 0xFF;
    buf[9] = buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6] ^ buf[7] ^ buf[8]; // 简单 CRC
    buf[10] = 0x55;     // EOF
    USART1_Send_ArrayU8(buf, 11);
    osDelay(20);
  }
  /* USER CODE END ReportHandleTask */
}

/* USER CODE BEGIN Header_ReceiveHandleTask */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveHandleTask */
void ReceiveHandleTask(void *argument)
{
  /* USER CODE BEGIN ReceiveHandleTask */
  /* Infinite loop */
  uint8_t byte;
  uint8_t buf[20];  // 缓冲解析帧
  uint8_t idx = 0;
  HAL_UART_Receive_DMA(&huart1 ,uart_rx_data_t[uart_buff_ctrl].buffer,UART_BUFFER_SIZE);
  UART_RX_TypeDef *RecvUartData;//定义一个结构体指针，用来存储接收的数据的.buffer和.size
  enum FrameState {
    WAIT_SOF,
    WAIT_LEN,
    WAIT_CMD,
    WAIT_DATA,
    WAIT_CRC,
    WAIT_EOF
} state = WAIT_SOF;

  uint8_t len = 0, cmd = 0, crc = 0;
  uint8_t data[6];
  uint8_t data_idx = 0;
    int16_t vx,vy,vz;
for (;;)
    {
        // 阻塞读取队列，RecvUartData 是指向 DMA 缓冲区的指针
        if (xQueueReceive(uart1RxQueueHandle, &RecvUartData, portMAX_DELAY) == pdTRUE)
        {
            for (int i = 0; i < RecvUartData->size; i++)
            {
                uint8_t byte = RecvUartData->buffer[i];

                switch (state)
                {
                    case WAIT_SOF:
                        if (byte == 0xAA)  // SOF
                        {
                            state = WAIT_LEN;
                            crc = 0;
                        }
                        break;

                    case WAIT_LEN:
                        len = byte;
                        crc ^= len;
                        state = WAIT_CMD;
                        break;

                    case WAIT_CMD:
                        cmd = byte;
                        crc ^= cmd;
                        data_idx = 0;
                        if (len == 7 && cmd == 0x20)  // CMD_CTRL
                            state = WAIT_DATA;
                        else
                            state = WAIT_SOF;  // 异常帧
                        break;

                    case WAIT_DATA:
                        data[data_idx++] = byte;
                        crc ^= byte;
                        if (data_idx >= 6)
                            state = WAIT_CRC;
                        break;

                    case WAIT_CRC:
                        if (byte == crc)
                            state = WAIT_EOF;
                        else
                            state = WAIT_SOF;  // CRC错误
                        break;

                    case WAIT_EOF:
                        if (byte == 0x55)  // EOF
                        {
                            vx = data[0] | (data[1] << 8);
                           vy = data[2] | (data[3] << 8);
                            vz = data[4] | (data[5] << 8);

                            if (g_ctrl_mode == CTRL_MODE_PC) {


                                Motion_Ctrl(vx, vy, vz); // 上位机控制才执行
                            }
                        }
                        state = WAIT_SOF;
                        break;
                }
            }
        }
    }
  /* USER CODE END ReceiveHandleTask */
}

/* USER CODE BEGIN Header_ServoHandleTask */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoHandleTask */
void ServoHandleTask(void *argument)
{
  /* USER CODE BEGIN ServoHandleTask */
  /* Infinite loop */
    uint8_t servo_id = 0x01;
  for(;;)
  {
      if (osMutexAcquire(servocMutexHandle, 1) == osOK)
      {
          UartServo_Ctrl(servo_id, Servo_Ctrl, 1000);
          osMutexRelease(servocMutexHandle);
      }
      osDelay(10);
      UartServo_Get_Angle(servo_id);
      osDelay(10);
      UartServo_Rx_Parse();
      osDelay(10);
  }
  /* USER CODE END ServoHandleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

