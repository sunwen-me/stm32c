/*
* bsp_uart.c
 *
 *  Created on: Mar 4, 2022
 *      Author: Administrator
 */

#include "bsp_uart.h"
#include "bsp.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "queue.h"
#define ENABLE_UART_DMA    1

uint8_t RxTemp = 0;
uint8_t RxTemp_2 = 0;
extern UART_RX_TypeDef uart_rx_data_t[UART_BUFFER_QUEUE];
uint8_t uart_buff_ctrl=0;
extern osMessageQueueId_t uart1RxQueueHandle;
extern DMA_HandleTypeDef hdma_usart1_rx;

void USART1_DMAHandler(void){
    //创建一个DMA函数，一旦串口产生中断，则执行该函数，
    //所以该函数要写在USART1_IRQHandler函数当中，一旦上位机发送信号触发，执行USART1_IRQHandler函数，即可执行到该函数

    if(__HAL_UART_GET_FLAG(&huart1 ,UART_FLAG_IDLE)!=RESET){
        //判断是否发生空闲中断，此时既然能进入该函数，一定已经发生了空闲中断

        UART_RX_TypeDef *Data;//再定义一个这个结构体，包含的东西与uart_rx_data_t一致，我们要将uart_rx_data_t中的数据传到Data指针当中再一个一个传输

        __HAL_UART_CLEAR_IDLEFLAG(&huart1);//清楚空闲中断标志，方便下次判断是否空闲中断
        HAL_UART_DMAStop(&huart1 );//停止DMA传输，关闭DMA

        uart_rx_data_t[uart_buff_ctrl].size = UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        //从DMA的结构体中找到这个数据长度（单片机接收的数据）；注意这里的__HAL_DMA_GET_COUNTER函数计算的是未发送的数据长度，所以要用总长度UART_BUFFER_SIZE减一下
        //实际上算出来的这个长度，不用管它具体是多少，减完之后肯定大于.size里实际传输的长度
        //uart_rx_data_t[uart_buff_ctrl].size的意思是uart_rx_data_t这个结构体中的第uart_buff_ctrl个数组中的size项的值

        Data = &uart_rx_data_t[uart_buff_ctrl];
        //第一次传输时，uart_buff_ctrl = 0，传的是结构体中的第一个数组的意思，之后每次接收时uart_buff_ctrl都自加，分别传到不同的结构体数组当中，传到第五个时，uart_buff_ctrl又要重新取0，达到环形
        //将这个Data指针地址发送至队列，让队列存一个地址

        xQueueSendFromISR(uart1RxQueueHandle,&Data,NULL);

        uart_buff_ctrl++;//自加

        uart_buff_ctrl %= UART_BUFFER_QUEUE; //再取余，让它计到5时重新归零，当然这里也可以直接使用if语句

        HAL_UART_Receive_DMA(&huart1 , uart_rx_data_t[uart_buff_ctrl].buffer,UART_BUFFER_SIZE);//重新开启DMA传输，传输buffer。这个函数在任务当中也要用到



    }

}
// Initialize USART1  初始化串口1
void USART1_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart1 ,UART_IT_IDLE);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxTemp_2, 1);

    //printf("start serial\n");
}

// The serial port sends one byte  串口发送一个字节
void USART1_Send_U8(uint8_t ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
}

// The serial port sends a string of data  串口发送一串数据
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
#if ENABLE_UART_DMA
    HAL_UART_Transmit_DMA(&huart1, BufferPtr, Length);
#else
    while (Length--)
    {
        USART1_Send_U8(*BufferPtr);
        BufferPtr++;
    }
#endif
}

// The serial port receiving is interrupted. Procedure  串口接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    // if (huart == &huart1)
    // {
    //     osMessageQueuePut(uart1RxQueueHandle, &RxTemp, 0, 0);
    //     HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxTemp, 1);
    // }
    if (huart == &huart2)
    {
        SBUS_Reveive(RxTemp_2);
        // printf("a:%d\n", RxTemp_2);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxTemp_2, 1);
    }
}



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
