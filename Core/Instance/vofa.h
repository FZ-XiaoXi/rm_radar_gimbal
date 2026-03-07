#ifndef ALGORITHM_H
#define ALGORITHM_H
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"

#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))

void VOFA_Send_Data(uint8_t num, float data);
void VOFA_Send_Frametail(UART_HandleTypeDef *huart);
void vofa_demo(float data1,
	       float data2,
	       float data3,
	       float data4,
	       float data5,
	       float data6,
	       UART_HandleTypeDef *huart);
void vofa_demo2(float data1, float data2, UART_HandleTypeDef *huart);
void vofa_demo3(float data1, float data2,float data3 , UART_HandleTypeDef *huart);
#endif // DEBUG

