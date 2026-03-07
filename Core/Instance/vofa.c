/*
 * @file		vofa.c/h
 * @brief	    vofa+发送数据（调参）
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		冯俊玮		2024/9/10		向上位机vofa+发送数据
 */
#include "vofa.h"

#define MAX_BUFFER_SIZE 128
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;
/**
 ***********************************************************************
 * @brief:      VOFA_Transmit(uint8_t* buf, uint16_t len)
 * @param:		void
 * @retval:     void
 * @details:    修改通信工具，USART或者USB
 ***********************************************************************
 **/
void VOFA_Transmit(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
  HAL_UART_Transmit(huart, buf, len, 0xff);
}
/**
 ***********************************************************************
 * @brief:      VOFA_Send_Data(float data)
 * @param[in]:  num: 数据编号 data: 数据
 * @retval:     void
 * @details:    将浮点数据拆分成单字节
 ***********************************************************************
 **/
void VOFA_Send_Data(uint8_t num, float data)
{
  send_buf[cnt++] = byte0(data);
  send_buf[cnt++] = byte1(data);
  send_buf[cnt++] = byte2(data);
  send_buf[cnt++] = byte3(data);
}
/**
 ***********************************************************************
 * @brief      VOFA_Send_Frametail(void)
 * @param      NULL
 * @retval     void
 * @details:   给数据包发送帧尾
 ***********************************************************************
 **/
void VOFA_Send_Frametail(UART_HandleTypeDef *huart)
{
  send_buf[cnt++] = 0x00;
  send_buf[cnt++] = 0x00;
  send_buf[cnt++] = 0x80;
  send_buf[cnt++] = 0x7f;

  /* 将数据和帧尾打包发送 */
  VOFA_Transmit(huart, (uint8_t*) send_buf, cnt);
  cnt = 0; // 每次发送完帧尾都需要清零
}
/**
 ***********************************************************************
 * @brief      vofa_demo(void)
 * @param      NULL
 * @retval     void
 * @details:   demo示例
 ***********************************************************************
 **/
void vofa_demo(float data1,
	       float data2,
	       float data3,
	       float data4,
	       float data5,
	       float data6,
	       UART_HandleTypeDef *huart)
{

  // Call the function to store the data in the buffer
  VOFA_Send_Data(0, data1);
  VOFA_Send_Data(1, data2);
  VOFA_Send_Data(2, data3);
  VOFA_Send_Data(3, data4);
  VOFA_Send_Data(4, data5);
  VOFA_Send_Data(5, data6);
  // Call the function to send the frame tail
  VOFA_Send_Frametail(huart);
}

void vofa_demo2(float data1, float data2, UART_HandleTypeDef *huart)
{

  // Call the function to store the data in the buffer
  VOFA_Send_Data(0, data1);
  VOFA_Send_Data(1, data2);
  // Call the function to send the frame tail
  VOFA_Send_Frametail(huart);
}

void vofa_demo3(float data1, float data2, float data3 , UART_HandleTypeDef *huart)
{

  // Call the function to store the data in the buffer
  VOFA_Send_Data(0, data1);
  VOFA_Send_Data(1, data2);
  VOFA_Send_Data(2, data3);
  // Call the function to send the frame tail
  VOFA_Send_Frametail(huart);
}