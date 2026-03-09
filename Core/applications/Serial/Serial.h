	/*
 * @file		Serial.c/h
 * @brief		usb-cdc通信程序，用于上位机和stm32的通信，需配合上位机代码使用，现采用zy版本
 * @history
 * 版本			作者			编写日期
 * v1.0.0		许金帅		2023/4/1
 * v2.0.0		冯俊玮		2024/10/29
 *
 */
#ifndef __Serial_h__
#define __Serial_h__
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "usart.h"
#define RX_BUFFER_SIZE 512
//下面是为和ROS2上位机沟通而定制的结构体(zy)
 typedef struct
 {
   uint8_t header ;//0x5A
   uint8_t detect_color : 1;  // 0-red 1-blue
   uint8_t task_mode : 2;     // 0-auto 1-aim 2-buff
   uint8_t reset_tracker : 1;
   uint8_t is_play : 1;
   uint8_t change_target : 1;
   uint8_t reserved : 2;
   float roll;
   float pitch;
   float yaw;
   float aim_x;
   float aim_y;
   float aim_z;
   uint16_t game_time;  // (s) game time [0, 450]
   uint32_t timestamp;  // (ms) board time
   uint16_t checksum ;
 } __attribute__((packed)) send_packet_t;


typedef struct
{
  uint8_t header;//0xA5
  uint8_t state : 2;       // 0-untracking 1-tracking-aim
  uint8_t id : 3;          // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal

  //o-auto
  float yaw;
  float pitch;

  float yaw_diff;
  float pitch_diff;
  int fire_advice;

  uint16_t checksum ;
} __attribute__((packed)) receive_packet_t;

extern uint8_t rx_buffer[RX_BUFFER_SIZE];

extern receive_packet_t aim_packet_from_nuc;
extern send_packet_t aim_packet_to_nuc;
extern uint8_t buf_receive_from_nuc[sizeof(receive_packet_t)];//具体应用在usb接收函数中（在cdc_if.c中）


int CDC_SendFeed(uint8_t* Fed, uint16_t Len);//CDC发送反馈数据的函数
//ROS2发送代码
void Pack_And_Send_Data_ROS2(send_packet_t *sendinfo);
void UnPack_Data_ROS2(uint8_t *receive_Array, receive_packet_t *receive_info, uint16_t Len);
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage,uint32_t dwLength, uint16_t wCRC);
void Send_Packet_Init(send_packet_t *send_packet);
float Serial_GetRoll(send_packet_t *send_packet);
float Serial_GetYaw(send_packet_t *send_packet);
float Serial_GetPitch(send_packet_t *send_packet);


int CDC_Receive_ROS2(uint8_t *receive_buf, receive_packet_t *receive_packet, uint32_t Len);

extern receive_packet_t   aim_packet_from_nuc;
extern send_packet_t      aim_packet_to_nuc;
extern uint8_t buf_receive_from_nuc[sizeof(receive_packet_t)];

typedef struct
{
    uint8_t header;                 // 固定为 0xAC
    int mode;
    float yaw_angle_p;
    float yaw_angle_i;
    float yaw_angle_d;
    float pitch_angle_p;
    float pitch_angle_i;
    float pitch_angle_d;
    float yaw_speed_p;
    float yaw_speed_i;
    float yaw_speed_d;
    float pitch_speed_p;
    float pitch_speed_i;
    float pitch_speed_d;
} __attribute__((packed)) to_stm_pid_packet_t;

#endif