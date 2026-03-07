/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include "imu.h"
#include "vofa.h"
#include "INS.h"
#include "Gimbal.h"
motor_measure_t motor_chassis[7];
extern gimbal_control_t gimbal_control;


extern CAN_HandleTypeDef hcan1;

CAN_RxHeaderTypeDef rx_header;

//motor data read
void get_motor_measure(Motor_list *ptr, uint8_t *data)
	    {
	        ptr->angle = (uint16_t)((data[0] << 8) | data[1]);
	        ptr->speed = (int16_t)((data[2] << 8) | data[3]);
	        ptr->given_current = (int16_t)((data[4] << 8) | data[5]);
	        ptr->temperate = data[6];
	    }


static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
uint8_t rx_data[8];



/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
		    {
		        case CAN_YAW_MOTOR_ID:
		        	get_motor_measure(&motor_data[0], rx_data);
		        	break;
		        case CAN_PIT_MOTOR_ID:
		        	get_motor_measure(&motor_data[1], rx_data);
		        	break;
//		        case CAN_3508_M1_ID:
//		        	get_motor_measure(&motor_data[1], rx_data);
//		        	break;
//		        case CAN_3508_M4_ID:
//		        	get_motor_measure(&motor_data[1], rx_data);
//		        	break;
		        default:
		                {
		                    break;
		                }
		    }
	}



//        case CAN_YAW_MOTOR_ID:
//        case CAN_PIT_MOTOR_ID:
//        case CAN_TRIGGER_MOTOR_ID:
//        {
//        	static uint8_t i = 0;
//        	//get motor id
//        	i = rx_header.StdId - CAN_3508_M1_ID;
//        	get_motor_measure(&motor_chassis[i], rx_data);
//            break;
//        }


//    for(int i = 0; i<4;i++){
//    	if(i == rx_header.StdId - CAN_3508_M1_ID){
//    		get_motor_measure(&motor_chassis[i], rx_data);
//    	}
//    }
}


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t shoot, int16_t pitch, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1FF;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (shoot >> 8);
    gimbal_can_send_data[3] = shoot;
    gimbal_can_send_data[4] = (pitch >> 8);
    gimbal_can_send_data[5] = pitch;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */

void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void Can_Send(void){
  while(1){
  //CAN_cmd_chassis(motor_ready[0]->output,0,0,0);

	CAN_cmd_gimbal(motor_ready[0].output,0,motor_ready[1].output,0);

  vofa_demo3(motor_data[0].angle,motor_ready[0].target,motor_ready[1].target,&huart6);

  }
}
