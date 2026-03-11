#include "VPC.h"
#include "motor.h"
#include "gimbal.h"
#include "pid.h"

 void VPC_Receive(void)
 {
  /* Wait until Serial notifies that a validated packet is ready. */
  if (g_xSemVPC == NULL) {
    /* semaphore not ready yet; avoid calling null handle */
    vTaskDelay(pdMS_TO_TICKS(10));
		 
    return;
  }
	//HAL_GPIO_TogglePin(laser_GPIO_Port,laser_Pin);
	
  xSemaphoreTake(g_xSemVPC, portMAX_DELAY);

  /* Serial already copied validated frame into aim_packet_from_nuc in UnPack_Data_ROS2 */
  /* Copy relevant fields into outgoing packet so we reply with updated data */
  aim_packet_to_nuc.yaw = INS.Yaw;
  aim_packet_to_nuc.pitch = INS.Roll;
 }
 

void VPC_Init(void)
{
  aim_packet_to_nuc.detect_color=0;//1-blue 0-red
  Send_Packet_Init(&aim_packet_to_nuc);

}



 void VPC_Task(void *argument)
 {
  VPC_Init();
  /* one-shot debug notification to host to confirm task start */

  for(;;)
   {
    /* Block until a valid packet arrives, then send an updated response */
    VPC_Receive();
		extern gimbal_control_t gimbal_control;
		aim_packet_to_nuc.last_pc_timestamp_us = aim_packet_from_nuc.pc_timestamp_us;
		aim_packet_to_nuc.timestamp = HAL_GetTick();
		aim_packet_to_nuc.aim_x = 0;
		aim_packet_to_nuc.aim_y = 0;
		aim_packet_to_nuc.aim_z = 0;
    Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);
    /* small yield to allow lower priority work; avoid busy-looping */
    vTaskDelay(pdMS_TO_TICKS(1));
   }

 }


  