#include "Gimbal.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include <math.h>
#include "CAN_receive.h"
#include "QuaternionEKF.h"
#include "VPC.h"


uint8_t   GIMBAL_OFFSET_FLAG=1; //云台标志位
gimbal_control_t  gimbal_control;


float add_yaw;
float add_pitch;

SemaphoreHandle_t g_xSemTicks;
/* g_xSemVPC is defined in freertos.c (extern in VPC.h) */
  

typedef struct{
    float yaw;
    float pitch;
}tempdata_t;

static tempdata_t temp_data;


void Gimbal_task(void const * argument){
	gimbal_control.vision_ctrl_mode = 0;
	HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);	//TODO TEST
	g_xSemTicks=xSemaphoreCreateBinary( );
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    while (1)
    {

      gimbal_control.Ctl_mode=0;  //云台远程操控模式   0 为视觉自动模式  1为遥控器模式

      if (gimbal_control.Ctl_mode==1)//远程操控模式
		  {
            gimbal_detact_calibration(&gimbal_control);
            gimbal_feedback_update(&gimbal_control,&add_yaw,&add_pitch,gimbal_control.Ctl_mode);
            if(gimbal_control.gimbal_rc_ctrl->rc.s[0]==1)
							HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
						else
							HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
            gimbal_set_tar(&gimbal_control,&add_yaw,&add_pitch);
            //以absolute_angle_set为目标值，absolute_angle为当前值，进行pid串级环的运算，并将值存到motor_ready[]结构体中
            Motor_Calc(&gimbal_control);
       }
       else if (gimbal_control.Ctl_mode==0)//视觉自动模式//todo当锁定无人机发射激光
       {
				  if(gimbal_control.vision_ctrl_mode ==0){
						temp_data.yaw=msp(aim_packet_from_nuc.yaw,-pi,pi,-180,180);
						temp_data.pitch=msp(aim_packet_from_nuc.pitch,-pi,pi,-180,180);
						aim_packet_from_nuc.yaw=0;
						aim_packet_from_nuc.pitch=0;
	//          if(gimbal_control.gimbal_rc_ctrl->rc.s[0]==1)
	//						HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);
	//					else
	//						HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET);	//TODO TEST
						gimbal_detact_calibration(&gimbal_control);
						gimbal_feedback_update(&gimbal_control,&temp_data.yaw,&temp_data.pitch,gimbal_control.Ctl_mode);
						gimbal_set_tar(&gimbal_control,&temp_data.yaw,&temp_data.pitch);
					}else{
						temp_data.yaw=aim_packet_from_nuc.yaw;
						temp_data.pitch=aim_packet_from_nuc.pitch;
						//aim_packet_from_nuc.yaw=0;
						//aim_packet_from_nuc.pitch=0;
						gimbal_detact_calibration(&gimbal_control);
						gimbal_feedback_update(&gimbal_control,&temp_data.yaw,&temp_data.pitch,gimbal_control.Ctl_mode);
						
						gimbal_set_tar(&gimbal_control,&temp_data.yaw,&temp_data.pitch);
					}
          Motor_Calc(&gimbal_control);
          vTaskDelay(pdMS_TO_TICKS(1));
       }
       
    }
}
/**
  * @brief          计算ecd与offset_ecd(中值)之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd = ECD_RANGE-relative_ecd;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    //return relative_ecd * MOTOR_ECD_TO_RAD;
    return relative_ecd;
}

//云台更新数据
static void gimbal_feedback_update(gimbal_control_t *feedback_update,float *add_yaw,float *add_pitch,uint8_t Crtl_mode){

    /* g_xSemVPC must not be created here; it's created centrally in MX_FREERTOS_Init */
    //更新电机实时角度
    feedback_update->gimbal_pitch_motor.motor_gyro=motor_data[1].angle;
    feedback_update->gimbal_yaw_motor.motor_gyro=motor_data[0].angle;

    //更新姿态角实时角度

	  feedback_update->gimbal_pitch_motor.absolute_angle=INS.Roll;//这里算法解算出来的Roll对应实际小云台的俯仰角Pitch
    feedback_update->gimbal_yaw_motor.absolute_angle=INS.Yaw;

    if(Crtl_mode==1)//更新遥控器实时角度
	  {

      feedback_update->gimbal_rc_ctrl=get_remote_control_point();
      //获取遥控器输入值并映射为目标角度值
      *add_yaw=-msp(feedback_update->gimbal_rc_ctrl->rc.ch[2],-660,660,-180,180);//待修改
      *add_pitch=-msp(feedback_update->gimbal_rc_ctrl->rc.ch[3],-660,660,-90,90);		
      
      feedback_update->gimbal_pitch_motor.absolute_angle_set=*add_pitch;
      feedback_update->gimbal_yaw_motor.absolute_angle_set=*add_yaw;

    }
    else if(Crtl_mode==0)//更新视觉控制实时角度
    {
      //更新电机目标机械角度
			if(feedback_update->vision_ctrl_mode == 0){
				//feedback_update->gimbal_pitch_motor.absolute_angle_set= temp_data.pitch+  	feedback_update->gimbal_pitch_motor.absolute_angle;
				//feedback_update->gimbal_yaw_motor.absolute_angle_set=   temp_data.yaw+				feedback_update->gimbal_yaw_motor.absolute_angle;
				feedback_update->gimbal_pitch_motor.absolute_angle_set= temp_data.pitch+  	feedback_update->gimbal_pitch_motor.absolute_angle;
				feedback_update->gimbal_yaw_motor.absolute_angle_set=   temp_data.yaw+				feedback_update->gimbal_yaw_motor.absolute_angle;
			}else{
				feedback_update->gimbal_pitch_motor.motor_speed_set = temp_data.pitch;
				feedback_update->gimbal_yaw_motor.motor_speed_set = temp_data.yaw;
			}

      xSemaphoreGive(g_xSemVPC);
    } 
		vTaskDelay(5);

}


//云台校准中值并执行归中
void gimbal_detact_calibration(gimbal_control_t *gimbal_motort){
    
    //添加标志位判断有无执行过归中，如果有，则不再归中
    while(GIMBAL_GET_FLAG(GIMBAL_OFFSET_FLAG)){
        static uint16_t int_time=0;
        static uint16_t int_stop_time=0;
        gimbal_motort->gimbal_pitch_motor.motor_gyro=motor_data[1].angle;
				gimbal_motort->gimbal_yaw_motor.motor_gyro=motor_data[0].angle;
        MotorSetTar(&motor_ready[0],YAW_OFFSET_ECD, ABS);  
        MotorSetTar(&motor_ready[1], PITCH_OFFSET_ECD, ABS);
				Motor_return(gimbal_motort);
       int_time++;
       if((fabs(gimbal_motort->gimbal_yaw_motor.motor_gyro-YAW_OFFSET_ECD)<GIMBAL_INIT_ANGLE_ERROR)||
				(fabs(gimbal_motort->gimbal_pitch_motor.motor_gyro-PITCH_OFFSET_ECD))<GIMBAL_INIT_ANGLE_ERROR){
            //if(int_stop_time<GIMBAL_INIT_STOP_TIME){
            int_stop_time++;
            //}
           }
        //此处为源码的逻辑，但觉得int_time无需重复累加两次，故删去
        //    else{
        //     if(int_time<GIMBAL_INIT_TIME){
        //         int_time++;
        //     }
        //    }
        //超过初始化额定时间，或者已经稳定到中值一段时间后，退出归中并释放信号量开启执行INS任务
        if((int_time > GIMBAL_INIT_TIME && int_stop_time > GIMBAL_INIT_STOP_TIME)){
					//在归中结束后将当前位置设为目标位置，抑制电机归中后漂移
					//reset_pid_integrals(&gimbal_pitch_angle_pid_return);
					//reset_pid_integrals(&gimbal_pitch_speed_pid_return);
					//reset_pid_integrals(&gimbal_pitch_speed_pid);
          MotorSetTar(&motor_ready[MOTOR_YAW],motor_data[MOTOR_YAW].angle, ABS);
          MotorSetTar(&motor_ready[MOTOR_PITCH],motor_data[MOTOR_PITCH].angle, ABS);			
          //信号量释放
          xSemaphoreGive(g_xSemTicks);
          int_stop_time = 0;
          int_time = 0;
          GIMBAL_FLAG_RESET(GIMBAL_OFFSET_FLAG);
        }
    }
}


//云台限幅并计算出目标值
void gimbal_set_tar(gimbal_control_t *gimbal_motort,float *add_yaw,float *add_pitch){
	 if (gimbal_motort == NULL || add_pitch == NULL||add_yaw==NULL) {
        return;
    }
	 //pitch

    //根据当前输入值的正负来判断处于左值还是右值，从而决定目标值,并且加上一个死区，防止频繁切换
    if(*add_pitch<0)//-40
			{
        if(gimbal_motort->gimbal_pitch_motor.absolute_angle<=(PITCH_Limit_Hight+4))//为边界粗略写的死区，下同
					{
            MotorSetTar(&motor_ready[MOTOR_PITCH],PITCH_Limit_Hight,ABS);
          }
				else
					{
            MotorSetTar(&motor_ready[MOTOR_PITCH],gimbal_motort->gimbal_pitch_motor.absolute_angle_set,ABS);
          }
      }
		else if(*add_pitch>0)//25
			{
        if(gimbal_motort->gimbal_pitch_motor.absolute_angle>=(PITCH_Limit_Low-1.5))
					{
            MotorSetTar(&motor_ready[MOTOR_PITCH],PITCH_Limit_Low,ABS);
          }
					else
					{
				    MotorSetTar(&motor_ready[MOTOR_PITCH],gimbal_motort->gimbal_pitch_motor.absolute_angle_set,ABS);
          }
      }   
		else
			{
 				MotorSetTar(&motor_ready[MOTOR_PITCH],gimbal_motort->gimbal_pitch_motor.absolute_angle_set,ABS);
      }


    //yaw
    if(*add_yaw<0)
			{
        if(*add_yaw<=YAW_Limit_Low)//-90
					{
           if(gimbal_motort->gimbal_yaw_motor.absolute_angle<=(YAW_Limit_Low+3))
						 {
               MotorSetTar(&motor_ready[MOTOR_YAW],YAW_Limit_Low,ABS);
             }
          }
					else
					{
               MotorSetTar(&motor_ready[MOTOR_YAW],gimbal_motort->gimbal_yaw_motor.absolute_angle_set,ABS);
          }
      }
		 else if(*add_yaw>0)
			{
        if(*add_yaw>=YAW_Limit_Hight)//90
					{
              if(gimbal_motort->gimbal_yaw_motor.absolute_angle>=(YAW_Limit_Hight-3))
								{
                  MotorSetTar(&motor_ready[MOTOR_YAW],YAW_Limit_Hight,ABS);
                }
							else
								{
									MotorSetTar(&motor_ready[MOTOR_YAW],gimbal_motort->gimbal_yaw_motor.absolute_angle_set,ABS);
								}
				  }
			  else
					{
				    MotorSetTar(&motor_ready[MOTOR_YAW],gimbal_motort->gimbal_yaw_motor.absolute_angle_set,ABS);
          }
       }
		else
			{
				MotorSetTar(&motor_ready[MOTOR_YAW],gimbal_motort->gimbal_yaw_motor.absolute_angle_set,ABS);
			}


}

