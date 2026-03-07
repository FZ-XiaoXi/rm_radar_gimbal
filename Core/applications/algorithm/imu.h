#ifndef __IMU_H
#define __IMU_H

#include "gpio.h"
#include "math.h"
#include "BMI088driver.h"



//对 SENSER_OFFSET_FLAG 的位的操作
#define SENSER_FLAG_SET(FLAG)   SENSER_OFFSET_FLAG|=FLAG                //标志位置1
#define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG&=~FLAG               //标志位值0
#define GET_FLAG(FLAG)         (SENSER_OFFSET_FLAG&FLAG)==FLAG ? 1 : 0  //获取标志位状态

#define calibration_OFFSET 0x01 //第一位校准标志位
extern uint8_t   SENSER_OFFSET_FLAG; //标志位组

			typedef struct{
				float AX;
				float AY;
				float AZ;
				float GX;
				float GY;
				float GZ;
			}param_imu;



		    typedef struct{
				float Pitch;
				float Roll;
				float Yaw;
			}param_Angle;

			typedef struct{
				float aX;
				float aY;
				float aZ;
				float gX;
				float gY;
				float gZ;
			}offset;
			
			// typedef struct
			// {
			// 	float x;
			// 	float y;
			// 	float z;
			// 	} axis_3f_t;
				
     		extern param_Angle imu_offset;
			extern param_Angle imu_Angle;

			extern uint16_t cnt_a;

			extern float gyro[3], accel[3];
			
void IMU_getEuleranAngles(void);



#endif
