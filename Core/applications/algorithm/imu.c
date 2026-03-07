#include "gpio.h"
#include <math.h>
#include "BMI088driver.h"
#include "imu.h"
#include "FreeRTOS.h"
#include "semphr.h"     // 信号量相关类型和函数声明（包含 SemaphoreHandle_t 定义）
#include "Gimbal.h"
#include "MahonyAHRS.h"
#include "bsp_dwt.h"
#include "QuaternionEKF.h"
#include "kalman_filter.h"

#define kp 				25.00f
#define ki 				0.005f
#define cycle_T 		0.005f//200hz 0.005
#define half_T 			0.0025f

int16_t Ax,Ay,Az,Gx,Gy,Gz;
float gyro[3], accel[3], temp;
float insq[4];
//param_imu imu_data;
param_Angle imu_Angle;
param_Angle imu_offset;
float q[4] = {1.0,0.0,0.0,0.0};//四元数数组
float exInt = 0.0 ,eyInt = 0.0 ,ezInt = 0.0 ;
static float dt = 0, t = 0;
// axis_3f_t gyro_ins, accel_ins;
uint8_t    	 SENSER_OFFSET_FLAG=1;  //传感器校准标志位

//////////////////////////////////////////////////////////////////////////////////
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);  //0x5f3759df是一个平方根倒数速算法
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//////////////////////////////////////////////////////////////////////////////////

// void IMU_GetValues(void)
// {
// 	// BMI088_read(gyro, accel, &temp);
// 	#ifdef Mahony_AHRS_DEBUG
// 	gyro[1]-=-4895900 / 208192 * BMI088_GYRO_500_SEN;
// 	gyro[2]-=1707958 / 208192 * BMI088_GYRO_500_SEN;
// 	accel[1]-=-126158552 / 208192 * BMI088_ACCEL_3G_SEN;
// 	#endif 
// 	 float init_quaternion[4] = { 0 };
// //   EKF_Quaternion_Init(init_quaternion);
//   IMU_QuaternionEKF_Init(
// 			 10,
// 			 0.001f,//0.001f,
// 			 1000000,//1000000,
// 			 0.9996f,
// 			 0.000399f);
// 	}

void IMU_AHRSupdate(param_imu* imu_temp)
{
	float ax,ay,az;
	float gx,gy,gz;
	ax = imu_temp->AX;
	ay = imu_temp->AY;
	az = imu_temp->AZ;
	gx = imu_temp->GX;
	gy = imu_temp->GY;
	gz = imu_temp->GZ;

	float vx, vy, vz;
	float ex, ey, ez;

	float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

	float norm = fast_sqrt(ax*ax+ay*ay+az*az);
    ax = ax * norm;//将加速度归一化，与四元数对应
    ay = ay * norm;
    az = az * norm;

	vx = 2 * (q1*q3 - q0*q2);//提取姿态矩阵中的重力分量
	vy = 2 * (q2*q3 + q0*q1);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    ex = (ay * vz - az * vy);//  |A|*|B|*sin<A,B>，叉乘得出姿态误差向量
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

	exInt += ki * ex;//对误差进行积分
	eyInt += ki * ey;
	ezInt += ki * ez;

	gx += kp * ex + exInt;//修正角速度值
	gy += kp * ey + eyInt;
	gz += kp * ez + ezInt;

	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;//更新四元数
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy)  * half_T;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx)  * half_T;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx)  * half_T;

	norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

	q[0] = q0 * norm;
	q[1] = q1 * norm;
	q[2] = q2 * norm;
	q[3] = q3 * norm;

}



// uint8_t IMU_Offset(param_imu *value,offset *set){
// 	static float tempgx=0,tempgy=0,tempgz=0;
// 	static float tempax=0,tempay=0,tempaz=0;
// 	static uint16_t cnt_a=0;
// 	if(cnt_a==0){
// 		tempgx=0;
// 		tempgy=0;
// 		tempgz=0;
// 		tempax=0;
// 		tempay=0;
// 		tempaz=0;
// 		cnt_a=1;

// 	}
// 	tempax+=value->AX;
// 	tempay+=value->AY;
// 	tempaz+=value->AZ;
// 	tempgx+=value->GX;
// 	tempgy+=value->GY;
// 	tempgz+=value->GZ;
	
// 	if(cnt_a==100){
// 		set->aX=tempax/cnt_a;
// 		set->aY=tempay/cnt_a;
// 		set->aZ=tempaz/cnt_a;
// 		set->gX=tempgx/cnt_a;
// 		set->gY=tempgy/cnt_a;
// 		set->gZ=tempgz/cnt_a;
// 		cnt_a = 0;
// 		return 1;
// 	}
// 	cnt_a++;
//   return 0;
// }

uint8_t IMU_Offset(param_Angle *value,param_Angle *set){
	static float temppitch=0,temproll=0,tempyaw=0;
	static uint16_t cnt_a=0;
	if(cnt_a==0){
		temppitch=0;
		temproll=0;
		tempyaw=0;
		cnt_a=1;

	}
	temppitch+=value->Pitch;
	temproll+=value->Roll;
	tempyaw+=value->Yaw;
	
	if(cnt_a==100){
		set->Pitch=temppitch/cnt_a;
		set->Roll=temproll/cnt_a;
		set->Yaw=tempyaw/cnt_a;
		cnt_a = 0;
		return 1;
	}
	cnt_a++;
  return 0;
}

// void IMU_Calibration(void){
// 	gyro[0]-=imu_offset.gX;
// 	gyro[1]-=imu_offset.gY;
// 	gyro[2]-=imu_offset.gZ;

// 	accel[0]-=imu_offset.aX;
// 	accel[1]-=imu_offset.aY;
// 	accel[2]-=imu_offset.aZ;
// 	if(GET_FLAG(calibration_OFFSET)){
// 		if(IMU_Offset(&imu_data,&imu_offset)){
// 			SENSER_FLAG_RESET(calibration_OFFSET);
// 		}
// 	}
// }

void IMU_Calibration(void){
	imu_Angle.Pitch-=imu_offset.Pitch;
	imu_Angle.Roll-=imu_offset.Roll;
	//imu_Angle.Yaw-=imu_offset.Yaw;
	imu_Angle.Yaw-=1.527;
	if(GET_FLAG(calibration_OFFSET)){
		if(IMU_Offset(&imu_Angle,&imu_offset)){
			SENSER_FLAG_RESET(calibration_OFFSET);
		}
	}
}

// void INS(void)
// {
// 	static uint32_t INS_dwt_count = 0;
// 	float ins_dt = 0.0f;
// 	//云台归中完成获取信号量
// 	xSemaphoreTake(g_xSemTicks, portMAX_DELAY);
// 	while(1){
// 	 ins_dt = DWT_GetDeltaT(&INS_dwt_count);
// 	//dt = DWT_GetDeltaT(&INS_dwt_count);
//     //t += dt;
// 	IMU_GetValues();
// 	accel_ins.x = accel[0];
// 	accel_ins.y = accel[1];
// 	accel_ins.z = accel[2];
// 	gyro_ins.x = gyro[0];
// 	gyro_ins.y = gyro[1];
// 	gyro_ins.z = gyro[2];
// 	// //IMU_Calibration();
// 	// IMU_AHRSupdate(&imu_data);
// 	// MahonyAHRSupdateIMU(q, gyro, accel);
// 	// IMU_QuaternionEKF_Update(gyro[2], gyro[1], gyro[0], accel[2], accel[1], accel[0], ins_dt);
// 	IMU_QuaternionEKF_Update(gyro_ins.x, gyro_ins.y, gyro_ins.z, accel_ins.x, accel_ins.y, accel_ins.z, ins_dt);
		
//  	// q[0] = q[0] * QEKF_INS.accLPFcoef + QEKF_INS.q[0] * (1 - QEKF_INS.accLPFcoef);
//   	// q[1] = q[0] * QEKF_INS.accLPFcoef + QEKF_INS.q[1] * (1 - QEKF_INS.accLPFcoef);
//     // q[2] = q[0] * QEKF_INS.accLPFcoef + QEKF_INS.q[2] * (1 - QEKF_INS.accLPFcoef);
//     // q[3] = q[0] * QEKF_INS.accLPFcoef + QEKF_INS.q[3] * (1 - QEKF_INS.accLPFcoef);


// 	//转换成360度
// 	// imu_Angle.Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.2957;
// 	// imu_Angle.Roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.2957;
// 	// imu_Angle.Yaw=atan2f(2*q[1]*q[2]+2*q[0]*q[3],2*(q[0]*q[0]+q[1]*q[1])-1)*57.2957;

// 	// imu_Angle.Yaw  += imu_data.GZ* 57.2957* cycle_T* 4;//由于在静止状态下z轴方向本身就有重力加速度，所以不用将加速度计和陀螺仪的结果融合，直接对角速度进行积分（这里只有乘上4后才会变得准一些，具体原因尚不清楚）
	
// 	//弧度制
// 	// imu_Angle.Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]);
// 	// imu_Angle.Roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1);
// 	// imu_Angle.Yaw=atan2f(2*q[1]*q[2]+2*q[0]*q[3],2*(q[0]*q[0]+q[1]*q[1])-1);
// }

// 	}

