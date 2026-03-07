#include "motor.h"
#include "pid.h"
Motor_send motor_ready[MOTOR_NUM];
Motor_list motor_data[MOTOR_NUM];

/*
 * @brief  	设置电机目标值
 * @param	电机结构体指针
 * @param	目标值
 * @param   ABS->absolute target;
 *          INCR->add from perious target
 *          FEED->change feedforward
 * @retval 	无
 */
void MotorSetTar(Motor_send *motor,float val, ValSet_Type_e type)
{
    if ( type == ABS )
		motor -> target = val;
	else if ( type == INCR )
		motor -> target += val;
//         else
// 		sErrorHandel (OUT_OF_ENUM);
}
//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi)
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
	// X/8192 *2*PI
	//             pi     -pi        8192   0       -PI
	return (x)*(out_max-out_min)/(in_max-in_min);
}


//云台控制的计算函数
void Motor_Calc(gimbal_control_t *feedback_update)
{
	static float tar=0,real=0;

	//yaw轴计算
	tar=msp(motor_ready[MOTOR_YAW].target,-180,+180,-pi,pi);//限幅待修改
	real=msp(feedback_update->gimbal_yaw_motor.absolute_angle,-180,+180,-pi,pi);
	motor_ready[MOTOR_YAW].output_Position=pid_calc_raw(&gimbal_yaw_angle_pid,tar,real);
    motor_ready[MOTOR_YAW].output=pid_calc_speed(&gimbal_yaw_speed_pid,motor_ready[MOTOR_YAW].output_Position,motor_data[MOTOR_YAW].speed);

	//pitch轴计算
	tar=msp(motor_ready[MOTOR_PITCH].target,-90,+90,-pi,pi);//待修改
	real=msp(feedback_update->gimbal_pitch_motor.absolute_angle,-90,+90,-pi,pi);
	motor_ready[MOTOR_PITCH].output_Position=pid_calc_raw(&gimbal_pitch_angle_pid,tar,real);
    motor_ready[MOTOR_PITCH].output=pid_calc_speed(&gimbal_pitch_speed_pid,motor_ready[MOTOR_PITCH].output_Position,motor_data[MOTOR_PITCH].speed);

}

//归中的计算函数
void Motor_return(gimbal_control_t *feedback_update){
	if (feedback_update == NULL ) {
		return;
	}
	static float tar=0,real=0;
	//yaw轴计算
	tar=msp(motor_ready[MOTOR_YAW].target,0,8191,-pi,pi);
	real=msp(feedback_update->gimbal_yaw_motor.motor_gyro,0,8191,-pi,pi);
	motor_ready[MOTOR_YAW].output_Position=pid_calc_raw_return(&gimbal_yaw_angle_pid_return,tar,real);
  motor_ready[MOTOR_YAW].output=pid_calc_speed(&gimbal_yaw_speed_pid_return,motor_ready[MOTOR_YAW].output_Position,motor_data[MOTOR_YAW].speed);

	//pitch轴计算
	tar=msp(motor_ready[MOTOR_PITCH].target,0,8191,-pi,pi);
	real=msp(feedback_update->gimbal_pitch_motor.motor_gyro,0,8191,-pi,pi);
	motor_ready[MOTOR_PITCH].output_Position=pid_calc_raw_return(&gimbal_pitch_angle_pid_return,tar,real);
  motor_ready[MOTOR_PITCH].output=pid_calc_speed(&gimbal_pitch_speed_pid_return,motor_ready[MOTOR_PITCH].output_Position,motor_data[MOTOR_PITCH].speed);
}
