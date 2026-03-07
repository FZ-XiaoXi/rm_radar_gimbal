#ifndef __PID_H_
#define __PID_H_

#include "main.h"
#include "stm32f4xx.h"

typedef struct _pid_struct_t
{
  float kp;//比例
  float ki;//积分
  float kd;//微分
  float i_max;//积分限幅
  float out_max;//输出限幅

  float ref;      // target value目标角度
  float fdb;      // feedback value设定角度
  float err[2];   // error and last error差值

  float p_out;//比例输出
  float i_out;//积分输出
  float d_out;//微分输出
  float output;//pid总输出
}pid_struct_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

void gimbal_PID_init(void);
//float pid_calcr(pid_struct_t *pid, float ref, float fdb);

//float pid_calcs(pid_struct_t *pid, float ref, float fdb);
extern pid_struct_t gimbal_yaw_angle_pid;
extern pid_struct_t gimbal_yaw_speed_pid;
extern pid_struct_t gimbal_yaw_speed_pid_return;
extern pid_struct_t gimbal_yaw_angle_pid_return;
							
extern pid_struct_t gimbal_pitch_speed_pid;
extern pid_struct_t gimbal_pitch_angle_pid;
extern pid_struct_t gimbal_pitch_speed_pid_return;
extern pid_struct_t gimbal_pitch_angle_pid_return;
							
extern float pid_calc_raw(pid_struct_t *pid, float tar, float real);
extern float pid_calc_raw_return(pid_struct_t *pid, float tar, float real);
extern float pid_calc_speed(pid_struct_t *pid, float tar, float real);
extern void reset_pid_integrals(pid_struct_t *gimbal);
#endif
