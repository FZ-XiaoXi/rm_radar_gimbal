#include "pid.h"

float PI=3.1415926;
pid_struct_t gimbal_yaw_speed_pid;
pid_struct_t gimbal_yaw_angle_pid;
pid_struct_t gimbal_yaw_speed_pid_return;
pid_struct_t gimbal_yaw_angle_pid_return;

pid_struct_t gimbal_pitch_speed_pid;
pid_struct_t gimbal_pitch_angle_pid;
pid_struct_t gimbal_pitch_speed_pid_return;
pid_struct_t gimbal_pitch_angle_pid_return;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID初始化函数
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

float rad_format(float tar, float real){
	if(tar-real>PI){
			real+=2*PI;

	}else if(tar-real<-PI){
			real=real-2*PI;
	}
	return tar-real;
	}

  float rad_format_limit(float tar, float real){
    if(tar<PI&&real<PI){
        return tar-real;
    }else if(tar>PI&&real>PI){
        return tar-real;
    }else if(tar>PI&&real<PI){
        return tar-(real+2*PI);
    }else{
        return (tar+2*PI)-real;
    }
  }
	
void LIMIT_MIN_MAX(float *be,float nmax,float mmax){
	if(*be>=mmax){
		*be=mmax;
	}else if(*be<=nmax){
		*be=nmax;
	}
}
  //重置PID积分项和误差项，使得归中后更过度更平滑（未完善，可优化）
void reset_pid_integrals(pid_struct_t *pid)
{
	pid->i_out =0;
	pid->err[0]=0;
	pid->err[1]=0;
	pid->d_out=0;
}
                                         //目标      //实际
float pid_calc_speed(pid_struct_t *pid, float tar, float real)//PID运算函数
{
  pid->ref = tar;
  pid->fdb = real;

  pid->err[0] = pid->ref - pid->fdb;


  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[1] = pid->err[0];


  LIMIT_MIN_MAX(&pid->i_out, -pid->i_max, pid->i_max);
	pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(&pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
                                         //目标      //实际
float pid_calc_raw(pid_struct_t *pid, float tar, float real)//PID运算函数
{
		pid->err[0] = tar-real;
	pid->p_out = pid->kp * pid->err[0];
	float i_separation_threshold = 0.1f;
	
	if(fabs(pid->err[0]) < i_separation_threshold){
		pid->i_out += pid->ki * pid->err[0];
		
	}else{
		pid->i_out=0.0f;
	}
	LIMIT_MIN_MAX(&pid->i_out, -pid->i_max, pid->i_max);
	pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
	pid->err[1] = pid->err[0];
	
	pid->output = pid->p_out + pid->i_out + pid->d_out;
	LIMIT_MIN_MAX(&pid->output, -pid->out_max, pid->out_max);
	return pid->output;
}

                                         //目标      //实际
float pid_calc_raw_return(pid_struct_t *pid, float tar, float real)//PID运算函数
{
  pid->ref = tar;
  pid->fdb = real;

  pid->err[0] = rad_format(pid->ref, pid->fdb);


  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  pid->err[1] = pid->err[0];

  LIMIT_MIN_MAX(&pid->i_out, -pid->i_max, pid->i_max);
	//积分限幅，防止出现积分饱和现象
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(&pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}


void gimbal_PID_init()//角度环和速度环的PID初始化,只是初测出来的数据，具体还需要测试
{
	//YAW轴初始化
	pid_init(&gimbal_yaw_speed_pid,      300,    1,   0.1, 1000, 2000);//P=30,I=0,D=0
	pid_init(&gimbal_yaw_angle_pid,      300,    1,   0.1, 1000, 20);//P=500,I=0,D=1
  pid_init(&gimbal_yaw_speed_pid_return, 200,    0.06,   0.003, 1000, 3000);//P=30,I=0,D=0`
  pid_init(&gimbal_yaw_angle_pid_return, 50 ,   0,   0.005, 100, 1000);//P=500,I=0,D=1
	//PITCH轴初始化
	//pid_init(&gimbal_pitch_speed_pid,150,0,0, 1000, 1000);//P=30,I=0,D=0
	//pid_init(&gimbal_pitch_angle_pid,0,0,0,100, 1000);//P=500,I=0,D=1
	pid_init(&gimbal_pitch_speed_pid,      200,     5, 0.1, 1000, 2000);//P=30,I=0,D=0
	pid_init(&gimbal_pitch_angle_pid,      300,     3, 0.1, 1000, 20);//P=500,I=0,D=1//1800,0.1,1
  pid_init(&gimbal_pitch_speed_pid_return, 16,     0.005, 0, 1000, 3000);//P=30,I=0,D=0`
  pid_init(&gimbal_pitch_angle_pid_return, 300,     0, 0.01, 100, 1000);//P=500,I=0,D=1
}
