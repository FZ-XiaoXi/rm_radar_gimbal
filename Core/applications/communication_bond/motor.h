#ifndef MOTOR_H
#define MOTOR_H
// #include "main.h"
#include "config.h"
#include "struct_typedef.h"
#include "Gimbal.h"

#define pi 3.1415926f

//接收电机结构体
typedef struct 
{
    uint16_t angle;
    int16_t speed;
    int16_t given_current;
    uint16_t temperate;/* data */
    int16_t last_ecd;
}__attribute__((__packed__)) Motor_list;

//发送电机结构体
typedef struct 
{
    float target;/* data */
    float output;
    float output_Position;
}Motor_send;

extern Motor_send motor_ready[MOTOR_NUM];
extern Motor_list motor_data[MOTOR_NUM];


void Motor_Calc(gimbal_control_t *feedback_update);
void MotorSetTar(Motor_send *motor,float val, ValSet_Type_e type);
void Motor_return(gimbal_control_t *feedback_update);

double msp(double x, double in_min, double in_max, double out_min, double out_max);
#endif /* __MOTOR_H */
