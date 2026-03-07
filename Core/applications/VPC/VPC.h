#ifndef __VPC_H__
#define __VPC_H__

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stdint.h"
#include "Serial.h"
#include "vofa.h"
#include "gimbal.h"
#include "usbd_cdc_if.h"
#include "INS.h"

void VPC_Init(void);
void VPC_Receive(void);
void VPC_Task(void *argument);

extern SemaphoreHandle_t g_xSemVPC;

#define VPC_TASK_PERIOD 1
#endif