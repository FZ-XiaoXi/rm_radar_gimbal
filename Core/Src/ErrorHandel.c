#include "main.h"
#include "ErrorHandel.h"

uint8_t error_flag=0;


void sErrorHandel(uint8_t flag)
{
    while(1)
    {
        error_flag = flag;
    }
}