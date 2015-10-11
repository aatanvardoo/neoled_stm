#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include <stdio.h>

typedef struct
{
    u16 msgId;
    int16_t temperature;
    u16 humidity;
    char eof;
}SFrame;
#endif
