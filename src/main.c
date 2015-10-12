#include "stm32f10x.h"
#include "Conf.h"
#include "Timers.h"
#include "main.h"
#include <string.h>
#include "led.h"
sTime gTime;
extern uint8_t gDoMeasurement;
uint8_t gTabToSend[10];
SFrame frame = {0xAAAA,0xBB,0xCC,'\n'};
#define BLACK {0x0F, 0x0F, 0xA0}
#define RED {0x10, 0x00, 0x00}
#define GREEN {0x00, 0x13, 0x0F}
#define BLUE {0x00, 0x00, 0x10}
#define WHITE {0xff, 0xff, 0xff}

static uint8_t black[][3] = {GREEN, GREEN, GREEN, GREEN,
		GREEN, GREEN, GREEN, GREEN,
		GREEN, GREEN, GREEN, GREEN,
		GREEN, GREEN, GREEN, GREEN,
                            };

int main()
{

    RCC_Configuration();
    GPIO_Configuration();
    RTC_Conf();
    ConfirureTimerForDelay();
    UsartConf();
    NVIC_Configuration();
    SystemCoreClockUpdate();
    ws2812Init();

    ws2812Send(black, 16);

    ws2812Send(black, 2);
    while(1)
    {

		GPIOC->BSRR |= 1<<0;
		delay10();
		GPIOC->BRR |= 1<<0;

        DelayUs(10000);
    }
}
