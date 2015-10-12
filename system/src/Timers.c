/*
 * Timers.c
 *
 *  Created on: Sep 9, 2014
 *      Author: Adi
 */
#include "main.h"
#include "Conf.h"
#include <string.h>
#define LED_PER_HALF 1
static volatile uint32_t TimingDelay;
static volatile uint32_t TimeDelayMs;
static volatile uint32_t TimeDelayUs;
uint16_t capture = 0;
__IO uint16_t CCR1_Val = 12000;
__IO uint16_t CCR2_Val = 1200;
__IO uint16_t CCR3_Val = 13654;
__IO uint16_t CCR4_Val = 6826;

extern uint32_t  * const ledDmaPtr;
extern uint8_t sizeofLedBuffer;

void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void SysTick_Handler(void)
{
	  TimingDelay_Decrement();
}

void DelayMs(volatile uint32_t time)
{
	TimeDelayMs = time-1;//-1 bo np od 5 do  to trzeba 6 przerwa� == 6ms
    capture = TIM_GetCapture1(TIM2);
    TIM_SetCompare1(TIM2, capture + CCR1_Val);

    while(TimeDelayMs > 0);
}
/* Z ziarnem 100us*/
void DelayUs(volatile uint32_t time)
{
	TimeDelayUs = time-1;//-1 bo np od 5 do  to trzeba 6 przerwa� == 6ms
    capture = TIM_GetCapture2(TIM2);
    TIM_SetCompare2(TIM2, capture + CCR2_Val);

    while(TimeDelayUs > 0);
}

void ConfigureTImerForEncoder(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);    // odpalamy zegar timera

     // TimeBase configuration
     TIM_TimeBaseStructure.TIM_Prescaler     = 0x0;
     TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
     TIM_TimeBaseStructure.TIM_Period        = 256;
     TIM_TimeBaseStructure.TIM_ClockDivision = 0;
     TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

     TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              //Inicjalizacja licznika


     TIM_ICStructInit(&TIM_ICInitStructure);
     //Initialize input capture structure: Ch2
     TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
     TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
     //TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
     //TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
     TIM_ICInitStructure.TIM_ICFilter    = 0xF;
     TIM_ICInit(TIM4, &TIM_ICInitStructure);

     //Initialize input capture structure: Ch2
     TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
     TIM_ICInit(TIM4, &TIM_ICInitStructure);

      // timer 8 jako enkoder
      TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);




     //TIM_ClearFlag(TIM8, TIM_FLAG_Update | TIM_FLAG_COM | TIM_FLAG_Break | TIM_FLAG_CC1 | TIM_FLAG_CC2 |TIM_FLAG_CC3|TIM_FLAG_CC4);
      //TIM_ITConfig(TIM8, TIM_IT_Update | TIM_IT_COM | TIM_IT_Break | TIM_IT_CC1 | TIM_IT_CC2 |TIM_IT_CC3|TIM_IT_CC4, ENABLE);

     // Clear all pending interrupts
     TIM_ClearFlag(TIM4, TIM_FLAG_Update);
     //TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

     TIM_SetCounter(TIM4, 0);

     TIM_Cmd(TIM4, ENABLE);
}
void ConfirureTimerForDelay(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	__IO uint16_t PrescalerValue = 0;
	/* ---------------------------------------------------------------
	  TIM2 Configuration: Output Compare Timing Mode:
	  TIM2 counter clock at 6 MHz
	  CC1 update rate = TIM2 counter clock / CCR1_Val = 1000Hz
	  CC2 update rate = TIM2 counter clock / CCR2_Val = 219.7 Hz
	  CC3 update rate = TIM2 counter clock / CCR3_Val = 439.4 Hz
	  CC4 update rate = TIM2 counter clock / CCR4_Val = 878.9 Hz
	--------------------------------------------------------------- */
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 12000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

	TIM_OC3Init(TIM2, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* Output Compare Timing Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    /* Pin PC.06 toggling with frequency = 73.24 Hz */
    //GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)));
    TimeDelayMs--;
    capture = TIM_GetCapture1(TIM2);
    TIM_SetCompare1(TIM2, capture + CCR1_Val);
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

    /* Pin PC.07 toggling with frequency = 109.8 Hz */
    //GPIO_WriteBit(GPIOC, GPIO_Pin_7, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7)));
    TimeDelayUs--;
    capture = TIM_GetCapture2(TIM2);
    TIM_SetCompare2(TIM2, capture + CCR2_Val);
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

    /* Pin PC.08 toggling with frequency = 219.7 Hz */
    //GPIO_WriteBit(GPIOC, GPIO_Pin_8, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_8)));
    capture = TIM_GetCapture3(TIM2);
    TIM_SetCompare3(TIM2, capture + CCR3_Val);
  }
  else
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);

    /* Pin PC.09 toggling with frequency = 439.4 Hz */
    //GPIO_WriteBit(GPIOC, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_9)));
    capture = TIM_GetCapture4(TIM2);
    TIM_SetCompare4(TIM2, capture + CCR4_Val);
  }
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    }
}
u32 gcnt = 0;
void RTC_IRQHandler(void)
{

    if(RTC->CRL & (1<<0))
    {
        gcnt++;

        if(gcnt % 2)
            GPIOC->BRR = GPIO_Pin_12;
        else
            GPIOC->BSRR = GPIO_Pin_12;

        RTC->CRL &=~(1<<0);
    }
}
void RTC_Conf(void)
{

  RCC->APB1ENR |= (3<<27); //W��czenie CLK dla PWR i BKP
  PWR->CR |= 0x0100; //ZEZWOLENIE na zapis do BKP registers BDP =1

  //RCC->BDCR |= (1<<16);//Reset rejestr�w Backup na to trzeba uwa�a�
  //RCC->BDCR &= ~(1<<16);

  RCC->BDCR |= (1<<0); //LSE ON (33...kHz)
  while(!(RCC->BDCR & (1<<1)));//External low-speed oscillator ready

  RCC->BDCR |= (1<<8); //LSE zrod�em RTC
  RCC->BDCR |= (1<<15);  //RTC ENABLE

  //---------------------------------------------------conf mode
  while(!(RTC->CRL & 0x0020));//wait for RTOFF
  RTC->CRL |= 0x0010; //Enter conf mode
  RTC->PRLL = 0x7fff;//prescaler 32767+1, signal period 1s
  RTC->CRH |= 0x0001;//second interrupt enable
  RTC->CRL &= ~0x0010; //exit conf mode
  while(!(RTC->CRL & 0x0020));

}
void delay10()
{
    uint32_t temp = 260;
    while(temp--);
}


void ws2812Init(void)
{
	uint16_t PrescalerValue;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Must BE!! if structs declared from stack */
	memset(&TIM_OCInitStructure,0,sizeof(TIM_OCInitStructure));
	memset(&GPIO_InitStructure,0,sizeof(GPIO_InitStructure));
	memset(&TIM_TimeBaseStructure,0,sizeof(TIM_TimeBaseStructure));
	memset(&DMA_InitStructure,0,sizeof(DMA_InitStructure));
	memset(&NVIC_InitStructure,0,sizeof(NVIC_InitStructure));

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
	/* GPIOA Configuration: TIM3 Channel 1 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (72000000 / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 29; // 800kHz
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);// enable Timer 1

	/* configure DMA */
	/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Channel2 Config */
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM1->CCR1; //TIM1_CCR1_Address;	// physical address of Timer 3 CCR1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(ledDmaPtr);		// this is the buffer memory
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						// data shifted from memory to peripheral
	DMA_InitStructure.DMA_BufferSize = sizeofLedBuffer;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_HT, ENABLE);

	/* TIM1 CC1 DMA Request enable */
	TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE);

}


