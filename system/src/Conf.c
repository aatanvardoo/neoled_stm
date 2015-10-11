/*
 * RCC_conf.c
 *
 *  Created on: Sep 9, 2014
 *      Author: Adi
 */
#include "stm32f10x_rcc.h"
#include "Conf.h"
#include "stm32f10x_usart.h"
#include <string.h>
#include "main.h"
#define MY_UART USART1
static u16 gMsgId = 0;
u32 gcnt2 = 0;
uint8_t gDoMeasurement = 0;

void RCC_Configuration(void)
{
	  /* Enable the GPIO_LED Clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//??????????????
	  /* TIM2 clock enable */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	  /* I2C1 and I2C2 Periph clock enable */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure USART1 Tx (PA.9) as alternate function push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Configure USART1 Rx (PA.10) as input floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

 // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_7 | GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}



void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  memset(&NVIC_InitStructure, 0 , sizeof(NVIC_InitTypeDef));
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  memset(&NVIC_InitStructure, 0 , sizeof(NVIC_InitTypeDef));
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  memset(&NVIC_InitStructure, 0 , sizeof(NVIC_InitTypeDef));
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

 // NVIC->ISER[0] |= 0x00000008;
}

void USART_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    /* USARTy and USARTz configuration ------------------------------------------------------*/
      /* USARTy and USARTz configured as follow:
            - BaudRate = 9600 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
      */
      USART_InitStructure.USART_BaudRate = 9600;
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      USART_InitStructure.USART_Parity = USART_Parity_No;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

      /* Configure USARTy */
      USART_Init(USART2, &USART_InitStructure);

      /* Enable USARTy Receive and Transmit interrupts */
      USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
      //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

      /* Enable the USARTy */
      USART_Cmd(USART2, ENABLE);

}



void SetRTC(sTime *Time)
{

  uint32_t Counter_T = ((Time->RTC_H*3600UL) + (Time->RTC_M*60UL)+(Time->RTC_S));
  uint32_t Counter = (Counter_T*1000UL/1000);
  while(!(RTC->CRL & 0x0020));//wait for RTOFF
  RTC->CRL |= 0x0010; //Enter conf mode,Setting CNF bit
  RTC->CNTH = (Counter>>16) & 0xFFFF;
  RTC->CNTL = Counter & 0xFFFF;
  RTC->CRL &= ~0x0010; //exit conf mode
  while(!(RTC->CRL & 0x0020));

}

void GetRTC(sTime *Time)
{
    uint16_t counterL = 0;
    uint16_t counterH = 0;
    uint32_t counter = 0;
    uint16_t CountedDays=0;

    counterH = RTC->CNTH;
    counterL = RTC->CNTL;
    counter = (counterH<<16) | counterL;

    CountedDays= counter/86400;
    Time->RTC_H = (counter-(CountedDays*86400))/3600;
    Time->RTC_M = (counter%3600)/60;
    Time->RTC_S = (counter%3600)%60;

}


void UsartConf(void)
{
    USART_InitTypeDef USART_InitStructure;
    /* USARTy and USARTz configuration ------------------------------------------------------*/
      /* USARTy and USARTz configured as follow:
            - BaudRate = 9600 baud
            - Word Length = 8 Bits
            - One Stop Bit
            - No parity
            - Hardware flow control disabled (RTS and CTS signals)
            - Receive and transmit enabled
      */
      USART_InitStructure.USART_BaudRate = 115200;
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      USART_InitStructure.USART_Parity = USART_Parity_No;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

      /* Configure USARTy */
      USART_Init(MY_UART, &USART_InitStructure);

      /* Enable USARTy Receive and Transmit interrupts */
      USART_ITConfig(MY_UART, USART_IT_RXNE, ENABLE);
     // USART_ITConfig(MY_UART, USART_IT_TXE, ENABLE);

      /* Enable the USARTy */
      USART_Cmd(MY_UART, ENABLE);

}

void UsartSend(uint8_t Data)
{
    if((MY_UART->SR & (1<<7)))
       MY_UART->DR = Data;
}

void UsartSendText(char *str)
{

  while(*str)
  {
    UsartSend(*str++);
    while(!(MY_UART->SR & (1<<7)));
  }
}
void UARTSend(unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(--ulCount)
    {
        //
        // Write the next character to the UART.
        //
        UsartSend(*pucBuffer++);
        while(!(MY_UART->SR & (1<<7)));
    }
}
void GetMsgId(u8* const data, u16* const msgId)
{
    *msgId = (*msgId)<<8;
    *msgId |= 0x00FF & (*data);
}

void USART1_IRQHandler(void)
{
	uint8_t byte;
	if(USART_GetITStatus(MY_UART, USART_IT_RXNE) != RESET)
	{
		/* Read one byte from the receive data register */
		byte = USART_ReceiveData(MY_UART);
		GetMsgId(&byte, &gMsgId);

		if(gMsgId == 0xABAA)
		{
			gDoMeasurement = 1;

		}

	}

}

void I2C_Dummy_INIT(void)
{
	  I2C_InitTypeDef  I2C_InitStructure;
	  I2C_StructInit(&I2C_InitStructure);

	  /* I2C2 configuration */
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	  //I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDR;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_ClockSpeed = 50000;
	  I2C_Init(I2C1, &I2C_InitStructure);
	  /*enable I2C*/
	  I2C_Cmd(I2C1, ENABLE);
}
