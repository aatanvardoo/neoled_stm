/*
 * conf.h
 *
 *  Created on: Sep 9, 2014
 *      Author: Adi
 */

#ifndef CONF_H_
#define CONF_H_
#include <stdint.h>
typedef struct
{
  uint32_t RTC_H;
  uint32_t RTC_M;
  uint32_t RTC_S;
}sTime;

typedef enum
{
    second,
    minute,
    minute_10,
    hour,
    hour_10
}ETime;

#define set_h_A GPIOB->BSRR |= 1<<8
#define set_h_B GPIOB->BSRR |= 1<<10
#define set_h_C GPIOB->BSRR |= 1<<11
#define set_h_D GPIOB->BSRR |= 1<<9

#define clr_h_A GPIOB->BRR |= 1<<8
#define clr_h_B GPIOB->BRR |= 1<<10
#define clr_h_C GPIOB->BRR |= 1<<11
#define clr_h_D GPIOB->BRR |= 1<<9

#define set_h10_A GPIOC->BSRR |= 1<<8
#define set_h10_B GPIOC->BSRR |= 1<<10
#define set_h10_C GPIOC->BSRR |= 1<<13
#define set_h10_D GPIOC->BSRR |= 1<<9

#define clr_h10_A GPIOC->BRR |= 1<<8
#define clr_h10_B GPIOC->BRR |= 1<<10
#define clr_h10_C GPIOC->BRR |= 1<<13
#define clr_h10_D GPIOC->BRR |= 1<<9

#define set_min10_A GPIOB->BSRR |= 1<<0
#define set_min10_B GPIOB->BSRR |= 1<<2
#define set_min10_C GPIOB->BSRR |= 1<<5
#define set_min10_D GPIOB->BSRR |= 1<<1

#define clr_min10_A GPIOB->BRR |= 1<<0
#define clr_min10_B GPIOB->BRR |= 1<<2
#define clr_min10_C GPIOB->BRR |= 1<<5
#define clr_min10_D GPIOB->BRR |= 1<<1

#define set_min_A GPIOC->BSRR |= 1<<0
#define set_min_B GPIOC->BSRR |= 1<<2
#define set_min_C GPIOC->BSRR |= 1<<3
#define set_min_D GPIOC->BSRR |= 1<<1

#define clr_min_A GPIOC->BRR |= 1<<0
#define clr_min_B GPIOC->BRR |= 1<<2
#define clr_min_C GPIOC->BRR |= 1<<3
#define clr_min_D GPIOC->BRR |= 1<<1

#define set_sec_A GPIOC->BSRR |= 1<<4
#define set_sec_B GPIOC->BSRR |= 1<<6
#define set_sec_C GPIOC->BSRR |= 1<<7
#define set_sec_D GPIOC->BSRR |= 1<<5

#define clr_sec_A GPIOC->BRR |= 1<<4
#define clr_sec_B GPIOC->BRR |= 1<<6
#define clr_sec_C GPIOC->BRR |= 1<<7
#define clr_sec_D GPIOC->BRR |= 1<<5




void Set_Sec_A(uint8_t set);
void (*Aptr)(uint8_t set);
void Set_Sec_B(uint8_t set);
void (*Bptr)(uint8_t set);
void Set_Sec_C(uint8_t set);
void (*Cptr)(uint8_t set);
void Set_Sec_D(uint8_t set);
void (*Dptr)(uint8_t set);
void Set_Min_A(uint8_t set);
void Set_Min_B(uint8_t set);
void Set_Min_C(uint8_t set);
void Set_Min_D(uint8_t set);
void Set_Min10_A(uint8_t set);
void Set_Min10_B(uint8_t set);
void Set_Min10_C(uint8_t set);
void Set_Min10_D(uint8_t set);

void Set_H_A(uint8_t set);
void Set_H_B(uint8_t set);
void Set_H_C(uint8_t set);
void Set_H_D(uint8_t set);

void Set_H10_A(uint8_t set);
void Set_H10_B(uint8_t set);
void Set_H10_C(uint8_t set);
void Set_H10_D(uint8_t set);

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void RTC_Conf(void);
void SetRTC(sTime *Time);
void GetRTC(sTime *Time);
void set_Sec(ETime type,uint8_t sec);

void ConfigureTImerForEncoder(void);
void UsartConf(void);
void UsartSendText(char *str);
void UARTSend(unsigned char *pucBuffer, unsigned long ulCount);
void I2C_Dummy_INIT(void);
void ws2812Init(void);
void ws2812Send(uint8_t (*color)[3], int len);
#endif /* CONF_H_ */
