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

#endif /* CONF_H_ */
