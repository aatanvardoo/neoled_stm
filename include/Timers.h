/*
 * Timers.h
 *
 *  Created on: Sep 9, 2014
 *      Author: Adi
 */

#ifndef TIMERS_H_
#define TIMERS_H_
#include <stdint.h>
void Delay(volatile uint32_t nTime);
void DelayMs(volatile uint32_t time);
void DelayUs(volatile uint32_t time);
void ConfirureTimerForDelay(void);
void ConfigureTImerForEncoder(void);
uint8_t ReadMinute(void);
void RTC_Conf(void);
void DoAM2302Measurements(void);
void delay10(void);
#endif /* TIMERS_H_ */
