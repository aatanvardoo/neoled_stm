/*
 * led.h
 *
 *  Created on: 12 paź 2015
 *      Author: root
 */

#ifndef INCLUDE_LED_H_
#define INCLUDE_LED_H_

/* One led is half of DMA buffer*/
#define LED_PER_HALF 1

void ws2812Init(void);
void ws2812Send(uint8_t (*color)[3], int len);

#endif /* INCLUDE_LED_H_ */
