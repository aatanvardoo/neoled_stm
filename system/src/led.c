/*
 * led.c
 *
 *  Created on: 12 paź 2015
 *      Author: root
 */
#include "main.h"
#include "led.h"

static union {
    uint8_t buffer[2*LED_PER_HALF*24];
    struct {
        uint8_t begin[LED_PER_HALF*24];
        uint8_t end[LED_PER_HALF*24];
    } __attribute__((packed));
} led_dma;

uint32_t  * const ledDmaPtr =  (uint32_t *)led_dma.buffer;
uint8_t sizeofLedBuffer = sizeof(led_dma.buffer);
static int current_led = 0;
static int total_led = 0;
static uint8_t (*color_led)[3] = NULL;

uint8_t transferIsDone = 0;

static void fillLed(uint8_t *buffer, uint8_t *color)
{
    int i;

    for(i=0; i<8; i++) // GREEN data
	{
	    buffer[i] = ((color[1]<<i) & 0x80)?17:9;
	}
	for(i=0; i<8; i++) // RED
	{
	    buffer[8+i] = ((color[0]<<i) & 0x80)?17:9;
	}
	for(i=0; i<8; i++) // BLUE
	{
	    buffer[16+i] = ((color[2]<<i) & 0x80)?17:9;
	}
}

void ws2812Send(uint8_t (*color)[3], int len)
{
    int i;
	if(len<1) return;

	//Wait for previous transfer to be finished
	while(transferIsDone);

	// Set interrupt context ...
	current_led = 0;
	total_led = len;
	color_led = color;

    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
        if (current_led<total_led)
            fillLed(led_dma.begin+(24*i), color_led[current_led]);
        else
            bzero(led_dma.begin+(24*i), 24);
    }

    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
        if (current_led<total_led)
            fillLed(led_dma.end+(24*i), color_led[current_led]);
        else
            bzero(led_dma.end+(24*i), 24);
    }
    transferIsDone = 1;
	DMA1_Channel2->CNDTR = sizeof(led_dma.buffer); // load number of bytes to be transferred
	DMA_Cmd(DMA1_Channel2, ENABLE); 			// enable DMA channel 2
	TIM_Cmd(TIM1, ENABLE);                      // Go!!!
}

/* DMA chanell 2 interrupt routine */
void DMA1_Channel2_IRQHandler(void)
{

    uint8_t * buffer;
    int i;

    if (total_led == 0)
    {
        TIM_Cmd(TIM1, DISABLE);
    	DMA_Cmd(DMA1_Channel2, DISABLE);
    }

    if (DMA_GetITStatus(DMA1_IT_HT2))
    {
        DMA_ClearITPendingBit(DMA1_IT_HT2);
        buffer = led_dma.begin;
    }

    if (DMA_GetITStatus(DMA1_IT_TC2))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);
        buffer = led_dma.end;
    }

    for(i=0; (i<LED_PER_HALF) && (current_led<total_led+2); i++, current_led++) {
        if (current_led<total_led)
            fillLed(buffer+(24*i), color_led[current_led]);
        else
            bzero(buffer+(24*i), 24);
    }

    if (current_led >= total_led+2) {
    	transferIsDone = 0;

	    TIM_Cmd(TIM1, DISABLE); 					// disable Timer 1
	    DMA_Cmd(DMA1_Channel2, DISABLE); 			// disable DMA channel 2

	    total_led = 0;
    }

}
