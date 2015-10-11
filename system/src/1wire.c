#include <stdint.h>
#include "stm32f10x_rcc.h"
#include "Timers.h"
#include "main.h"
extern SFrame frame;
void SetToOutput(uint16_t gpio)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = gpio;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SetToInput(uint16_t gpio)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = gpio;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void Start(void)
{
    volatile u8 iVal = 0;
    u32 loopCnt = 0;
   // Here we send the 'start' sequence to the DHT sensor! We pull the DHT_IO pin low for 10mS, and
   // then high for 300uS. The DHT sensor then responds by pulling the DHT_IO pin low, and then pulling it
   // back high.

   SetToOutput(GPIO_Pin_0);//Output_low(DHT_IO);
   GPIOC->BRR |= 1<<0;
   DelayUs(100);//9ms

   GPIOC->BSRR |= 1<<0;//Output_high(DHT_IO);
   delay10();//SysTimeSleep(30, ESleepType_USec);
   iVal = 0;
   // Here we wait while the DHT_IO pin remains low.....
   SetToInput(GPIO_Pin_0);
   do
   {
       iVal = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);

       loopCnt++;

       if(loopCnt>10000)
           break;

   }while(!iVal);

   loopCnt = 0;
   do
   {
       iVal = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
       loopCnt++;

       if(loopCnt>10000)
           break;

   }while(iVal);
}

static u8 ReadData(void)
{

   // This subroutine reads a byte of data (8 bits) from the DHT sensor...
   u8 i;
   u8 value = 0;
   u8 val = 0;
   u16 loopCnt = 0;

   for(i = 0; i < 8 ; i++)
   {
       do
       {
           val = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);

           loopCnt++;

           if(loopCnt>10000)
               break;

       }while (!val);
       delay10();//SysTimeSleep(10, ESleepType_USec);
       loopCnt = 0;
       val = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
      if (val)
      {
          value = value |(1<<(7 - i));

         do
         {
             val = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
             loopCnt++;

             if(loopCnt>10000)
                 break;

         }while (val);
      }
   }

   return value;
}
u8 DHTData[5];
int16_t temperatura;
u16 wilgotnosc;
void DoAM2302Measurements(void)
{

    u8 i;
    u16 _CRC;
    Start();
    for (i = 0; i < 5 ; i++)
    {
       DHTData[i] = ReadData();
    }
    // Here we calculate the CRC by adding the 1st four bytes, and looking at only the lower 8 bits.
    // This value should match the 5th byte sent by the sensor which is the CRC byte!
    _CRC = DHTData[0] + DHTData[1] + DHTData[2] + DHTData[3];
    _CRC = _CRC & 0xFF;

    if (_CRC != DHTData[4])
    {
        wilgotnosc = 0xDEAD;
        temperatura = 0xDEAD;
    }
    else
    {
        wilgotnosc = DHTData[0]<<8 | DHTData[1];
        temperatura = (DHTData[2]&0x7F)<<8 | DHTData[3];
    }

    frame.temperature = temperatura;
    frame.humidity = wilgotnosc;
}
