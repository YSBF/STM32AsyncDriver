#include "systemtime.h"

#include "stm32f4xx.h"
//#include "stm32f4xx_it.h"
volatile uint32_t sysTickUptime = 0;
void Delay(__IO uint32_t us) {
  uint32_t start = micros();
  while (micros() - start < (uint32_t) us) {
    //asm('nop');
  }
}


// __STATIC_INLINE void DelayUs(uint32_t us) {
// uint32_t start = micros();
// while (micros()-start < (uint32_t) us) {
// //asm('nop');
// }
// }


void DelayMs(uint16_t nms) {
  uint32_t t0 = micros();
  while (micros() - t0 < nms * 1000);
}

uint32_t millis(void) {
  return sysTickUptime;
}

uint32_t micros(void)//us
{
  register uint32_t ms, cycle_cnt;
  do {
    ms = sysTickUptime;
    cycle_cnt = SysTick->VAL;
  } while (ms != sysTickUptime);
  return (ms * 1000) + (SystemCoreClock - 1000 * cycle_cnt) / (SystemCoreClock / 1000);
}

// __STATIC_INLINE void Delay_us(volatile uint32_t microseconds)
// {
//  uint32_t clk_cycle_start = DWT->CYCCNT;
//  /* Go to number of cycles for system */
//  microseconds *= (SystemCoreClock / 1000000);
//  /* Delay till end */
//  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
// }





// uint32_t DWT_Delay_Init(void) {
//  /* Disable TRC */
//  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
//  /* Enable TRC */
//  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
//  /* Disable clock cycle counter */
//
//	DWT->CTRL |= 1 ; // enable the counter
//	DWT->CYCCNT = 0; // reset the counter
//  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
//  /* Enable clock cycle counter */
//  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
//  /* Reset the clock cycle counter value */
//  DWT->CYCCNT = 0;
// /* 3 NO OPERATION instructions */
////  __asm volatile ('NOP');
////  __asm volatile ('NOP');
////  __asm volatile ('NOP');
//
//  /* Check if clock cycle counter has started */
// if(DWT->CYCCNT)
// {
//  return 0; /*clock cycle counter started*/
// }
// else
//  {
//  return 1; /*clock cycle counter not started*/
//  }
// }


//
//#pragma GCC push_options
//#pragma GCC optimize ("O3")
//void delayUS_DWT(uint32_t us) {
//	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
//	volatile uint32_t start = DWT->CYCCNT;
//	do  {
//	} while(DWT->CYCCNT - start < cycles);
//}
//#pragma GCC pop_options

//
//void TimingDelay_Decrement(void)
//{
//	if (uwTimingDelay != 0x00)
//	{
//		uwTimingDelay--;
//
//	}
//}




















